import argparse
import subprocess
import time
import pigpio

from .magnetometer import MagnetometerReader
from .motor import StepperMotor
from .relay import RelayController
from .utils import angle_to_steps, plot_magnetometer_data

MAG_RES_CHOICES = ["4", "8", "12","16"]
ADC_RES_CHOICES = ["0_256", "0_512", "1_024", "2_048", "4_096", "6_144"]

def coil_angle(value):
    fvalue = float(value)
    if fvalue > 90:
        raise argparse.ArgumentTypeError(f"Coil angle must be <= 90, got {fvalue}")
    return fvalue

def parse_args():
    parser = argparse.ArgumentParser(description="Magnetometer scanning system")

    parser.add_argument("--num_points", type=int, default=10,
                        help="Number of measurement points (>=2)")
    
    parser.add_argument("--num_samples", type=int, default=128,
                        help="Number of measurement samples (<=256)")

    parser.add_argument("--relay_12v", action="store_true",
                        help="Use 12V relay")

    parser.add_argument("--relay_on", action="store_true",
                        help="Enable relay")

    parser.add_argument("--magnetometer_resolution",
                        choices=MAG_RES_CHOICES,
                        default="8")

    parser.add_argument("--adc_resolution",
                        choices=ADC_RES_CHOICES,
                        default="0_512")

    parser.add_argument("--coil_angle", type=coil_angle, default=0,
                        help="Coil angle in degrees. Max 90. Default 0.")
    
    parser.add_argument("--start_position", type=int, default=3000,
                        help="Start position in steps. Default 3000.")
    
    parser.add_argument("--max_position", type=int, default=0,
                        help="Max position in steps. Default 0 switch.")

    args = parser.parse_args()

    if args.num_points < 2:
        parser.error("--num-points must be >= 2")

    #if the starting position is out of bounds, when the coil is horizontal
    if (args.coil_angle == 0) and (args.start_position < 0 or args.start_position > 3500):
        parser.error("--start_position must be between 0 and 3500")

    #if the max position is out of bounds, when the coil is horizontal
    if (args.coil_angle == 0) and (args.max_position < 0 or args.max_position > 3500):
        parser.error("--start_position must be between 0 and 3500")
    
    #if the starting position is out of bounds, when the coil is vertical
    if (args.coil_angle != 0) and (args.start_position < 847 or args.start_position > 3500):
        parser.error("--max_position must be >= 847 when coil angle is not 0 or < 3500")

    #if the max position is out of bounds, when the coil is vertical
    if (args.coil_angle != 0) and (args.max_position < 847 or args.max_position > 3500):
        parser.error("--max_position must be >= 847 when coil angle is not 0 or < 3500")
	
    #note that max_position < starting_position, this means that the kart moves towards the coil
    #when starting_position < max_position, then the kart moves away from the coil


    return args


# =========================
# pigpio Management
# =========================
def start_pigpiod():
    subprocess.run(["sudo", "pigpiod"], check=True)
    time.sleep(0.5)
    print("pigpiod started")


def stop_pigpiod():
    subprocess.run(["sudo", "killall", "pigpiod"], check=True)
    print("pigpiod stopped")



def main():
    args = parse_args()
    reader = None

    start_pigpiod()
    pi = pigpio.pi()

    if not pi.connected:
        raise RuntimeError("Failed to connect to pigpiod")

    try:
        # Initialize hardware
        relay = RelayController(pi)
        relay.configure(args.relay_on, args.relay_12v)

        reader = MagnetometerReader(
            pi=pi,
            drdy_pin=13,
            adc_addr=0x48,
            lis3mdl_addr=0x1C,
            max_data_points = args.num_samples,
            magnetometer_resolution=args.magnetometer_resolution,
            adc_resolution=args.adc_resolution
        )

        motor = StepperMotor(
            pi=pi,
            pins=[17, 18, 27, 22],
            switch_pin=14,
            max_steps=3500,
            step_delay=0.002    
        )

        coil = StepperMotor(
            pi=pi,
            pins=[23, 24, 10, 9],
            switch_pin=4,
            step_delay=0.003,
            direction="backward"
        )

        coil.find_max()
        time.sleep(1)
        motor.find_max()
        time.sleep(1)
        motor.move_steps(args.start_position, "backward")
        time.sleep(1)
        

        coil.move_steps(angle_to_steps(args.coil_angle),"backward")
        
        motor.position = 0
        motor.max_position = args.start_position - args.max_position
        
        step_interval = motor.max_position // (args.num_points - 1)
        print(f"Step interval: {step_interval}")

        data_dict = {}
        i = 0

        while i < args.num_points:
            target = i * step_interval
            move = target - motor.position

            if move > 0:
                motor.move_steps(move, "forward")
            elif move < 0:
                motor.move_steps(abs(move), "backward")

            reader.start()
            data = reader.reset()

            if (not reader.repeat) and (reader.repeat_counter == 1):
                data_dict[i] = data
                i += 1
                reader.repeat_counter = 0

        coil.find_max()
        motor.find_max()
        motor.disable()
        reader.stop()
        relay.off()
        start_positions = [i * step_interval for i in range(args.num_points)]
        return_data = {}
        for j, data in data_dict.items():
            df = plot_magnetometer_data(
                data,
                filename=f"magnetic_plot_{j}.png"
            )
            return_data[start_positions[j]] = df
        print("Scan complete!")
    except KeyboardInterrupt:
        print("Interrupted by user")

        if reader:
            data = reader.get_data_copy()
            plot_magnetometer_data(data=data)
            reader.stop()

        motor.disable()
        relay.off()

    except Exception as e:
        print(f"Error: {e}")

        if reader:
            reader.stop()

        motor.disable()
        reader.stop()
        relay.off()
        raise

    finally:
        pi.stop()
        stop_pigpiod()
    return return_data


# =========================
# Entry Point
# =========================
if __name__ == "__main__":
    main()