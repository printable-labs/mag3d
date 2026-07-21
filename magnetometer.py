import pigpio
import time
from queue import Queue, Full, Empty
import smbus2

from .utils import adc_resolution_to_hex, adc_resolution_to_vfs, magnetometer_resolution_to_range

import queue


class ADS1115_wDRDY:
    """
    ADS1115 single-shot reader using ALERT/RDY as DRDY.
    DRDY interrupt pushes timestamps into a queue.
    read() consumes queue and reads conversion result.
    """

    def __init__(self, i2c_bus: int = 1, i2c_addr: int = 0x49, drdy_pin: int = 17, queue_size: int = 100, adc_resolution = "0_512"):
        self.i2c_addr = i2c_addr
        self.bus = smbus2.SMBus(i2c_bus)

        # pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running")

        self.drdy_pin = drdy_pin
        self.pi.set_mode(self.drdy_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.drdy_pin, pigpio.PUD_UP)

        # Queue for DRDY events (timestamps)
        self._queue = queue.Queue(maxsize=queue_size)

        # Register callback
        self._cb = self.pi.callback(
            self.drdy_pin,
            pigpio.FALLING_EDGE,
            self._drdy_callback
        )

        # Registers
        self._reg_conversion = 0x00
        self._reg_config     = 0x01
        self._reg_lo_thresh  = 0x02
        self._reg_hi_thresh  = 0x03

        # Config
        self._os_start_single  = 0x8000
        self._mux_ain0_ain1    = 0x0000
        self._pga_bits         = adc_resolution_to_hex(adc_resolution)
        self._mode_single_shot = 0x0100
        self._rate_475         = 0x00E0

        # Comparator → DRDY mode
        self._comp_mode_traditional = 0x0000
        self._comp_pol_active_low   = 0x0000
        self._comp_lat_non_latch    = 0x0000
        self._comp_que_assert       = 0x0000

        self._config_word = (
            self._mux_ain0_ain1 |
            self._pga_bits |
            self._mode_single_shot |
            self._rate_475 |
            self._comp_mode_traditional |
            self._comp_pol_active_low |
            self._comp_lat_non_latch |
            self._comp_que_assert
        )

        # Configure ALERT/RDY behavior
        self._write_register(self._reg_lo_thresh, 0x0000)
        self._write_register(self._reg_hi_thresh, 0xFFFF)

        # Voltage scaling
        self._vfs = adc_resolution_to_vfs(adc_resolution)
        self._max_count  = 32768
        self._volt_scale = 1

    # ── Public API ───────────────────────────────────────────────────────────

    def start_conversion(self):
        """Trigger a single conversion."""
        self._write_register(
            self._reg_config,
            self._config_word | self._os_start_single
        )

    def read(self, timeout: float = None):
        """
        Wait for DRDY event from queue, then read ADC.

        Returns:
            (timestamp, voltage)
        """
        self.start_conversion()
        try:
            _ = self._queue.get(timeout=timeout)
        except queue.Empty:
            raise TimeoutError("No DRDY event received")

        raw = self._read_register(self._reg_conversion)
        voltage = (raw * self._vfs / self._max_count) * self._volt_scale

        return voltage

    def close(self):
        if self._cb:
            self._cb.cancel()
        if self.bus:
            self.bus.close()

    # ── Internal ─────────────────────────────────────────────────────────────

    def _drdy_callback(self, gpio, level, tick):
        """
        Called on DRDY falling edge.
        Push timestamp into queue (non-blocking).
        """

        try:
            self._queue.put_nowait(True)
        except queue.Full:
            # Drop oldest or ignore (design choice)
            pass

    def _write_register(self, reg: int, value: int):
        self.bus.write_i2c_block_data(
            self.i2c_addr,
            reg,
            [(value >> 8) & 0xFF, value & 0xFF]
        )

    def _read_register(self, reg: int) -> int:
        data = self.bus.read_i2c_block_data(self.i2c_addr, reg, 2)
        raw = (data[0] << 8) | data[1]
        return raw - 65536 if raw > 32767 else raw

class MagnetometerReader:
    def __init__(self,pi: pigpio.pi, drdy_pin=6,lis3mdl_addr = 0x1C, adc_addr = 0x49, max_data_points = 128, max_queue=256, magnetometer_resolution = "8", adc_resolution = "0_512"):
        # Initialize sensor
        from board import I2C
        from adafruit_lis3mdl import LIS3MDL, Rate, Range

        i2c = I2C()
        self.sensor = LIS3MDL(i2c, address= lis3mdl_addr)
        self.adc = ADS1115_wDRDY(i2c_addr=adc_addr, drdy_pin=25, adc_resolution=adc_resolution)
        self.sensor.data_rate = Rate.RATE_155_HZ
        self.sensor.range = magnetometer_resolution_to_range(magnetometer_resolution)
        self.sensor.enable_data_ready = True

        self.drdy_pin = drdy_pin
        self.max_queue = max_queue

        # pigpio daemon connection
        self.pi = pi
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running. Start with: sudo pigpiod")

        self.pi.set_mode(self.drdy_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.drdy_pin, pigpio.PUD_DOWN)
        self.max_data_points = max_data_points
        # Queue and data store
        self.sample_queue = Queue(maxsize=max_queue)
        self.max_queue = max_queue
        self.data = []
        self.data_count = 0
        self.running = False
        self._cb = None
        self.repeat = False
        self.repeat_counter = 0

    def _drdy_callback(self, gpio, level, tick):
        """
        Called by pigpio on rising edge.
        
        `tick` is a 32-bit microsecond counter from the pigpio daemon,
        timestamped in the DMA/hardware layer — not the OS scheduler.
        It wraps around every ~71.6 minutes.
        """
        if level != 1:  # Only care about rising edge
            return
        
        try:
            # Convert tick (µs, wrapping uint32) to a float seconds timestamp
            # anchored to the same epoch as time.perf_counter() via self._t0_offset
            self.sample_queue.put_nowait(self._tick_to_seconds(tick))
        except Full:
            pass  # Drop sample rather than block

    def _tick_to_seconds(self, tick):
        """
        Converts pigpio tick (µs, wrapping uint32) to seconds,
        aligned to perf_counter epoch set at start().
        """
        # Handle 32-bit wraparound
        elapsed_us = pigpio.tickDiff(self._start_tick, tick)
        return self._start_perf + elapsed_us * 1e-6
    

    def start(self):
        
        # Anchor pigpio tick to perf_counter so timestamps are comparable
        self._start_tick = self.pi.get_current_tick()
        self._start_perf = time.perf_counter()

        # Register pigpio callback — edge timestamp is captured in hardware
        self._cb = self.pi.callback(
            self.drdy_pin,
            pigpio.RISING_EDGE,
            self._drdy_callback
        )
        self.repeat = False
        _ = self.sensor.magnetic  # Flush first sample
        self.running = True
        while self.running:
            try:
                t = self.sample_queue.get(timeout=1000)
            except Empty:
                continue
            
            mx, my, mz = self.sensor.magnetic
            v_shunt = self.adc.read() 
            self.data_count += 1
            self.data.append((t, mx, my, mz, v_shunt))
            if self.data_count >= self.max_data_points :
                break
    def stop(self):
        self.running = False
        if self._cb:
            self._cb.cancel()
        self.adc.close()
        

    def reset(self):
        """Pause collection, return data, and clear buffers."""
        self.running = False
        if self._cb:
            self._cb.cancel()
            self._cb = None

        return_data = self.get_data_copy()

        if return_data:
            seconds = [sample[0] for sample in return_data]

            timestamps_sec = []

            for i in range(len(seconds)):
                # Convert to seconds relative to start
                timestamps_sec.append(seconds[i])

                # Check gap (from second sample onward)
                if i > 0:
                    dt_s = timestamps_sec[i] - timestamps_sec[i-1]

                    if dt_s > 0.01:
                        self.repeat = True
                        self.repeat_counter = 0
            if self.repeat == False:
                self.repeat_counter +=1
                print("Aquisition completed")
        self.data = []
        self.data_count = 0

        # Drain queue
        while not self.sample_queue.empty():
            try:
                self.sample_queue.get_nowait()
            except Empty:
                break

        return return_data

    def get_data_copy(self):
        return list(self.data)


