import time
from machine import I2C, Pin

class INA219:
    # Standard registers for the INA219
    REG_CONFIG = 0x00
    REG_SHUNT_VOLTAGE = 0x01
    REG_BUS_VOLTAGE = 0x02
    REG_POWER = 0x03
    REG_CURRENT = 0x04
    REG_CALIBRATION = 0x05

    def __init__(self, i2c_bus, address=0x41):
        """
        Initialize the INA219. 
        Note: Many 3S UPS modules use address 0x43. Standard modules use 0x40.
        """
        self.i2c = i2c_bus
        self.address = address
        
        # Calibration for 32V, 2A Max
        self.cal_value = 4096 
        self.current_lsb = 0.1 # 100uA per bit
        self.power_lsb = 0.002 # 2mW per bit
        
        self.configure()

    def _write_register(self, reg, value):
        # Break the 16-bit value into two 8-bit bytes
        data = bytearray([(value >> 8) & 0xFF, value & 0xFF])
        self.i2c.writeto_mem(self.address, reg, data)

    def _read_register(self, reg):
        # Read 2 bytes from the register
        data = self.i2c.readfrom_mem(self.address, reg, 2)
        return int.from_bytes(data, 'big')

    def _read_signed_register(self, reg):
        val = self._read_register(reg)
        if val > 32767:
            val -= 65536
        return val

    def configure(self):
        # Write calibration
        self._write_register(self.REG_CALIBRATION, self.cal_value)
        # Config: 32V range, Gain 8 (320mV), 12-bit ADC, Continuous
        self._write_register(self.REG_CONFIG, 0x399F)

    def get_bus_voltage_V(self):
        # Shift right 3 bits, multiply by 4mV (0.004)
        val = self._read_register(self.REG_BUS_VOLTAGE)
        return ((val >> 3) * 4) / 1000.0

    def get_current_mA(self):
        # The INA219 sometimes resets the calibration register, so we rewrite it
        self._write_register(self.REG_CALIBRATION, self.cal_value)
        val = self._read_signed_register(self.REG_CURRENT)
        return val * self.current_lsb

    def get_power_mW(self):
        self._write_register(self.REG_CALIBRATION, self.cal_value)
        val = self._read_register(self.REG_POWER)
        return val * (self.power_lsb * 1000)

# ==========================================
# Main Execution Script
# ==========================================

print("Initializing I2C...")
# Setup I2C on Pico pins GP0 (SDA) and GP1 (SCL)
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)

# Scan for I2C devices to confirm the UPS is connected
devices = i2c.scan()
if devices:
    print("I2C devices found at addresses:", [hex(d) for d in devices])
else:
    print("No I2C devices found! Check wiring.")

# Initialize the sensor. 
# CHANGE 0x43 to 0x40 or 0x41 if your scan output shows a different address!
ina = INA219(i2c, address=0x41)

print("Starting UPS Monitor...")
try:
    while True:
        voltage = ina.get_bus_voltage_V()
        current = ina.get_current_mA()
        power = ina.get_power_mW()

        # A 3S battery is fully charged at ~12.6V and empty around ~9.0V
        print(f"Battery Voltage: {voltage:.2f} V")
        print(f"Current Draw:    {current:.2f} mA")
        print(f"Power Usage:     {power:.2f} mW")
        print("-" * 30)
        
        time.sleep(1)

except KeyboardInterrupt:
    print("\nMonitoring stopped.")