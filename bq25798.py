import smbus2
import time


class BQ25798:
    def __init__(self, i2c_bus=1, i2c_address=0x6B, charge_voltage=14.2, charge_current=3.0):
        """
        Initialize the BQ25798 module.

        Args:
            i2c_bus (int): The I²C bus number (default 1 for Raspberry Pi).
            i2c_address (int): The I²C address of the BQ25798 (default 0x6B).
            charge_voltage (float): Battery charge voltage in volts. Default is 14.2V.
            charge_current (float): Battery charge current in amps. Default is 3.0A.
        """
        self.bus = smbus2.SMBus(i2c_bus)
        self.address = i2c_address
        self.configure_adc_one_shot()
        self.set_charge_parameters(charge_voltage, charge_current)

    def _read_register(self, reg_address):
        """Read a 16-bit register value from the device."""
        data = self.bus.read_word_data(self.address, reg_address)
        return (data & 0xFF) << 8 | (data >> 8)

    def _write_register(self, reg_address, value):
        """Write a 16-bit value to a register."""
        data = ((value & 0xFF) << 8) | (value >> 8)
        self.bus.write_word_data(self.address, reg_address, data)

    def configure_adc_one_shot(self):
        """Configure the ADC to one-shot mode for minimal quiescent current."""
        reg_value = self._read_register(0x2C)  # REG_ADC_CTRL (example address)
        reg_value &= ~(1 << 0)  # Clear ADC_EN bit to disable continuous mode
        reg_value |= (1 << 1)  # Set ADC_ONE_SHOT bit to enable one-shot mode
        self._write_register(0x2C, reg_value)
        print("ADC configured to one-shot mode.")

    def request_adc_update(self):
        """Manually request ADC updates for all measured values."""
        reg_value = self._read_register(0x2C)  # REG_ADC_CTRL
        reg_value |= (1 << 2)  # Set ADC_REQ bit to trigger an update
        self._write_register(0x2C, reg_value)
        print("ADC update requested.")

    def set_charge_parameters(self, voltage_v, current_a):
        """
        Set the battery charging voltage and current.

        Args:
            voltage_v (float): Charging voltage in volts.
            current_a (float): Charging current in amps.
        """
        voltage_mv = int(voltage_v * 1000)
        current_ma = int(current_a * 1000)
        voltage_reg = int(voltage_mv / 16)  # Voltage LSB is 16mV
        current_reg = int(current_ma / 64)  # Current LSB is 64mA
        self._write_register(0x04, voltage_reg)  # Charge voltage register
        self._write_register(0x02, current_reg)  # Charge current register
        print(f"Charging parameters set: {voltage_v} V, {current_a} A.")

    def enable_charging(self):
        """Enable battery charging."""
        reg_value = self._read_register(0x09)
        reg_value |= (1 << 4)  # Set CHARGE_EN bit
        self._write_register(0x09, reg_value)
        print("Charging enabled.")

    def disable_charging(self):
        """Disable battery charging."""
        reg_value = self._read_register(0x09)
        reg_value &= ~(1 << 4)  # Clear CHARGE_EN bit
        self._write_register(0x09, reg_value)
        print("Charging disabled.")

    def enable_usb_sourcing(self, voltage_v=5.0, current_a=3.0):
        """
        Enable USB-C sourcing with specified voltage and current.

        Args:
            voltage_v (float): Sourcing voltage in volts. Default is 5V.
            current_a (float): Sourcing current in amps. Default is 3A.
        """
        voltage_mv = int(voltage_v * 1000)
        current_ma = int(current_a * 1000)
        voltage_reg = int(voltage_mv / 16)  # Voltage LSB is 16mV
        current_reg = int(current_ma / 50)  # Current LSB is 50mA
        self._write_register(0x16, voltage_reg)  # Sourcing voltage register
        self._write_register(0x18, current_reg)  # Sourcing current register
        reg_value = self._read_register(0x19)
        reg_value |= (1 << 0)  # Enable sourcing bit
        self._write_register(0x19, reg_value)
        print(f"USB-C sourcing enabled: {voltage_v} V, {current_a} A.")

    def disable_usb_sourcing(self):
        """
        Disable USB-C power sourcing and return to adapter sink mode.
        """
        reg_value = self._read_register(0x19)
        reg_value &= ~(1 << 0)  # Clear sourcing enable bit
        self._write_register(0x19, reg_value)
        print("USB-C sourcing disabled. Adapter sink mode re-enabled.")

    def enter_ship_mode(self):
        """Enter ship mode to minimize quiescent current."""
        reg_value = self._read_register(0x09)  # OTG_CONFIG register
        reg_value |= (1 << 5)  # Set SHIP_MODE bit
        self._write_register(0x09, reg_value)
        print("Ship mode enabled.")

    def exit_ship_mode(self):
        """Exit ship mode to enable normal charger operations."""
        reg_value = self._read_register(0x09)
        reg_value &= ~(1 << 5)  # Clear SHIP_MODE bit
        self._write_register(0x09, reg_value)
        print("Exited ship mode.")

    def power_on_system(self):
        """Simulates powering on the system. Assumes the QON pin is handled externally."""
        self.exit_ship_mode()
        print("System powered on.")

    def power_off_system(self, delay_seconds=10):
        """
        Simulates powering off the system by entering ship mode.

        Args:
            delay_seconds (int): Delay to allow graceful system shutdown.
        """
        print(f"Shutting down system in {delay_seconds} seconds...")
        time.sleep(delay_seconds)
        self.enter_ship_mode()
        print("System powered off.")

    def close(self):
        """Close the I²C bus."""
        self.bus.close()
        print("I²C bus closed.")
