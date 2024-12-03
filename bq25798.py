import smbus2
import time

# Register Definitions
REG_CHARGE_VOLTAGE = 0x04      # Charge Voltage Control Register
REG_CHARGE_CURRENT = 0x02      # Charge Current Control Register
REG_ADC_CTRL = 0x2C            # ADC Control Register
REG_USB_SOURCE_VOLTAGE = 0x16  # USB-C Sourcing Voltage Register
REG_USB_SOURCE_CURRENT = 0x18  # USB-C Sourcing Current Register
REG_USB_SOURCE_CTRL = 0x19     # USB-C Source Control Register
REG_OTG_CONFIG = 0x09          # OTG and Ship Mode Control Register
REG_MPPT_CTRL = 0x1A           # MPPT Control Register
REG_INPUT_SOURCE_CTRL = 0x1B   # Input Source Control Register

# ADC Result Registers
REG_ADC_IBUS = 0x34            # Input Current (IBUS) ADC Result
REG_ADC_IBAT = 0x36            # Battery Charge/Discharge Current (IBAT) ADC Result
REG_ADC_VBUS = 0x30            # Input Voltage (VBUS) ADC Result
REG_ADC_VPMID = 0x38           # PMID Voltage ADC Result
REG_ADC_VBAT = 0x32            # Battery Voltage (VBAT) ADC Result
REG_ADC_VSYS = 0x3A            # System Voltage (VSYS) ADC Result
REG_ADC_TS = 0x3C              # External Temperature Sensor (TS) ADC Result
REG_ADC_TDIE = 0x3E            # Die Temperature (TDIE) ADC Result


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

        # Initialize ADC readings (class variables)
        self.ibus = 0.0  # Input current (amps)
        self.ibat = 0.0  # Battery current (amps)
        self.vbus = 0.0  # Input voltage (volts)
        self.vpmid = 0.0  # PMID voltage (volts)
        self.vbat = 0.0  # Battery voltage (volts)
        self.vsys = 0.0  # System voltage (volts)
        self.ts = 0.0  # External temperature sensor (°C)
        self.tdie = 0.0  # Die temperature (°C)

        # Configure ADC to one-shot mode and update readings
        self.set_adc_one_shot_mode()
        self.set_charge_parameters(charge_voltage, charge_current)
        self.update_adc_readings()

    def _read_register(self, reg_address, reg_bits=16):
        """
        Read a register value from the device.
    
        Args:
            reg_address (int): The register address to read from.
            reg_bits (int): Number of bits in the register (8 or 16). Default is 16.
        
        Returns:
            int: The value read from the register.
        """
        if reg_bits == 16:
            data = self.bus.read_word_data(self.address, reg_address)
            return (data & 0xFF) << 8 | (data >> 8)  # Swap byte order for 16-bit values
        elif reg_bits == 8:
            return self.bus.read_byte_data(self.address, reg_address)
        else:
            raise ValueError("Invalid reg_bits value. Only 8 or 16 are supported.")
    
    def _write_register(self, reg_address, value, reg_bits=16):
        """
        Write a value to a register.
    
        Args:
            reg_address (int): The register address to write to.
            value (int): The value to write.
            reg_bits (int): Number of bits in the register (8 or 16). Default is 16.
        """
        if reg_bits == 16:
            data = ((value & 0xFF) << 8) | (value >> 8)  # Swap byte order for 16-bit values
            self.bus.write_word_data(self.address, reg_address, data)
        elif reg_bits == 8:
            self.bus.write_byte_data(self.address, reg_address, value)
        else:
            raise ValueError("Invalid reg_bits value. Only 8 or 16 are supported.")

    def enable_adc(self):
        """Enable the ADC."""
        reg_value = self._read_register(REG_ADC_CTRL)
        reg_value |= (1 << 0)  # Set ADC_EN bit to enable ADC
        self._write_register(REG_ADC_CTRL, reg_value)
        print("ADC enabled.")

    def disable_adc(self):
        """Disable the ADC."""
        reg_value = self._read_register(REG_ADC_CTRL)
        reg_value &= ~(1 << 0)  # Clear ADC_EN bit to disable ADC
        self._write_register(REG_ADC_CTRL, reg_value)
        print("ADC disabled.")

    def set_adc_one_shot_mode(self):
        """Set the ADC to one-shot mode for minimal quiescent current."""
        reg_value = self._read_register(REG_ADC_CTRL)
        reg_value |= (1 << 1)  # Set ADC_ONE_SHOT bit to enable one-shot mode
        self._write_register(REG_ADC_CTRL, reg_value)
        print("ADC configured to one-shot mode.")

    def request_adc_update(self):
        """Manually request ADC updates for all measured values."""
        reg_value = self._read_register(REG_ADC_CTRL)
        reg_value |= (1 << 2)  # Set ADC_REQ bit to trigger an update
        self._write_register(REG_ADC_CTRL, reg_value)
        print("ADC update requested.")

    def update_adc_readings(self):
        """
        Read all ADC values from the device and update class variables with human-readable values.
        """
        # Enable ADC, request update, and read values
        self.enable_adc()
        self.request_adc_update()
        time.sleep(0.1)  # Small delay to allow ADC update to complete

        self.ibus = self._read_register(REG_ADC_IBUS) * 50 / 1000  # LSB = 50mA
        self.ibat = self._read_register(REG_ADC_IBAT) * 64 / 1000  # LSB = 64mA
        self.vbus = self._read_register(REG_ADC_VBUS) * 16 / 1000  # LSB = 16mV
        self.vpmid = self._read_register(REG_ADC_VPMID) * 16 / 1000  # LSB = 16mV
        self.vbat = self._read_register(REG_ADC_VBAT) * 16 / 1000  # LSB = 16mV
        self.vsys = self._read_register(REG_ADC_VSYS) * 16 / 1000  # LSB = 16mV
        self.ts = self._read_register(REG_ADC_TS) * 0.2  # LSB = 0.2°C
        self.tdie = self._read_register(REG_ADC_TDIE) * 0.2  # LSB = 0.2°C

        self.disable_adc()  # Disable ADC to minimize quiescent current

        print(f"Updated ADC Readings: IBUS: {self.ibus} A, IBAT: {self.ibat} A, VBUS: {self.vbus} V, "
              f"VPMID: {self.vpmid} V, VBAT: {self.vbat} V, VSYS: {self.vsys} V, "
              f"TS: {self.ts} °C, TDIE: {self.tdie} °C.")

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
        self._write_register(REG_CHARGE_VOLTAGE, voltage_reg)
        self._write_register(REG_CHARGE_CURRENT, current_reg)
        print(f"Charging parameters set: {voltage_v} V, {current_a} A.")

    def enable_charging(self):
        """Enable battery charging."""
        reg_value = self._read_register(REG_OTG_CONFIG)
        reg_value |= (1 << 4)  # Set CHARGE_EN bit
        self._write_register(REG_OTG_CONFIG, reg_value)
        print("Charging enabled.")

    def disable_charging(self):
        """Disable battery charging."""
        reg_value = self._read_register(REG_OTG_CONFIG)
        reg_value &= ~(1 << 4)  # Clear CHARGE_EN bit
        self._write_register(REG_OTG_CONFIG, reg_value)
        print("Charging disabled.")

    def enable_mppt(self, port=2):
        """
        Enable MPPT (Maximum Power Point Tracking) for a specific port.

        Args:
            port (int): The port to enable MPPT for (1 or 2). Default is 2.
        """
        if port not in [1, 2]:
            raise ValueError("Invalid port number. Must be 1 or 2.")

        reg_value = self._read_register(REG_MPPT_CTRL)

        # If MPPT is already enabled on a different port, disable it first
        current_port = (reg_value >> 1) & 0x01
        if current_port != (port - 1):
            reg_value &= ~(1 << 1)  # Clear MPPT_PORT bit
            reg_value &= ~(1 << 0)  # Disable MPPT
            self._write_register(REG_MPPT_CTRL, reg_value)
            print(f"MPPT disabled on port {current_port + 1}.")

        # Enable MPPT for the selected port
        reg_value |= (port - 1) << 1  # Set MPPT_PORT bit
        reg_value |= (1 << 0)  # Enable MPPT
        self._write_register(REG_MPPT_CTRL, reg_value)
        print(f"MPPT enabled on port {port}.")

    def disable_mppt(self):
        """Disable MPPT (Maximum Power Point Tracking)."""
        reg_value = self._read_register(REG_MPPT_CTRL)
        reg_value &= ~(1 << 0)  # Clear MPPT_EN bit
        self._write_register(REG_MPPT_CTRL, reg_value)
        print("MPPT disabled.")

    def enable_usb_sourcing(self, voltage_v=5.0, current_a=3.0, port=1):
        """
        Enable USB-C sourcing with specified voltage, current, and port.

        Args:
            voltage_v (float): Sourcing voltage in volts. Default is 5V.
            current_a (float): Sourcing current in amps. Default is 3A.
            port (int): The port to enable sourcing on (1 or 2). Default is 1.
        """
        if port not in [1, 2]:
            raise ValueError("Invalid port number. Must be 1 or 2.")

        reg_value = self._read_register(REG_USB_SOURCE_CTRL)

        # If sourcing is already enabled on a different port, disable it first
        current_port = (reg_value >> 1) & 0x01
        if current_port != (port - 1):
            reg_value &= ~(1 << 1)  # Clear SOURCE_PORT bit
            reg_value &= ~(1 << 0)  # Disable sourcing
            self._write_register(REG_USB_SOURCE_CTRL, reg_value)
            print(f"USB sourcing disabled on port {current_port + 1}.")

        # Set sourcing voltage and current
        voltage_mv = int(voltage_v * 1000)
        current_ma = int(current_a * 1000)
        voltage_reg = int(voltage_mv / 16)  # Voltage LSB is 16mV
        current_reg = int(current_ma / 50)  # Current LSB is 50mA
        self._write_register(REG_USB_SOURCE_VOLTAGE, voltage_reg)
        self._write_register(REG_USB_SOURCE_CURRENT, current_reg)

        # Enable sourcing on the selected port
        reg_value |= (port - 1) << 1  # Set SOURCE_PORT bit
        reg_value |= (1 << 0)  # Enable sourcing
        self._write_register(REG_USB_SOURCE_CTRL, reg_value)
        print(f"USB sourcing enabled: {voltage_v} V, {current_a} A on port {port}.")

    def disable_usb_sourcing(self):
        """Disable USB-C power sourcing."""
        reg_value = self._read_register(REG_USB_SOURCE_CTRL)
        reg_value &= ~(1 << 0)  # Clear SOURCE_EN bit
        self._write_register(REG_USB_SOURCE_CTRL, reg_value)
        print("USB-C sourcing disabled.")

    def enter_ship_mode(self, delay_enabled=False):
        """
        Enter ship mode to minimize quiescent current.
    
        Args:
            delay_enabled (bool): If True, enable the BQ25798's built-in 10-second delay 
                                  before entering ship mode. Default is False.
        """
        reg_value = self._read_register(REG_OTG_CONFIG)
        
        if delay_enabled:
            reg_value |= (1 << 6)  # Set SHIP_DLY_EN bit to enable 10-second delay
        else:
            reg_value &= ~(1 << 6)  # Clear SHIP_DLY_EN bit to disable 10-second delay
    
        reg_value |= (1 << 5)  # Set SHIP_MODE bit to enter ship mode
        self._write_register(REG_OTG_CONFIG, reg_value)
        
        print(f"Ship mode enabled. 10-second delay: {'enabled' if delay_enabled else 'disabled'}.")

    def exit_ship_mode(self):
        """Exit ship mode to enable normal charger operations."""
        reg_value = self._read_register(REG_OTG_CONFIG)
        reg_value &= ~(1 << 5)  # Clear SHIP_MODE bit
        self._write_register(REG_OTG_CONFIG, reg_value)
        print("Exited ship mode.")

    def close(self):
        """Close the I²C bus."""
        self.bus.close()
        print("I²C bus closed.")
    
