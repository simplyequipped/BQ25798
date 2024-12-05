# Datasheet: https://www.ti.com/lit/ds/symlink/bq25798.pdf

import smbus2
import time

# Register Definitions
REG_CHARGE_VOLTAGE = 0x04  # Charge Voltage Control Register
REG_CHARGE_CURRENT = 0x02  # Charge Current Control Register
REG_PRECHARGE_CURRENT = 0x06  # Pre-Charge and Termination Current Control Register
REG_ADC_CTRL = 0x2C        # ADC Control Register
REG_USB_SOURCE_VOLTAGE = 0x16  # USB-C Sourcing Voltage Register
REG_USB_SOURCE_CURRENT = 0x18  # USB-C Sourcing Current Register
REG_USB_SOURCE_CTRL = 0x19     # USB-C Source Control Register
REG_OTG_CONFIG = 0x09          # OTG and Ship Mode Control Register
REG_MPPT_CTRL = 0x1A           # MPPT Control Register
REG_INPUT_SOURCE_CTRL = 0x1B   # Input Source Control Register
REG_MIN_SYS_VOLTAGE = 0x0E     # Minimum System Voltage Control Register
REG_TIMER_CONTROL = 0x12       # Safety Timer and Watchdog Control Register
REG_FAULT_STATUS = 0x20        # Fault Status Register
REG_PROTECTION_CTRL = 0x1C     # Protection Control Register

# ADC Result Registers
REG_ADC_IBUS = 0x34       # Input Current (IBUS) ADC Result
REG_ADC_IBAT = 0x36       # Battery Charge/Discharge Current (IBAT) ADC Result
REG_ADC_VBUS = 0x30       # Input Voltage (VBUS) ADC Result
REG_ADC_VPMID = 0x38      # PMID Voltage ADC Result
REG_ADC_VBAT = 0x32       # Battery Voltage (VBAT) ADC Result
REG_ADC_VSYS = 0x3A       # System Voltage (VSYS) ADC Result
REG_ADC_TS = 0x3C         # External Temperature Sensor (TS) ADC Result
REG_ADC_TDIE = 0x3E       # Die Temperature (TDIE) ADC Result


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

        # Configure ADC and charging parameters
        self.set_adc_one_shot_mode()
        self.set_charge_parameters(charge_voltage, charge_current)
        self.update_adc_readings()

    def _read_register(self, reg_address, reg_bits=16):
        """Read a register value (8-bit or 16-bit)."""
        if reg_bits == 16:
            data = self.bus.read_word_data(self.address, reg_address)
            return (data & 0xFF) << 8 | (data >> 8)  # Swap byte order
        elif reg_bits == 8:
            return self.bus.read_byte_data(self.address, reg_address)
        else:
            raise ValueError("Invalid reg_bits value. Only 8 or 16 are supported.")

    def _write_register(self, reg_address, value, reg_bits=16):
        """Write a register value (8-bit or 16-bit)."""
        if reg_bits == 16:
            data = ((value & 0xFF) << 8) | (value >> 8)  # Swap byte order
            self.bus.write_word_data(self.address, reg_address, data)
        elif reg_bits == 8:
            self.bus.write_byte_data(self.address, reg_address, value)
        else:
            raise ValueError("Invalid reg_bits value. Only 8 or 16 are supported.")

    ### ADC Functions ###
    def set_adc_one_shot_mode(self):
        """Set the ADC to one-shot mode for minimal quiescent current."""
        reg_value = self._read_register(REG_ADC_CTRL)
        reg_value |= (1 << 1)  # Set ADC_ONE_SHOT bit
        self._write_register(REG_ADC_CTRL, reg_value)
        print("ADC set to one-shot mode.")

    def enable_adc(self):
        """Enable the ADC."""
        reg_value = self._read_register(REG_ADC_CTRL)
        reg_value |= (1 << 0)  # Enable ADC
        self._write_register(REG_ADC_CTRL, reg_value)
        print("ADC enabled.")

    def disable_adc(self):
        """Disable the ADC."""
        reg_value = self._read_register(REG_ADC_CTRL)
        reg_value &= ~(1 << 0)  # Disable ADC
        self._write_register(REG_ADC_CTRL, reg_value)
        print("ADC disabled.")

    def update_adc_readings(self):
        """Read all ADC values and update internal class variables."""
        self.enable_adc()
        time.sleep(0.1)  # Allow ADC to complete conversions
        self.ibus = self._read_register(REG_ADC_IBUS) * 50 / 1000  # Convert to amps
        self.ibat = self._read_register(REG_ADC_IBAT) * 64 / 1000  # Convert to amps
        self.vbus = self._read_register(REG_ADC_VBUS) * 16 / 1000  # Convert to volts
        self.vpmid = self._read_register(REG_ADC_VPMID) * 16 / 1000  # Convert to volts
        self.vbat = self._read_register(REG_ADC_VBAT) * 16 / 1000  # Convert to volts
        self.vsys = self._read_register(REG_ADC_VSYS) * 16 / 1000  # Convert to volts
        self.ts = self._read_register(REG_ADC_TS) * 0.2  # Convert to °C
        self.tdie = self._read_register(REG_ADC_TDIE) * 0.2  # Convert to °C
        self.disable_adc()
        print(f"Updated ADC readings: IBUS={self.ibus} A, IBAT={self.ibat} A, VBUS={self.vbus} V, "
              f"VPMID={self.vpmid} V, VBAT={self.vbat} V, VSYS={self.vsys} V, "
              f"TS={self.ts} °C, TDIE={self.tdie} °C.")

    ### MPPT Functions ###
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

    ### Charging Functions ###
    def set_charge_parameters(self, voltage_v, current_a):
        """Set charging voltage and current."""
        voltage_mv = int(voltage_v * 1000)
        current_ma = int(current_a * 1000)
        voltage_reg = voltage_mv // 16  # LSB = 16mV
        current_reg = current_ma // 64  # LSB = 64mA
        self._write_register(REG_CHARGE_VOLTAGE, voltage_reg)
        self._write_register(REG_CHARGE_CURRENT, current_reg)
        print(f"Charge parameters set: {voltage_v} V, {current_a} A.")

    def enable_charging(self):
        """Enable charging."""
        reg_value = self._read_register(REG_PROTECTION_CTRL)
        reg_value |= (1 << 0)  # Enable charging
        self._write_register(REG_PROTECTION_CTRL, reg_value)
        print("Charging enabled.")

    def disable_charging(self):
        """Disable charging."""
        reg_value = self._read_register(REG_PROTECTION_CTRL)
        reg_value &= ~(1 << 0)  # Disable charging
        self._write_register(REG_PROTECTION_CTRL, reg_value)
        print("Charging disabled.")

    ### USB OTG (Source Mode) ###
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

    def enter_shutdown_mode(self):
        """Enter shutdown mode."""
        reg_value = self._read_register(REG_PROTECTION_CTRL)
        reg_value |= (1 << 7)  # Enable shutdown mode
        self._write_register(REG_PROTECTION_CTRL, reg_value)
        print("Shutdown mode enabled.")

    ### Fault Monitoring ###
    def get_fault_status(self):
        """Retrieve fault status."""
        fault_reg = self._read_register(REG_FAULT_STATUS)
        faults = {
            "OVP": bool(fault_reg & (1 << 7)),  # Over-voltage protection fault
            "OCP": bool(fault_reg & (1 << 6)),  # Over-current protection fault
            "OTP": bool(fault_reg & (1 << 5)),  # Over-temperature protection fault
        }
        print(f"Fault status: {faults}")
        return faults

    ### Safety Timers ###
    def configure_safety_timers(self, precharge_timeout=30, fast_charge_timeout=180):
        """
        Configure precharge and fast charge safety timers.

        Args:
            precharge_timeout (int): Precharge timeout in minutes.
            fast_charge_timeout (int): Fast charge timeout in minutes.
        """
        precharge_val = min(max(precharge_timeout, 0), 60) // 10
        fastcharge_val = min(max(fast_charge_timeout, 0), 320) // 40
        timer_reg = (fastcharge_val << 4) | precharge_val
        self._write_register(REG_TIMER_CONTROL, timer_reg, reg_bits=8)
        print(f"Safety timers set: Precharge={precharge_timeout} min, Fast Charge={fast_charge_timeout} min.")

    ### Adapter and Input Management ###
    def set_input_current_limit(self, current_a):
        """Set input current limit."""
        current_ma = int(current_a * 1000)
        input_limit_reg = current_ma // 50  # LSB = 50mA
        self._write_register(REG_INPUT_SOURCE_CTRL, input_limit_reg, reg_bits=8)
        print(f"Input current limit set to {current_a} A.")

    def set_minimum_system_voltage(self, voltage_v):
        """Set minimum system voltage."""
        voltage_mv = int(voltage_v * 1000)
        min_sys_reg = voltage_mv // 16  # LSB = 16mV
        self._write_register(REG_MIN_SYS_VOLTAGE, min_sys_reg)
        print(f"Minimum system voltage set to {voltage_v} V.")
    
