# Datasheet: https://www.ti.com/lit/ds/symlink/bq25798.pdf

import time
import threading
import logging
from logging.handlers import RotatingFileHandler

import smbus2

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


# Create a logger
logger = logging.getLogger('bq25798_log')
logger.setLevel(logging.DEBUG)
# Create a rotating file handler (max size: 1MB, keep 3 backups)
handler = RotatingFileHandler('bq25798.log', maxBytes=1_000_000, backupCount=3)
handler.setLevel(logging.DEBUG)
# Add formatter
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
# Add the handler to the logger
logger.addHandler(handler)

#TODO Log messages
#logging.debug('This is a debug message')
#logging.info('This is an info message')
#logging.warning('This is a warning message')
#logging.error('This is an error message')
#logging.critical('This is a critical message')



class BQ25798:
    def __init__(self, i2c_bus=1, i2c_address=0x6B, charge_voltage=14.2, charge_current=3.0, adc_update_seconds=3):
        '''
        Initialize the BQ25798 module.

        Args:
            i2c_bus (int): I2C bus number, defaults to 1
            i2c_address (int): I2C address of the BQ25798, defaults 0x6B
            charge_voltage (float): Battery charge voltage in volts, defaults to 14.2
            charge_current (float): Battery charge current in amps, defaults to 3.0
            adc_update_seconds (int): How often to update ADC readings, defaults to 3
        '''
        self.bus = smbus2.SMBus(i2c_bus)
        self.address = i2c_address

        # initialize ADC readings
        self.vbus = 0.0  # input voltage (volts)
        self.ibus = 0.0  # input current (amps)
        self.vbat = 0.0  # battery voltage (volts)
        self.ibat = 0.0  # battery current (amps)
        self.vpmid = 0.0  # PMID voltage (volts)
        self.vsys = 0.0  # system voltage (volts)
        self.ts = 0.0  # external temperature sensor (°C)
        self.tdie = 0.0  # die temperature (°C)

        logging.info(f'BQ25798 initalized with I2C address {self.address}')

        self.set_charge_parameters(charge_voltage, charge_current)
        self.set_adc_one_shot_mode()
        self.update_adc_readings()
        # start thread to periodically update ADC values
        #TODO should this happen in module or should application perform looping function?
        adc_thread = threading.Thread(target=self._adc_loop, args=(adc_update,), daemon=True)
        adc_thread.start()

    def _read_register(self, reg_address, reg_bits=16):
        '''Read register value

        Args:
            reg_address (int): Register address to read (example: 0x20)
            reg_bits (int): Number of bits in the specified register (8 or 16), defaults to 16

        Raises:
            ValueError: Invalid reg_bits value, only 8 or 16 are supported
        '''
        if reg_bits == 16:
            data = self.bus.read_word_data(self.address, reg_address)
            return (data & 0xFF) << 8 | (data >> 8)  # Swap byte order
        elif reg_bits == 8:
            return self.bus.read_byte_data(self.address, reg_address)
        else:
            raise ValueError('Invalid reg_bits value, only 8 or 16 are supported')

    def _write_register(self, reg_address, value, reg_bits=16):
        '''Write register value

        Args:
            reg_address (int): Register address to read (example: 0x20)
            reg_bits (int): Number of bits in the specified register (8 or 16), defaults to 16

        Raises:
            ValueError: Invalid reg_bits value, only 8 or 16 are supported
        '''
        if reg_bits == 16:
            data = ((value & 0xFF) << 8) | (value >> 8)  # Swap byte order
            self.bus.write_word_data(self.address, reg_address, data)
        elif reg_bits == 8:
            self.bus.write_byte_data(self.address, reg_address, value)
        else:
            raise ValueError('Invalid reg_bits value, only 8 or 16 are supported')

    def close(self):
        '''Close the I2C bus connection.'''
        if self.bus:
            self.bus.close()
            print('I2C bus closed')
    
    
    ### ADC Functions ###
    def enable_adc(self):
        '''Enable ADC'''
        reg_value = self._read_register(REG_ADC_CTRL)
        reg_value |= (1 << 0)  # Enable ADC
        self._write_register(REG_ADC_CTRL, reg_value)
        logging.info('ADC enabled')

    def disable_adc(self):
        '''Disable ADC'''
        reg_value = self._read_register(REG_ADC_CTRL)
        reg_value &= ~(1 << 0)  # Disable ADC
        self._write_register(REG_ADC_CTRL, reg_value)
        logging.info('ADC disabled')

    def adc_enabled(self):
        '''
        Check if ADC is enabled.
    
        Returns:
            bool: True if the ADC is enabled (in either continuous or one-shot mode), False otherwise
        '''
        return self.get_adc_mode() in ('continuous', 'one-shot')
        
    def set_adc_one_shot_mode(self):
        '''Set ADC to one-shot mode for minimal quiescent current'''
        reg_value = self._read_register(REG_ADC_CTRL)
        reg_value |= (1 << 1)  # Set ADC_ONE_SHOT bit
        self._write_register(REG_ADC_CTRL, reg_value)
        logging.info('ADC set to one-shot mode')

    def set_adc_continuous_mode(self):
        '''Set the ADC to continuous mode.'''
        reg_value = self._read_register(REG_ADC_CTRL, reg_bits=8)
        reg_value &= ~(1 << 1)  # clear the ADC_ONE_SHOT bit
        reg_value |= (1 << 0)  # set the ADC_EN bit
        self._write_register(REG_ADC_CTRL, reg_value, reg_bits=8)
        logging.info('ADC set to continuous mode')

    def get_adc_mode(self):
        '''
        Get current ADC mode (continuous or one-shot).
    
        Returns:
            str: 'continuous' if ADC is in continuous mode, 'one-shot' otherwise
        '''
        reg_value = self._read_register(REG_ADC_CTRL, reg_bits=8)
        if reg_value & (1 << 0):  # check if ADC_EN bit is set
            if reg_value & (1 << 1):  # check if ADC_ONE_SHOT bit is set
                return 'one-shot'
            else:
                return 'continuous'
        else:
            return 'disabled'

    def update_adc_readings(self):
        '''Read all ADC values and update class variables'''
        self.enable_adc()
        time.sleep(0.1)  # allow ADC to complete conversions
        self.ibus = self._read_register(REG_ADC_IBUS) * 50 / 1000  # convert to A, 50mA per bit scaling factor
        self.ibat = self._read_register(REG_ADC_IBAT) * 64 / 1000  # convert to A, 64mA per bit scaling factor
        self.vbus = self._read_register(REG_ADC_VBUS) * 16 / 1000  # convert to V, 16mV per bit scaling factor
        self.vpmid = self._read_register(REG_ADC_VPMID) * 16 / 1000  # convert to V, 16mV per bit scaling factor
        self.vbat = self._read_register(REG_ADC_VBAT) * 16 / 1000  # convert to V, 16mV per bit scaling factor
        self.vsys = self._read_register(REG_ADC_VSYS) * 16 / 1000  # convert to V, 16mV per bit scaling factor
        self.ts = self._read_register(REG_ADC_TS) * 0.2  # convert to °C, 0.2°C per bit scaling factor
        self.tdie = self._read_register(REG_ADC_TDIE) * 0.2  # convert to °C, 0.2°C per bit scaling factor
        self.disable_adc()
        
        logging.info(f'Updated ADC values: IBUS={self.ibus}A, IBAT={self.ibat}A, VBUS={self.vbus}V, VPMID={self.vpmid}V, VBAT={self.vbat}V, VSYS={self.vsys}V, TS={self.ts}°C, TDIE={self.tdie}°C')

    def _adc_loop(self, seconds):
        '''
        Loop to periodically update ADC readings.

        Args:
            seconds (int): How often to update ADC readings
        '''
        # SMBus.fd is set to None after SMBus.close() is called
        while self.bus.fd is not None:
            time.sleep(seconds)
            self.update_adc_readings()

    
    ### Solar MPPT Input ###
    def enable_mppt(self, port=2):
        '''
        Enable MPPT (Maximum Power Point Tracking) for a specific port.

        Args:
            port (int): Port to enable MPPT for (1 or 2), defaults to 2

        Raises:
            ValueError: Invalid port number, must be 1 or 2
        '''
        if port not in (1, 2):
            raise ValueError('Invalid port number, must be 1 or 2')

        reg_value = self._read_register(REG_MPPT_CTRL)

        # if MPPT is already enabled on a different port, disable it first
        current_port = (reg_value >> 1) & 0x01
        if current_port != (port - 1):
            reg_value &= ~(1 << 1)  # clear MPPT_PORT bit
            reg_value &= ~(1 << 0)  # disable MPPT
            self._write_register(REG_MPPT_CTRL, reg_value)
            print(f'MPPT disabled on port {current_port + 1}.')

        # enable MPPT for the selected port
        reg_value |= (port - 1) << 1  # set MPPT_PORT bit
        reg_value |= (1 << 0)  # enable MPPT
        self._write_register(REG_MPPT_CTRL, reg_value)
        logging.info(f'MPPT enabled on port {port}.')

    def disable_mppt(self):
        '''Disable MPPT (Maximum Power Point Tracking).'''
        reg_value = self._read_register(REG_MPPT_CTRL)
        reg_value &= ~(1 << 0)  # clear MPPT_EN bit
        self._write_register(REG_MPPT_CTRL, reg_value)
        logging.info('MPPT disabled')

    def mppt_enabled(self):
        '''
        Check if MPPT is enabled.
    
        Returns:
            bool: True if MPPT is enabled, False otherwise
        '''
        reg_value = self._read_register(REG_MPPT_CTRL, reg_bits=8)
        return bool(reg_value & (1 << 0))  # check if the MPPT_EN bit (bit 0) is set

    
    ### Battery Charging ###
    def get_charge_parameters(self):
        '''
        Get configured battery charging voltage and current.
    
        Returns:
            tuple: A tuple containing charge voltage (in volts) and charge current (in amps)
        '''
        voltage_reg = self._read_register(REG_CHARGE_VOLTAGE)
        current_reg = self._read_register(REG_CHARGE_CURRENT)
    
        charge_voltage = voltage_reg * 0.016  # scale 16mV per bit to volts
        charge_current = current_reg * 0.064  # scale 64mA per bit to amps
    
        return charge_voltage, charge_current

    def set_charge_parameters(self, voltage_v, current_a):
        '''Set battery charging voltage and current.

        Args:
            voltage_v (float): Battery charge voltage in volts
            current_a (float): Battery charge current in amps
        '''
        voltage_mv = int(voltage_v * 1000)
        current_ma = int(current_a * 1000)
        voltage_reg = voltage_mv // 16  # LSB = 16mV
        current_reg = current_ma // 64  # LSB = 64mA
        self._write_register(REG_CHARGE_VOLTAGE, voltage_reg)
        self._write_register(REG_CHARGE_CURRENT, current_reg)
        logging.info(f'Battery charge parameters set: {voltage_v}V, {current_a}A')

    def enable_charging(self):
        '''Enable battery charging.'''
        reg_value = self._read_register(REG_PROTECTION_CTRL)
        reg_value |= (1 << 0)  # Enable charging
        self._write_register(REG_PROTECTION_CTRL, reg_value)
        print('Charging enabled.')

    def disable_charging(self):
        '''Disable battery charging.'''
        reg_value = self._read_register(REG_PROTECTION_CTRL)
        reg_value &= ~(1 << 0)  # Disable charging
        self._write_register(REG_PROTECTION_CTRL, reg_value)
        print('Charging disabled.')
        
    def charging_enabled(self):
        '''
        Whether battery charging enabled.
    
        Returns:
            bool: True if charging is enabled, False otherwise
        '''
        reg_value = self._read_register(REG_PROTECTION_CTRL, reg_bits=8)
        return bool(reg_value & (1 << 0))  # check if the CHARGE_EN bit (bit 0) is set

    def charge_status(self, battery_chemistry, num_cells, battery_capacity_ah, min_battery_voltage=None):
        '''
        Get the current charge status of the battery.
    
        Args:
            battery_chemistry (str): Battery chemistry ('LiFePO4', 'Li-ion', 'NiMH')
            num_cells (int): Number of battery cells in series
            battery_capacity_ah (float): Battery capacity in amp-hours
            min_battery_voltage (float, optional): Minimum voltage per cell, defaults for supported chemistries are used if not provided
    
        Returns:
            dict: A dictionary containing charge status:
                  - 'is_charging': True if the battery is being charged, False otherwise
                  - 'configured_charge_voltage': Configured charge voltage in volts
                  - 'configured_charge_current': Configured charge current in amps
                  - 'actual_battery_voltage': Actual battery voltage in volts
                  - 'actual_battery_current': Actual battery current in amps
                  - 'charging_stage': Charging stage ('pre-charge', 'constant current', 'constant voltage', None if not charging)
                  - 'battery_percentage': Estimated battery percentage (0-100%)
                  - 'charge_time_remaining': Estimated time remaining to full charge in hours (None if not charging)

        Raises:
            ValueError: Unsupported battery chemistry
        '''
        # default minimum voltages for supported chemistries
        default_min_voltages = {
            'LiFePO4': 2.5,
            'Li-ion': 3.0,
            'NiMH': 1.0
        }
    
        # ensure battery chemistry is supported if minimum cell voltage not provided
        if min_battery_voltage is None and battery_chemistry not in default_min_voltages:
            raise ValueError('Unsupported battery chemistry')
    
        # determine minimum cell voltage and pack voltage
        min_battery_voltage = min_battery_voltage or default_min_voltages[battery_chemistry]
        total_min_voltage = min_battery_voltage * num_cells
    
        charge_voltage, charge_current = self.get_charge_parameters()
        vbat = self.vbat
        ibat = self.ibat
    
        # determine charging status
        is_charging = self.charging_enabled() and ibat > 0
    
        # determine charging stage based on battery voltage
        charging_stage = None
        if is_charging:
            if vbat < charge_voltage * 0.7:
                charging_stage = 'pre-charge'
            elif vbat < charge_voltage * 0.95:
                charging_stage = 'constant current'
            else: 
                charging_stage = 'constant voltage'
    
        # estimate battery percentage based on voltage
        # minimum 0%, maximum 100%
        battery_percentage = max(
            0, min(100, ((vbat - total_min_voltage) / (charge_voltage - total_min_voltage)) * 100)
        )
    
        # calculate charge time remaining
        if is_charging and battery_percentage < 100:
            remaining_capacity_ah = (1 - (battery_percentage / 100)) * battery_capacity_ah
            charge_time_remaining = remaining_capacity_ah / charge_current  # in hours
        else:
            charge_time_remaining = None
        
        return {
            'is_charging': is_charging,
            'configured_charge_voltage': charge_voltage,
            'configured_charge_current': charge_current,
            'actual_battery_voltage': vbat,
            'actual_battery_current': ibat,
            'charging_stage': charging_stage,
            'battery_percentage': battery_percentage,
            'charge_time_remaining': charge_time_remaining
        }

    
    ### USB Sourcing ###
    def get_usb_sourcing_parameters(self):
        '''
        Get configured USB sourcing voltage and current.
    
        Returns:
            tuple: A tuple containing USB sourcing voltage (in volts) and current (in amps)
        '''
        voltage_reg = self._read_register(REG_USB_SOURCE_VOLTAGE)
        current_reg = self._read_register(REG_USB_SOURCE_CURRENT)
    
        usb_voltage = voltage_reg * 0.016  # scale 16mV per bit to volts
        usb_current = current_reg * 0.05   # scale 50mA per bit to amps
    
        return usb_voltage, usb_current

    def enable_usb_sourcing(self, voltage_v=5.0, current_a=3.0, port=1):
        '''
        Enable USB-C sourcing with specified voltage, current, and port.

        Args:
            voltage_v (float): Sourcing voltage in volts, defaults to 5.0
            current_a (float): Sourcing current in amps, defaults to 3.0
            port (int): The port to enable sourcing on (1 or 2), defaults to 1
        Raises:
            ValueError: Invalid port number, must be 1 or 2
        '''
        if port not in [1, 2]:
            raise ValueError('Invalid port number, must be 1 or 2')

        reg_value = self._read_register(REG_USB_SOURCE_CTRL)

        # if sourcing is already enabled on a different port, disable it first
        current_port = (reg_value >> 1) & 0x01
        if current_port != (port - 1):
            reg_value &= ~(1 << 1)  # clear SOURCE_PORT bit
            reg_value &= ~(1 << 0)  # disable sourcing
            self._write_register(REG_USB_SOURCE_CTRL, reg_value)
            print(f'USB sourcing disabled on port {current_port + 1}.')

        # set sourcing voltage and current
        voltage_mv = int(voltage_v * 1000)
        current_ma = int(current_a * 1000)
        voltage_reg = int(voltage_mv / 16)  # voltage LSB is 16mV per bit
        current_reg = int(current_ma / 50)  # current LSB is 50mA per bit
        self._write_register(REG_USB_SOURCE_VOLTAGE, voltage_reg)
        self._write_register(REG_USB_SOURCE_CURRENT, current_reg)

        # enable sourcing on the selected port
        reg_value |= (port - 1) << 1  # set SOURCE_PORT bit
        reg_value |= (1 << 0)  # enable sourcing
        self._write_register(REG_USB_SOURCE_CTRL, reg_value)
        logging.info(f'USB sourcing enabled on port {port}: {voltage_v}V, {current_a}A')

    def disable_usb_sourcing(self):
        '''Disable USB-C sourcing.'''
        reg_value = self._read_register(REG_USB_SOURCE_CTRL)
        reg_value &= ~(1 << 0)  # clear SOURCE_EN bit
        self._write_register(REG_USB_SOURCE_CTRL, reg_value)
        logging.info('USB-C sourcing disabled')

    def usb_sourcing_enabled(self):
        '''
        Check if USB sourcing is enabled.
    
        Returns:
            bool: True if USB sourcing is enabled, False otherwise
        '''
        reg_value = self._read_register(REG_USB_SOURCE_CTRL, reg_bits=8)
        return bool(reg_value & (1 << 0))  # check if the USB_SOURCE_EN bit (bit 0) is set

    
    ### Ship and Shutdown Modes ###
    def enter_ship_mode(self, delay_enabled=False):
        '''
        Enter ship mode to minimize quiescent current.
    
        Args:
            delay_enabled (bool): Whether to delay 10 seconds before entering ship mode, defaults to False
        '''
        reg_value = self._read_register(REG_OTG_CONFIG)
        
        if delay_enabled:
            reg_value |= (1 << 6)  # set SHIP_DLY_EN bit to enable 10-second delay
        else:
            reg_value &= ~(1 << 6)  # clear SHIP_DLY_EN bit to disable 10-second delay
    
        reg_value |= (1 << 5)  # set SHIP_MODE bit to enter ship mode
        self._write_register(REG_OTG_CONFIG, reg_value)
        
        logging.info(f'Entering ship mode{' after 10-second delay' if delay_enabled else ''}')

    def exit_ship_mode(self):
        '''Exit ship mode to enable normal charger operations.'''
        reg_value = self._read_register(REG_OTG_CONFIG)
        reg_value &= ~(1 << 5)  # clear SHIP_MODE bit
        self._write_register(REG_OTG_CONFIG, reg_value)
        logging.info('Exiting ship mode')

    def ship_mode_enabled(self):
        '''
        Check if ship mode is enabled.
    
        Returns:
            bool: True if ship mode is enabled, False otherwise
        '''
        reg_value = self._read_register(REG_OTG_CONFIG, reg_bits=8)
        return bool(reg_value & (1 << 5))  # check if the SHIP_MODE_EN bit (bit 5) is set

    def enter_shutdown_mode(self):
        '''Enter shutdown mode.'''
        reg_value = self._read_register(REG_PROTECTION_CTRL)
        reg_value |= (1 << 7)  # enable shutdown mode
        self._write_register(REG_PROTECTION_CTRL, reg_value)
        logging.info('Entering shutdown mode')

    
    ### Fault Status ###
    #TODO detail fault status dict in doc string
    #TODO should these be monitored in a loop with a fault callback?
    def get_fault_status(self):
        '''
        Retrieve fault status.
    
        Returns:
            dict: Dictionary of all fault conditions and their states (True/False).
        '''
        fault_reg = self._read_register(REG_FAULT_STATUS, reg_bits=8)
    
        faults = {
            'INPUT_OVP': bool(fault_reg & (1 << 7)),   # input over-voltage protection fault
            'INPUT_UVP': bool(fault_reg & (1 << 6)),   # input under-voltage protection fault
            'INPUT_OCP': bool(fault_reg & (1 << 5)),   # input over-current protection fault
            'BAT_OVP': bool(fault_reg & (1 << 4)),     # battery over-voltage protection fault
            'BAT_OCP': bool(fault_reg & (1 << 3)),     # battery over-current protection fault
            'TS_FAULT': bool(fault_reg & (1 << 2)),    # temperature sensor fault
            'TDIE_FAULT': bool(fault_reg & (1 << 1)),  # die over-temperature protection fault
            'CHRG_FAULT': bool(fault_reg & (1 << 0)),  # charging fault
        }
    
        logging.info(f'Fault status: {faults}')
        return faults

    
    ### Safety Timers ###
    def get_safety_timers(self):
        '''
        Get precharge and fast charge safety timer values.
    
        Returns:
            tuple: A tuple containing precharge timeout (minutes) and fast charge timeout (minutes)
        '''
        reg_value = self._read_register(REG_TIMER_CONTROL, reg_bits=8)
        precharge_timeout = (reg_value & 0x0F) * 10  # lower 4 bits, scaled by 10 minutes
        fast_charge_timeout = ((reg_value >> 4) & 0x0F) * 40  # upper 4 bits, scaled by 40 minutes
        return precharge_timeout, fast_charge_timeout
        
    def set_safety_timers(self, precharge_timeout=30, fast_charge_timeout=180):
        '''
        Set precharge and fast charge safety timers.

        Args:
            precharge_timeout (int): Precharge timeout in minutes
            fast_charge_timeout (int): Fast charge timeout in minutes
        '''
        precharge_val = min(max(precharge_timeout, 0), 60) // 10
        fastcharge_val = min(max(fast_charge_timeout, 0), 320) // 40
        timer_reg = (fastcharge_val << 4) | precharge_val
        self._write_register(REG_TIMER_CONTROL, timer_reg, reg_bits=8)
        logging.info(f'Safety timers set: precharge = {precharge_timeout} min, fast charge = {fast_charge_timeout} min')
        

    ### Adapter and Input Management ###
    def port_status(self, port):
        '''
        Get status of the specified port.
    
        Args:
            port (int): Port number (1 or 2)
    
        Returns:
            dict: Dictionary containing the port status:
                  - 'is_sourcing': True if the port is sourcing power, False otherwise
                  - 'is_sinking': True if the port is sinking power, False otherwise
                  - 'adapter_connected': True if an adapter is connected, False otherwise
                  - 'input_current_limit': Input current limit in amps (if applicable, None otherwise)
                  - 'input_voltage': Input voltage in volts (if applicable, None otherwise)
                  - 'mppt_enabled': True if MPPT is enabled for the port, False otherwise
                  - 'sourcing_voltage': configured sourcing voltage in volts (None if sinking)
                  - 'sourcing_current': configured sourcing current in amps (None if sinking)
        '''
        if port not in [1, 2]:
            raise ValueError('invalid port number. port must be 1 or 2.')
    
        # read status from relevant registers
        input_source_ctrl = self._read_register(REG_INPUT_SOURCE_CTRL, reg_bits=8)
        mppt_ctrl = self._read_register(REG_MPPT_CTRL, reg_bits=8)
    
        # determine port sourcing/sinking status
        is_sourcing = bool(input_source_ctrl & (1 << 7))  # sourcing bit
        is_sinking = not is_sourcing  # assume port is sinking if not sourcing
    
        # determine if adapter is connected
        adapter_connected = bool(input_source_ctrl & (1 << 6))  # adapter detection bit
    
        # check input current limit and input voltage if sinking
        input_current_limit = (input_source_ctrl & 0xFF) * 0.05 if is_sinking else None  # convert 50mA/bit to amps
        input_voltage = self._read_register(REG_ADC_VBUS) * 0.016 if is_sinking else None  # convert 16mV/bit to volts
    
        # include mppt information if port is configured for mppt
        mppt_enabled = bool(mppt_ctrl & (1 << 0)) and ((mppt_ctrl >> 1) & 0x01) == (port - 1)
    
        # retrieve sourcing voltage and current if sourcing
        usb_source_voltage = self._read_register(REG_USB_SOURCE_VOLTAGE) if is_sourcing else None
        usb_source_current = self._read_register(REG_USB_SOURCE_CURRENT) if is_sourcing else None
        sourcing_voltage = usb_source_voltage * 0.016 if usb_source_voltage is not None else None  # convert 16mV/bit to volts
        sourcing_current = usb_source_current * 0.05 if usb_source_current is not None else None  # convert 50mA/bit to amps
    
        return {
            'is_sourcing': is_sourcing,
            'is_sinking': is_sinking,
            'adapter_connected': adapter_connected,
            'input_current_limit': input_current_limit,
            'input_voltage': input_voltage,
            'mppt_enabled': mppt_enabled,
            'sourcing_voltage': sourcing_voltage,
            'sourcing_current': sourcing_current
        }

    def get_input_current_limit(self):
        '''
        Get input current limit.
    
        Returns:
            float: Input current limit in amps
        '''
        reg_value = self._read_register(REG_INPUT_SOURCE_CTRL, reg_bits=8)
        return (reg_value & 0xFF) * 0.05  # scale 50mA per bit to amps

    def set_input_current_limit(self, current_a):
        '''Set input current limit.

        Args:
            current_a (float): Input current limit in amps
        '''
        current_ma = int(current_a * 1000)
        input_limit_reg = current_ma // 50  # LSB = 50mA
        self._write_register(REG_INPUT_SOURCE_CTRL, input_limit_reg, reg_bits=8)
        logging.info(f'Input current limit set to {current_a}A')

    def get_minimum_system_voltage(self):
        '''
        Get minimum system voltage.
    
        Returns:
            float: Minimum system voltage in volts
        '''
        reg_value = self._read_register(REG_MIN_SYS_VOLTAGE, reg_bits=16)
        return reg_value * 0.016  # scale 16mV per bit to volts
    
    def set_minimum_system_voltage(self, voltage_v):
        '''Set minimum system voltage.

        Args:
            voltage_v (float): Minimum system voltage in volts
        '''
        voltage_mv = int(voltage_v * 1000)
        min_sys_reg = voltage_mv // 16  # LSB = 16mV
        self._write_register(REG_MIN_SYS_VOLTAGE, min_sys_reg)
        logging.info(f'Minimum system voltage set to {voltage_v}V')
    
