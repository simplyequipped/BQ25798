# Datasheet: https://www.ti.com/lit/ds/symlink/bq25798.pdf

import time
import atexit
import threading
import logging
from logging.handlers import RotatingFileHandler

import smbus2


# configure logging
logger = logging.getLogger('bq25798_log')
logger.setLevel(logging.DEBUG)
# max size: 1MB, keep 3 backups
handler = RotatingFileHandler('bq25798.log', maxBytes=1_000_000, backupCount=3)
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)


class BQ25798:
    # configuration and control registers
    REG_MIN_SYS_VOLTAGE = 0x00     # minimum system voltage control register
    REG_CHARGE_VOLTAGE = 0x01      # charge voltage control register
    REG_CHARGE_CURRENT = 0x03      # charge current control register
    REG_PRECHARGE_CURRENT = 0x06   # pre-charge and termination current control register
    REG_ADC_CTRL = 0x2E            # ADC control register
    REG_USB_SOURCE_VOLTAGE = 0x16  # USB-C sourcing voltage register
    REG_USB_SOURCE_CURRENT = 0x18  # USB-C sourcing current register
    REG_USB_SOURCE_CTRL = 0x19     # USB-C source control register
    REG_OTG_CONFIG = 0x09          # OTG and ship mode control register
    REG_MPPT_CTRL = 0x1A           # mppt control register
    REG_INPUT_SOURCE_CTRL = 0x1B   # input source control register
    REG_TIMER_CONTROL = 0x12       # safety timer and watchdog control register
    REG_FAULT_STATUS = 0x20        # fault status register
    REG_PROTECTION_CTRL = 0x1C     # protection control register
    # ADC reading registers
    REG_ADC_IBUS = 0x31            # input current ADC result register
    REG_ADC_IBAT = 0x33            # battery charge/discharge current ADC result register
    REG_ADC_VBUS = 0x35            # input voltage ADC result register
    REG_ADC_VAC1 = 0x37            # port 1 voltage ADC result register
    REG_ADC_VAC2 = 0x38            # port 2 voltage ADC result register
    REG_ADC_VBAT = 0x3B            # battery voltage ADC result register
    REG_ADC_VSYS = 0x3D            # system voltage ADC result register
    REG_ADC_TS = 0x3F              # external temperature sensor ADC result register
    REG_ADC_TDIE = 0x41            # die temperature ADC result register
    
    # battery chemistry
    BAT_CHEMISTRY_LIFEPO4 = 'LiFePo4'
    BAT_CHEMISTRY_LI_ION  = 'Li-ion'
    BAT_CHEMISTRY_NIMH    = 'NiMH'
    BAT_CHEMISTRIES = [BAT_CHEMISTRY_LIFEPO4, BAT_CHEMISTRY_LI_ION, BAT_CHEMISTRY_NIMH]
    
    # state-of-charge (SoC) curve reference points
    BAT_SOC_CURVE = {
        BAT_CHEMISTRY_LIFEPO4 : [
            (2.5, 0),
            (2.8, 10),
            (3.0, 20),
            (3.2, 30),
            (3.25, 40),
            (3.3, 50),
            (3.35, 60),
            (3.4, 70),
            (3.45, 80),
            (3.5, 90),
            (3.6, 100)
        ],
        BAT_CHEMISTRY_LI_ION : [
            (3.0, 0),
            (3.2, 10),
            (3.4, 20),
            (3.6, 30),
            (3.7, 40),
            (3.8, 50),
            (3.9, 60),
            (4.0, 70),
            (4.1, 80),
            (4.15, 90),
            (4.2, 100)
        ],
        BAT_CHEMISTRY_NIMH : [
            (1.0, 0),
            (1.1, 10),
            (1.15, 20),
            (1.2, 30),
            (1.25, 40),
            (1.3, 50),
            (1.35, 60),
            (1.4, 70),
            (1.45, 80),
            (1.5, 90),
            (1.55, 100)
        ] 
    }
    '''State-of-charge (SoC) curve reference points for supported battery chemistries, used for estimating battery percentage.'''
    
    # battery states
    BAT_STATE_DISCHARGING = 'discharging'
    BAT_STATE_CHARGING    = 'charging'
    BAT_STATE_CHARGED     = 'charged'
    BAT_STATES = [BAT_STATE_DISCHARGING, BAT_STATE_CHARGING, BAT_STATE_CHARGED]
    
    # ADC modes
    ADC_MODE_ONE_SHOT     = 'one-shot'
    ADC_MODE_CONTINUOUS   = 'continuous'
    ADC_MODES = [ADC_MODE_ONE_SHOT, ADC_MODE_CONTINUOUS]
    
    FAULT_INPUT_OVER_VOLTAGE  = 'input over-voltage fault'
    FAULT_INPUT_UNDER_VOLTAGE = 'input under-voltage fault'
    FAULT_INPUT_OVER_CURRENT  = 'input over-current fault'
    FAULT_BAT_OVER_VOLTAGE    = 'battery over-voltage fault'
    FAULT_BAT_OVER_CURRENT    = 'battery over-current fault'
    FAULT_BAT_TEMP            = 'battery temperature fault'
    FAULT_DIE_TEMP            = 'die temperature fault'
    FAULT_CHARGING            = 'charging fault'
    FAULTS = [FAULT_INPUT_OVER_VOLTAGE, FAULT_INPUT_UNDER_VOLTAGE, FAULT_INPUT_OVER_CURRENT, FAULT_BAT_OVER_VOLTAGE, FAULT_BAT_OVER_CURRENT, FAULT_BAT_TEMP, FAULT_DIE_TEMP, FAULT_CHARGING]

    def __init__(self, charge_voltage, charge_current, bat_series_cells, bat_chemistry, bat_capacity_ah, i2c_bus=1, i2c_address=0x6B):
        '''Initialize BQ25798 instance.
        
        Args:
            i2c_bus (int): I2C bus number, defaults to 1
            i2c_address (int): I2C address of the BQ25798, defaults 0x6B
            charge_voltage (float): Battery charge voltage in volts
            charge_current (float): Battery charge current in amps
            bat_series_cells (int): Number of series battery cells (typically 1-4)
            bat_chemistry (str): Battery chemistry (LiFePo4, Li-ion, NiMH)
            bat_capacity_ah (float): Battery capacity in amp hours
        
        Returns:
            bq25798.BQ25798 (obj): Initialized class instance
        
        Raises:
            ValueError: Unsupported battery chemistry
            ValueError: Invalid charge voltage
            ValueError: Invalid charge current
        '''
        self.bus = smbus2.SMBus(i2c_bus)
        '''I2C bus instance object'''
        self.address = i2c_address
        '''I2C address'''
        self.charge_voltage = None
        '''Battery charge voltage'''
        self.charge_current = None
        '''Battery charge current'''
        self.async_update_seconds = 3
        '''How often to update local variables assocaited with ADC readings, state, and fault status'''
        self.bat_series_cells = bat_series_cells
        '''Number of battery series cells (ex 13.8V 4S battery = 4), used for calculating battery percentage'''
        self.bat_capacity_ah = bat_capacity_ah
        '''Battery capacity in amp hours, used for calculating battery percentage'''
        self.bat_chemistry = bat_chemistry
        '''Battery chemistry (see BQ25798.BAT_CHEMISTRY_* constants), used for calculating battery percentage'''
        
        # close I2C bus at exit
        atexit.register(self.close)
        
        # validate battery chemistry
        if self.bat_chemistry not in self.BAT_CHEMISTRIES:
            raise ValueError('Unsupported battery chemistry')
        
        # validate battery charging parameters
        if not (0 < charge_voltage <= 18.8):
            raise ValueError('Invalid charge voltage')
        if not (0 < charge_current <= 5):
            raise ValueError('Invalid charge current')
        
        self.adc_mode = self.ADC_MODE_ONE_SHOT
        '''ADC operating mode (see BQ25798.ADC_MODE_* constants)'''
        self.state = self.BAT_STATE_DISCHARGING
        '''Battery state (see BQ25798.BAT_STATE_* constants)'''
        
        self._state_lock = threading.Lock()
        self._adc_lock = threading.Lock()
        self._fault_lock = threading.Lock()
        
        self.state_change_callback = None
        '''Battery state change callback function, defaults to None
        - Callback signature: *func(state)* where *state* is a BQ25798.BAT_STATE_* constant'''
        self.fault_change_callback = None
        '''BQ25798 fault status change callback function, defaults to None
        - Callback signature: *func(fault, status)* where *fault* is a BQ25798.BAT_STATE_* constant'''
        
        # initialize ADC reading variables
        self.vbus = 0.0
        '''Input voltage ADC reading in volts'''
        self.ibus = 0.0
        '''Input current ADC reading in amps'''
        self.vbat = 0.0
        '''Battery voltage ADC reading in volts'''
        self.ibat = 0.0
        '''Battery current ADC reading in amps'''
        self.vac1 = 0.0
        '''Port 1 voltage ADC reading in volts'''
        self.vac2 = 0.0
        '''Port 2 voltage ADC reading in volts'''
        self.vsys = 0.0
        '''System voltage ADC reading in volts'''
        self.tbat = 0.0
        '''Battery (aka external) temperature ADC reading in °C'''
        self.tdie = 0.0
        '''BQ25798 die temperature ADC reading in °C'''
        
        # initalize fault status variables
        self.input_over_voltage_fault = False
        self.input_under_voltage_fault = False
        self.input_over_current_fault = False
        self.bat_over_voltage_fault = False
        self.bat_over_current_fault = False
        self.bat_temp_fault = False
        self.die_temp_fault = False
        self.charging_fault = False
        
        # ensure ship mode is disabled
        if self.ship_mode_enabled():
            self.disable_ship_mode()
        
        logging.info(f'BQ25798 initalized on I2C address {self.address}')
        
        # initialize configuration
        self.set_charge_voltage(charge_voltage)
        self.set_charge_current(charge_current)
        self.set_precharge_timeout(30) # 30 min
        self.set_fast_charge_timeout(180) # 180 min
        self.set_usb_sourcing_voltage(5.0) # 5V
        self.set_usb_sourcing_current(3.0) # 3A
        self.set_adc_one_shot_mode()
        self.disable_mppt()
        
        # initialize adc readings
        self.update_adc_readings()
        # initialize fault status
        self.update_fault_status()
        
        #TODO
        # start job loop thread
        #job_thread = threading.Thread(target=self._job_loop, daemon=True)
        #job_thread.start()
    
    def _job_loop(self):
        '''Jop loop to execute reoccuring tasks.
        
        Updates ADC readings, battery state, and fault status. Exists when I2C bus is closed (ex. self.close() is called).
        '''
        # SMBus.fd is set to None after SMBus.close() is called
        while self.bus.fd is not None:
            time.sleep(self.async_update_seconds)
            
            # update ADC readings
            with self._adc_lock:
                self.update_adc_readings()
            
            # update state
            previous_state = self.state
            with self._state_lock:
                if self.battery_charging():
                    if self.battery_percentage() < 100:
                        self.state = self.BAT_STATE_CHARGING
                    else:
                        self.state = self.BAT_STATE_CHARGED
                else:
                    self.state = self.BAT_STATE_DISCHARGING
            
            if self.state != previous_state:
                logging.info(f'State change: {self.state}')
                # state change callback
                if self.state_change_callback is not None:
                    self.state_change_callback(self.state)
            
            # update fault status
            previous_faults = {
                self.FAULT_INPUT_OVER_VOLTAGE  : self.input_over_voltage_fault,
                self.FAULT_INPUT_UNDER_VOLTAGE : self.input_under_voltage_fault,
                self.FAULT_INPUT_OVER_CURRENT  : self.input_over_current_fault,
                self.FAULT_BAT_OVER_VOLTAGE    : self.bat_over_voltage_fault,
                self.FAULT_BAT_OVER_CURRENT    : self.bat_over_current_fault,
                self.FAULT_BAT_TEMP            : self.bat_temp_fault,
                self.FAULT_DIE_TEMP            : self.die_temp_fault,
                self.FAULT_CHARGING            : self.charging_fault,
            }
            
            with self._fault_lock:
                self.update_fault_status()
            
            current_faults = {
                self.FAULT_INPUT_OVER_VOLTAGE  : self.input_over_voltage_fault,
                self.FAULT_INPUT_UNDER_VOLTAGE : self.input_under_voltage_fault,
                self.FAULT_INPUT_OVER_CURRENT  : self.input_over_current_fault,
                self.FAULT_BAT_OVER_VOLTAGE    : self.bat_over_voltage_fault,
                self.FAULT_BAT_OVER_CURRENT    : self.bat_over_current_fault,
                self.FAULT_BAT_TEMP            : self.bat_temp_fault,
                self.FAULT_DIE_TEMP            : self.die_temp_fault,
                self.FAULT_CHARGING            : self.charging_fault,
            }
            
            for fault, previous_status in previous_faults.items():
                current_status = current_faults[fault]
                if previous_status != current_status:
                    logging.info(f'Fault status change: {fault} {"active" if current_status else "inactive"}')
                    # fault change callback
                    if self.fault_change_callback is not None:
                        self.fault_change_callback(fault, current_status)
        
    def _read_register(self, register, length=8, start=None, end=None):
        '''Read register value.

        Args:
            register (int): Register address to read (example: 0x20)
            length (int): Register length, defaults to 8
            start (int): First bit for desired value, defaults to 0
            end (int): Last bit for desired value, defaults to *length*-1
        
        Returns:
            bytes: Register value
        
        Raises:
            ValueError: Register length must be 8 or 16
        '''
        if length not in (8, 16):
            raise ValueError('Register length must be 8 or 16')

        if start is None:
            start = 0

        if end is None:
            end = length - 1

        if length == 8:
            value = self.bus.read_byte_data(self.address, register) # read 8 bits
        elif length == 16:
            value = self.bus.read_word_data(self.address, register) # read 16 bits

        if start > 0:
            value = (value >> start)

        if end < (length - 1):
            value_length = end - start + 1
            value = value & self._bit_mask(value_length)

        return value

        #if reg_bits == 16:
        #    data = self.bus.read_word_data(self.address, reg_address)
        #    msb = (data >> 8) & 0xFF
        #    lsb = data & 0xFF
        #    value = (msb << 8) | lsb
        #    return value
        #elif reg_bits == 8:
        #    return self.bus.read_byte_data(self.address, reg_address)
        #else:
        #    raise ValueError('Invalid reg_bits value, only 8 and 16 are supported')
        
    def _write_register(self, register, value, length=8, value_length=None, start=0):
        '''Write register value

        Use *value_length* to indicate a value less than the full register length. User *start* to indicate the starting bit for the value in the register. A typical use case is setting bits in a configuration register.
        
        Args:
            register (int): Register address to write (example: 0x20)
            length (int): Register length, defaults to 8
            value_length (int, None): Number of bits to write, defaults to *length*
            start (in): First bit to write value, defaults to 0
        
        Raises:
            ValueError: Register length must be 8 or 16
        '''
        #if length == 8:
        #    reg_value = self._read_register(register, length=8)
        #elif length == 16:
        #    reg_value = self._read_register(register, length=16)

        if length not in (8, 16):
            raise ValueError('Register length must be 8 or 16')

        if value_length is None:
            value_length = length

        if value_length < length:
           register_value = self._read_register(register, length=length)
           value = self._set_bit_value(register_value, value, value_length, start)

        if length == 8:
            reg_value = self.bus.write_byte_data(self.address, register, value)
        elif length == 16:
            reg_value = self.bus.write_word_data(self.address, register, value)

        #if reg_bits == 16:
        #    msb = (value >> 8) & 0xFF
        #    lsb = value & 0xFF
        #    data = (msb << 8) | lsb
        #    self.bus.write_word_data(self.address, reg_address, data)
        #elif reg_bits == 8:
        #    self.bus.write_byte_data(self.address, reg_address, value)
        #else:
        #    raise ValueError('Invalid reg_bits value, only 8 and 16 are supported')

    def _bit_mask(self, length):
        return (1 << length) - 1

    def _set_bit_value(self, register_value, new_value, new_value_length, start=0):
        register_length = 16 if register_value > 0xFF else 8

        if start + new_value_length > register_length:
            raise ValueError('Value too large for register at specified position')

        mask = self._bit_mask(new_value_length) << start
        new_reg_value = (register_value & ~mask) | ((new_value << start) & mask) 
        
    def close(self):
        '''Close the I2C bus connection.'''
        if self.bus:
            self.bus.close()
            print('I2C bus closed')
        
    
    ### ADC Functions ###
    def enable_adc(self):
        '''Set ADC enable bit.'''
        reg_value = self._read_register(self.REG_ADC_CTRL)
        reg_value |= (1 << 0) # set enable bit
        self._write_register(self.REG_ADC_CTRL, reg_value)
        logging.info('ADC enabled')
        
    def disable_adc(self):
        '''Clear ADC enable bit.'''
        reg_value = self._read_register(self.REG_ADC_CTRL)
        reg_value &= ~(1 << 0) # clear enable bit
        self._write_register(self.REG_ADC_CTRL, reg_value)
        logging.info('ADC disabled')
        
    def adc_enabled(self):
        '''Check if ADC is enabled.
        
        Returns:
            bool: True if ADC is enabled, False otherwise
        '''
        reg_value = self._read_register(self.REG_ADC_CTRL, reg_bits=8)
        return bool(reg_value & (1 << 0)) # enable bit to boolean
        
    def set_adc_one_shot_mode(self):
        '''Set ADC to one-shot mode.'''
        reg_value = self._read_register(self.REG_ADC_CTRL)
        reg_value |= (1 << 1) # Set ADC_ONE_SHOT bit
        self._write_register(self.REG_ADC_CTRL, reg_value)
        self.adc_mode = self.ADC_MODE_ONE_SHOT
        logging.info('ADC in one-shot mode')
        
    def set_adc_continuous_mode(self):
        '''Set the ADC to continuous mode.'''
        reg_value = self._read_register(self.REG_ADC_CTRL, reg_bits=8)
        reg_value &= ~(1 << 1) # clear the ADC_ONE_SHOT bit
        reg_value |= (1 << 0) # set the ADC_EN bit
        self._write_register(self.REG_ADC_CTRL, reg_value, reg_bits=8)
        self.adc_mode = self.ADC_MODE_CONTINUOUS
        logging.info('ADC in continuous mode')
        
    def get_adc_mode(self):
        '''Get ADC operating mode.
        
        Returns:
            str: *ADC_MODE_ONE_SHOT* or *ADC_MODE_CONTINUOUS*
        '''
        reg_value = self._read_register(self.REG_ADC_CTRL, reg_bits=8)
        if reg_value & (1 << 1): # check if ADC_ONE_SHOT bit is set
            return self.ADC_MODE_ONE_SHOT
        else:
            return self.ADC_MODE_CONTINUOUS
        
    def update_adc_readings(self):
        '''Update local ADC reading values.
        
        If ADC is in one-shot mode the ADC is enabled before updating readings, and disabled after updates are complete.
        '''
        if self.adc_mode == self.ADC_MODE_ONE_SHOT:
            self.enable_adc()
            time.sleep(0.1) # allow time for ADC to complete conversions
        
        self.ibus = self._read_register(self.REG_ADC_IBUS) / 1000 # convert to A, 1mA per bit scaling factor
        self.ibat = self._read_register(self.REG_ADC_IBAT) / 1000 # convert to A, 1mA per bit scaling factor
        self.vbus = self._read_register(self.REG_ADC_VBUS) / 1000 # convert to V, 1mV per bit scaling factor
        self.vac1 = self._read_register(self.REG_ADC_VAC1) / 1000 # convert to V, 1mV per bit scaling factor
        self.vac2 = self._read_register(self.REG_ADC_VAC2) / 1000 # convert to V, 1mV per bit scaling factor
        self.vbat = self._read_register(self.REG_ADC_VBAT) / 1000 # convert to V, 1mV per bit scaling factor
        self.vsys = self._read_register(self.REG_ADC_VSYS) / 1000 # convert to V, 1mV per bit scaling factor
        self.tbat = self._read_register(self.REG_ADC_TS) * 0.0976563 # 0.0976563% per bit scaling factor
        self.tdie = self._read_register(self.REG_ADC_TDIE) * 0.5 # convert to °C, 0.5°C per bit scaling factor
        
        if self.adc_mode == self.ADC_MODE_ONE_SHOT:
            self.disable_adc()
        
    
    ### Solar MPPT Input ###
    def enable_mppt(self, port=2):
        '''Enable MPPT (Maximum Power Point Tracking) for a specific port.
        
        Args:
            port (int): Input port to enable MPPT for (1 or 2), defaults to 2
        
        Raises:
            ValueError: Invalid port number, must be 1 or 2
        '''
        if port not in (1, 2):
            raise ValueError('Invalid port number, must be 1 or 2')
        
        reg_value = self._read_register(self.REG_MPPT_CTRL)
        
        # if MPPT is already enabled on a different port, disable it first
        current_port = (reg_value >> 1) & 0x01
        if current_port != (port - 1):
            reg_value &= ~(1 << 1) # clear MPPT_PORT bit
            reg_value &= ~(1 << 0) # disable MPPT
            self._write_register(self.REG_MPPT_CTRL, reg_value)
        
        # enable MPPT for specified port
        reg_value |= (port - 1) << 1 # set MPPT_PORT bit
        reg_value |= (1 << 0) # enable MPPT
        self._write_register(self.REG_MPPT_CTRL, reg_value)
        
        logging.info(f'MPPT enabled on port {port}.')
        
    def disable_mppt(self):
        '''Disable MPPT (Maximum Power Point Tracking).'''
        reg_value = self._read_register(self.REG_MPPT_CTRL)
        reg_value &= ~(1 << 0) # clear MPPT_EN bit
        self._write_register(self.REG_MPPT_CTRL, reg_value)
        logging.info('MPPT disabled')
        
    def mppt_enabled(self):
        '''Check if MPPT (Maximum Power Point Tracking) is enabled.
        
        Returns:
            bool: True if MPPT is enabled, False otherwise
        '''
        reg_value = self._read_register(self.REG_MPPT_CTRL, reg_bits=8)
        return bool(reg_value & (1 << 0)) # check if the enable bit is set
        
    def get_mppt_port(self):
        '''Get the input port configured for MPPT (Maximum Power Point Tracking).
        
        Returns:
            int or None: Input port number (1 or 2) if configured, otherwise None
        '''
        reg_value = self._read_register(self.REG_MPPT_CTRL, reg_bits=8)
        mppt_port_bit = (reg_value >> 1) & 0x01
        mppt_port = mppt_port_bit + 1 if mppt_port_bit in (0,1) else None
        return mppt_port
        
    
    ### Battery Charging ###
    def get_charge_voltage(self):
        '''Get configured battery charging voltage.
        
        Returns:
            float: Battery charge voltage in volts
        '''
        voltage_reg = self._read_register(self.REG_CHARGE_VOLTAGE)
        charge_voltage = voltage_reg * 16 / 1000 # scale 16mV per bit to volts
        
        return charge_voltage
        
    def set_charge_voltage(self, voltage_v):
        '''Set battery charging voltage.
        
        Args:
            voltage_v (float): Battery charge voltage in volts
        '''
        self.charge_voltage = voltage_v
        
        voltage_mv = int(voltage_v * 1000) # scale to mV
        voltage_reg = voltage_mv // 16 # scale to 16mV per bit
        self._write_register(self.REG_CHARGE_VOLTAGE, voltage_reg)
        logging.info(f'Battery charge voltage set to {voltage_v}V')
        
    def get_charge_current(self):
        '''Get configured battery charging current.
        
        Returns:
            float: Battery charge current in amps
        '''
        current_reg = self._read_register(self.REG_CHARGE_CURRENT)
        charge_current = current_reg * 64 / 1000 # scale 64mA per bit to amps
        return charge_current
        
    def set_charge_current(self, current_a):
        '''Set battery charging current.
        
        Args:
            current_a (float): Battery charge current in amps
        '''
        self.charge_current = current_a
        
        current_ma = int(current_a * 1000) # scale to mA
        current_reg = current_ma // 64 # scale to 64mA per bit
        self._write_register(self.REG_CHARGE_CURRENT, current_reg)
        logging.info(f'Battery charge current set to {current_a}A')
        
    def enable_charging(self):
        '''Enable battery charging.'''
        reg_value = self._read_register(self.REG_PROTECTION_CTRL)
        reg_value |= (1 << 0) # set enable bit
        self._write_register(self.REG_PROTECTION_CTRL, reg_value)
        logging.info('Battery charging enabled')
        
    def disable_charging(self):
        '''Disable battery charging.'''
        reg_value = self._read_register(self.REG_PROTECTION_CTRL)
        reg_value &= ~(1 << 0) # clear enable bit
        self._write_register(self.REG_PROTECTION_CTRL, reg_value)
        logging.info('Battery charging disabled')
        
    def charging_enabled(self):
        '''Whether battery charging is enabled.
        
        Returns:
            bool: True if charging is enabled, False otherwise
        '''
        reg_value = self._read_register(self.REG_PROTECTION_CTRL, reg_bits=8)
        return bool(reg_value & (1 << 0)) # check if the enable bit is set
        
    def battery_charging(self): 
        return bool(self.charging_enabled() and self.ibat > 0)
        
    def battery_percentage(self):
        '''Estimate battery percentage based on voltage.
        
        Based on:
        - Battery chemistry state-of-charge curve points
        - Number of series battery cells
        - Battery voltage ADC reading
        
        Battery voltage ADC reading is scaled to per-cell voltage, and is then linearly interpolated between state-of-charge curve points. Percentage is clamped to 0-100.
        
        Returns:
            float: Estimated battery percentage (0-100)
        '''
        cell_voltage = self.vbat / self.num_cells # per-cell voltage
        
        # soc curve points based on battery chemistry
        if self.bat_chemistry == self.BAT_CHEMISTRY_LIFEPO4:
            soc_points = self.BAT_SOC_CURVE[self.BAT_CHEMISTRY_LIFEPO4]
        elif self.bat_chemistry == self.BAT_CHEMISTRY_LI_ION:
            soc_points = self.BAT_SOC_CURVE[self.BAT_CHEMISTRY_LI_ION]
        elif self.bat_chemistry == self.BAT_CHEMISTRY_NIMH:
            soc_points = self.BAT_SOC_CURVE[self.BAT_CHEMISTRY_NIMH]
        else:
            raise ValueError('Unsupported battery chemistry')
        
        # find two soc points for interpolation
        for i in range(len(soc_points) - 1):
            v1, p1 = soc_points[i]
            v2, p2 = soc_points[i + 1]
            if v1 <= cell_voltage <= v2:
                # linear interpolation between v1 and v2
                percentage = p1 + (cell_voltage - v1) * (p2 - p1) / (v2 - v1)
                return max(0, min(100, percentage)) # clamp to [0, 100]
        
        # if voltage out of range, clamp to 0% or 100%
        if cell_voltage < soc_points[0][0]:
            return 0
        if cell_voltage > soc_points[-1][0]:
            return 100
        
    def battery_charge_stage(self):
        ''' Get battery charge stage.
        
        Charge stages:
        - 'pre-charge'
        - 'constant current'
        - 'constant voltage'
        - *None* (if not charging)
        
        Returns:
            str or None: Battery charge stage, or None if not charging
        '''
        # determine charging stage based on battery voltage
        stage = None
        
        if self.battery_charging():
            if self.vbat < self.charge_voltage * 0.7:
                stage = 'pre-charge'
            elif self.vbat < self.charge_voltage * 0.95:
                stage = 'constant current'
            else: 
                stage = 'constant voltage'
        
        return stage

    def battery_charge_time_remaining(self):
        '''Estimate battery charge time remaining.
        
        Based on:
        - Estimated battery percentage
        - Battery capacity
        - Configured battery charge current
        
        Returns:
            int or None: Estimated charge time remaining in hours, or None if not charging
        '''
        percent = self.battery_percentage()
        
        # calculate charge time remaining
        if self.battery_charging():
            if percent < 100:
                remaining_capacity = (1 - (percent / 100)) * self.bat_capacity_ah
                return remaining_capacity / self.charge_current # hours
            else:
                return 0
        else:
            return None
        
    def get_precharge_timeout(self):
        '''Get configured battery precharge safety timeout.
        
        Returns:
            int: Precharge timeout in minutes
        '''
        reg_value = self._read_register(self.REG_TIMER_CONTROL, reg_bits=8)
        return (reg_value & 0x0F) * 10 # lower 4 bits scaled from 10 minute increments
        
    def set_precharge_timeout(self, precharge_timeout=30):
        '''Set battery precharge safety timeout.
        
        Args:
            precharge_timeout (int): Precharge timeout in minutes
        '''
        precharge_val = min(max(precharge_timeout, 0), 60) // 10
        reg_value = self._read_register(self.REG_TIMER_CONTROL, reg_bits=8)
        reg_value = (reg_value & 0xF0) | precharge_val # update lower 4 bits
        self._write_register(self.REG_TIMER_CONTROL, reg_value, reg_bits=8)
        logging.info(f'Precharge timeout set to {precharge_timeout} minutes')
        
    def get_fast_charge_timeout(self):
        '''Get configured battery fast charge safety timeout.
        
        Returns:
            int: Fast charge timeout in minutes
        '''
        reg_value = self._read_register(self.REG_TIMER_CONTROL, reg_bits=8)
        return ((reg_value >> 4) & 0x0F) * 40 # upper 4 bits scaled from 40 minute increments
        
    def set_fast_charge_timeout(self, fast_charge_timeout=180):
        '''Set battery fast charge safety timeout.
        
        Args:
            fast_charge_timeout (int): Fast charge timeout in minutes
        '''
        fastcharge_val = min(max(fast_charge_timeout, 0), 320) // 40
        reg_value = self._read_register(self.REG_TIMER_CONTROL, reg_bits=8)
        reg_value = (reg_value & 0x0F) | (fastcharge_val << 4) # update upper 4 bits
        self._write_register(self.REG_TIMER_CONTROL, reg_value, reg_bits=8)
        logging.info(f'Fast charge timeout set to {fast_charge_timeout} minutes')
        
    
    ### USB Sourcing ###
    def get_usb_sourcing_voltage(self):
        '''Get configured USB-C sourcing voltage.
        
        Returns:
            float: USB-C sourcing voltage in volts
        '''
        voltage_reg = self._read_register(self.REG_USB_SOURCE_VOLTAGE)
        usb_voltage = voltage_reg * 0.016 # scale 16mV per bit to volts
        return usb_voltage

    def set_usb_sourcing_voltage(self, voltage_v):
        '''Set USB-C sourcing voltage.
        
        Args:
            voltage_v (float): USB-C sourcing voltage in volts
        '''
        voltage_mv = int(voltage_v * 1000) # convert to mV
        voltage_reg = int(voltage_mv / 16) # LSB is 16mV per bit
        self._write_register(self.REG_USB_SOURCE_VOLTAGE, voltage_reg)
        logging.info(f'USB sourcing voltage set to {voltage_v}V')

    def get_usb_sourcing_current(self):
        '''Get configured USB-C sourcing current.
        
        Returns:
            float: USB-C sourcing current in amps
        '''
        current_reg = self._read_register(self.REG_USB_SOURCE_CURRENT)
        usb_current = current_reg * 0.05 # scale 50mA per bit to amps
        return usb_current
        
    def set_usb_sourcing_current(self, current_a):
        '''Set USB-C sourcing current.
        
        Args:
            current_a (float): USB-C sourcing current in amps
        '''
        current_ma = int(current_a * 1000) # convert to mA
        current_reg = int(current_ma / 50) # LSB is 50mA per bit
        self._write_register(self.REG_USB_SOURCE_CURRENT, current_reg)
        logging.info(f'USB sourcing current set to {current_a}A')
        
    def enable_usb_sourcing(self, port=1):
        '''Enable USB-C sourcing on specified input port.
        
        Args:
            port (int): Input port to enable sourcing on (1 or 2), defaults to 1
        
        Raises:
            ValueError: Invalid port number, must be 1 or 2
        '''
        if port not in [1, 2]:
            raise ValueError('Invalid port number, must be 1 or 2')
        
        reg_value = self._read_register(self.REG_USB_SOURCE_CTRL)
        
        # if sourcing is already enabled on a different port, disable it first
        current_port = (reg_value >> 1) & 0x01
        if current_port != (port - 1):
            reg_value &= ~(1 << 1) # clear SOURCE_PORT bit
            reg_value &= ~(1 << 0) # clear enable bit
            self._write_register(self.REG_USB_SOURCE_CTRL, reg_value)
        
        # enable sourcing on the specified port
        reg_value |= (port - 1) << 1 # set SOURCE_PORT bit
        reg_value |= (1 << 0) # set enable bit
        self._write_register(self.REG_USB_SOURCE_CTRL, reg_value)
        logging.info(f'USB sourcing enabled on port {port}')
        
    def disable_usb_sourcing(self):
        '''Disable USB-C sourcing.'''
        reg_value = self._read_register(self.REG_USB_SOURCE_CTRL)
        reg_value &= ~(1 << 0) # clear enable bit
        self._write_register(self.REG_USB_SOURCE_CTRL, reg_value)
        logging.info('USB sourcing disabled')
        
    def usb_sourcing_enabled(self):
        '''Check if USB-C sourcing is enabled.
        
        Returns:
            bool: True if USB-C sourcing is enabled, False otherwise
        '''
        reg_value = self._read_register(self.REG_USB_SOURCE_CTRL, reg_bits=8)
        return bool(reg_value & (1 << 0)) # check if enable bit is set
        
    def get_usb_sourcing_port(self):
        '''Get the input port configured for USB-C sourcing.
        
        Returns:
            int or None: Port number (1 or 2) if configured, otherwise None
        '''
        reg_value = self._read_register(self.REG_USB_SOURCE_CTRL, reg_bits=8)
        sourcing_port_bit = (reg_value >> 1) & 0x01
        sourcing_port = sourcing_port_bit + 1 if sourcing_port_bit in (0, 1) else None
        return sourcing_port
        
    
    ### Ship and Shutdown Modes ###
    def enable_ship_mode(self, delay_enabled=False):
        '''Enable ship mode.
        
        Args:
            delay_enabled (bool): Whether to delay 10 seconds before entering ship mode, defaults to False
        '''
        reg_value = self._read_register(self.REG_OTG_CONFIG)
        
        if delay_enabled:
            reg_value |= (1 << 6) # set SHIP_DLY_EN bit to enable 10-second delay
        else:
            reg_value &= ~(1 << 6) # clear SHIP_DLY_EN bit to disable 10-second delay
        
        reg_value |= (1 << 5) # set enable bit
        self._write_register(self.REG_OTG_CONFIG, reg_value)
        logging.info(f'Entering ship mode{" after 10-second delay" if delay_enabled else ""}')
        
    def disable_ship_mode(self):
        '''disable ship mode.'''
        reg_value = self._read_register(self.REG_OTG_CONFIG)
        reg_value &= ~(1 << 5) # clear enable bit
        self._write_register(self.REG_OTG_CONFIG, reg_value)
        logging.info('Exiting ship mode')
        
    def ship_mode_enabled(self):
        '''Check if ship mode is enabled.
        
        Returns:
            bool: True if ship mode is enabled, False otherwise
        '''
        reg_value = self._read_register(self.REG_OTG_CONFIG, reg_bits=8)
        return bool(reg_value & (1 << 5)) # check if enable bit is set
        
    def enable_shutdown_mode(self):
        '''Enter shutdown mode.'''
        reg_value = self._read_register(self.REG_PROTECTION_CTRL)
        reg_value |= (1 << 7) # set enable bit
        self._write_register(self.REG_PROTECTION_CTRL, reg_value)
        logging.info('Entering shutdown mode')
        
    
    ### Fault Status ###
    def update_fault_status(self):
        '''Update local fault status.
        
        Updates the following class variables:
            - input_over_voltage_fault
            - input_under_voltage_fault
            - input_over_current_fault
            - bat_over_voltage_fault
            - bat_over_current_fault
            - bat_temp_fault
            - die_temp_fault
            - charging_fault
        '''
        fault_reg = self._read_register(self.REG_FAULT_STATUS, reg_bits=8)
        
        self.input_over_voltage_fault = bool(fault_reg & (1 << 7))
        self.input_under_voltage_fault = bool(fault_reg & (1 << 6))
        self.input_over_current_fault = bool(fault_reg & (1 << 5))
        self.bat_over_voltage_fault = bool(fault_reg & (1 << 4))
        self.bat_over_current_fault = bool(fault_reg & (1 << 3))
        self.bat_temp_fault = bool(fault_reg & (1 << 2))
        self.die_temp_fault = bool(fault_reg & (1 << 1))
        self.charging_fault = bool(fault_reg & (1 << 0))
        
    
    ### Adapter and Input Port Management ###
    def adapter_connected(self, port):
        '''Check whether an adapter is connected to the specified port.
        
        Args:
            port (int): Port number (1 or 2)
        
        Returns:
            bool: True if an adapter is connected to the specified port, False otherwise
        '''
        if port not in (1, 2):
            raise ValueError('Invalid port number, must be 1 or 2')
    
        input_source_ctrl = self._read_register(self.REG_INPUT_SOURCE_CTRL, reg_bits=8)
        return bool(input_source_ctrl & (1 << 6))

    def get_input_current_limit(self):
        '''Get input current limit.
        
        Returns:
            float: Input current limit in amps
        '''
        reg_value = self._read_register(self.REG_INPUT_SOURCE_CTRL, reg_bits=8)
        return (reg_value & 0xFF) * 0.05 # scale 50mA per bit to amps

    def set_input_current_limit(self, current_a):
        '''Set input current limit.
        
        Args:
            current_a (float): Input current limit in amps
        '''
        current_ma = int(current_a * 1000)
        input_limit_reg = current_ma // 50 # scale amps to 50mA per bit
        self._write_register(self.REG_INPUT_SOURCE_CTRL, input_limit_reg, reg_bits=8)
        logging.info(f'Input current limit set to {current_a}A')

    def get_minimum_system_voltage(self):
        '''Get minimum system voltage.

        Returns:
            float: Minimum system voltage in volts
        '''
        register_value = self._read_register(self.REG_MIN_SYS_VOLTAGE, length=8, end=6) # first 6 bits
        voltage_mv = 2500 + (reg_value * 250)  # 2500mV offset, 250mV per bit
        return voltage_mv / 1000 # millivolts to volts
    
    def set_minimum_system_voltage(self, voltage_v):
        '''Set minimum system voltage.
        
        Range: 2.5 to 16V

        Set at power-on-reset according to the number of battery cells specified by the resistance on the PROG pin:
        1s: 3.5V
        2s: 7V
        3s: 9V
        4s: 12V

        Args:
            voltage_v (float): Minimum system voltage in volts
        '''
        voltage_mv = int(voltage_v * 1000) - 2500 # volts to millivolts, 2500mV offset
        new_value = voltage_mv // 250 # 250mV per bit

        register_value = self._read_register(self.REG_MIN_SYS_VOLTAGE, length=8)
        new_register_value = self._set_bit_value(register_value, new_value)

        self._write_register(self.REG_MIN_SYS_VOLTAGE, new_register_value, length=8)
        logging.info(f'Minimum system voltage set to {voltage_v}V')
    
