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
    # I2C registers
    REG_MIN_SYS_VOLTAGE = 0x00     # minimum system voltage
    REG_CHARGE_VOLTAGE = 0x01      # charge voltage control
    REG_CHARGE_CURRENT = 0x03      # charge current control
    REG_INPUT_VOLTAGE_LIMIT = 0x05 # input voltage limit
    REG_INPUT_CURRENT_LIMIT = 0x06 # input current limit
    REG_PRECHARGE_CTRL = 0x08      # pre-charge control
    REG_TERMINATION_CTRL = 0x09    # charge termination control
    REG_RECHARGE_CTRL = 0x0A       # re-charge control
    REG_USB_SOURCE_VOLTAGE = 0x0B  # USB-C sourcing voltage
    REG_USB_SOURCE_CURRENT = 0x0D  # USB-C sourcing current
    REG_TIMER_CONTROL = 0x0E       # safety timer and watchdog control
    REG_CHARGE_CTRL_0 = 0x0F       # charger control
    REG_CHARGE_CTRL_1 = 0x10       # charger control
    REG_CHARGE_CTRL_2 = 0x11       # charger control
    REG_CHARGE_CTRL_3 = 0x12       # charger control
    REG_CHARGE_CTRL_4 = 0x13       # charger control
    REG_CHARGE_CTRL_5 = 0x14       # charger control
    REG_MPPT_CTRL = 0x15           # MPPT control
    REG_TEMPERATURE_CTRL = 0x16    # temperature control
    REG_NTC_CTRL_0 = 0x17          # NTC control
    REG_NTC_CTRL_1 = 0x18          # NTC control
    REG_ICO_CURRENT_LIMIT = 0x19   # input current limit (from ICO or ILIM_HIZ pin)
    REG_CHARGE_STATUS_0 = 0x1B     # charger status
    REG_CHARGE_STATUS_1 = 0x1C     # charger status
    REG_CHARGE_STATUS_2 = 0x1D     # charger status
    REG_CHARGE_STATUS_3 = 0x1E     # charger status
    REG_CHARGE_STATUS_4 = 0x1F     # charger status
    REG_FAULT_STATUS_0 = 0x20      # fault status
    REG_FAULT_STATUS_1 = 0x21      # fault status
    REG_CHARGE_FLAG_0 = 0x22       # charger flag
    REG_CHARGE_FLAG_1 = 0x23       # charger flag
    REG_CHARGE_FLAG_2 = 0x24       # charger flag
    REG_CHARGE_FLAG_3 = 0x25       # charger flag
    REG_FAULT_FLAG_0 = 0x26        # fault flag
    REG_FAULT_FLAG_1 = 0x27        # fault flag
    REG_CHARGE_MASK_0 = 0x28       # charger flag
    REG_CHARGE_MASK_1 = 0x29       # charger flag
    REG_CHARGE_MASK_2 = 0x2A       # charger flag
    REG_CHARGE_MASK_3 = 0x2B       # charger flag
    REG_FAULT_MASK_0 = 0x2C        # fault flag
    REG_FAULT_MASK_1 = 0x2D        # fault flag
    REG_ADC_CTRL = 0x2E            # ADC control
    REG_ADC_FUNCTION_DISABLE_0 = 0x2F # ADC function disable
    REG_ADC_FUNCTION_DISABLE_1 = 0x30 # ADC function disable
    REG_ADC_IBUS = 0x31            # input current ADC result
    REG_ADC_IBAT = 0x33            # battery charge/discharge current ADC result
    REG_ADC_VBUS = 0x35            # input voltage ADC result
    REG_ADC_VAC1 = 0x37            # port 1 voltage ADC result
    REG_ADC_VAC2 = 0x39            # port 2 voltage ADC result
    REG_ADC_VBAT = 0x3B            # battery voltage ADC result
    REG_ADC_VSYS = 0x3D            # system voltage ADC result
    REG_ADC_TS = 0x3F              # external temperature sensor ADC result
    REG_ADC_TDIE = 0x41            # die temperature ADC result
    REG_ADC_DP = 0x43              # USB data plus ADC result
    REG_ADC_DM = 0x45              # USB data minus ADC result
    REG_DPDM_DRIVER = 0x47         # DPDM driver
    REG_PART_INFO = 0x48           # part information
    
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
    BAT_STATE_NOT_CHARGING = 'not charging'
    BAT_STATE_TRICKLE_CHARGE = 'trickle charge'
    BAT_STATE_PRECHARGE = 'precharge'
    BAT_STATE_FAST_CHARGE = 'fast charge'
    BAT_STATE_TAPER_CHARGE = 'taper charge'
    BAT_STATE_TOP_OFF_TIMER = 'top-off timer charging'
    BAT_STATE_CHARGED = 'charged'
    BAT_STATES = [BAT_STATE_NOT_CHARGING, BAT_STATE_TRICKLE_CHARGE, BAT_STATE_PRECHARGE, BAT_STATE_FAST_CHARGE, BAT_STATE_TAPER_CHARGE, BAT_STATE_TOP_OFF_TIMER, BAT_STATE_CHARGED]
    
    # ADC modes
    ADC_MODE_ONE_SHOT     = 'one-shot'
    ADC_MODE_CONTINUOUS   = 'continuous'
    ADC_MODES = [ADC_MODE_ONE_SHOT, ADC_MODE_CONTINUOUS]

    # faults
    FAULT_IBAT_REGULATION        = 'IBAT regulation fault'
    FAULT_VBUS_OVER_VOLTAGE      = 'VBUS over-voltage fault'
    FAULT_VBAT_OVER_VOLTAGE      = 'VBAT over-voltage fault'
    FAULT_IBUS_OVER_CURRENT      = 'IBUS over-current fault'
    FAULT_IBAT_OVER_CURRENT      = 'IBAT over-current fault'
    FAULT_CONVERTER_OVER_CURRENT = 'Converter over-current fault'
    FAULT_VAC2_OVER_VOLTAGE      = 'VAC2 over-voltage fault'
    FAULT_VAC1_OVER_VOLTAGE      = 'VAC1 over-voltage fault'
    FAULT_VSYS_SHORT             = 'VSYS short circuit fault'
    FAULT_VSYS_OVER_VOLTAGE      = 'VSYS over-voltage fault'
    FAULT_OTG_OVER_VOLTAGE       = 'OTG over-voltage fault'
    FAULT_OTG_UNDER_VOLTAGE      = 'OTG under-voltage fault'
    FAULT_TDIE_SHUTDOWN          = 'TDIE shutdown fault'
    FAULTS = [FAULT_IBAT_REGULATION, FAULT_VBUS_OVER_VOLTAGE, FAULT_VBAT_OVER_VOLTAGE, FAULT_IBUS_OVER_CURRENT, FAULT_IBAT_OVER_CURRENT, FAULT_CONVERTER_OVER_CURRENT, FAULT_VAC2_OVER_VOLTAGE, FAULT_VAC1_OVER_VOLTAGE, FAULT_VSYS_SHORT, FAULT_VSYS_OVER_VOLTAGE, FAULT_OTG_OVER_VOLTAGE, FAULT_OTG_UNDER_VOLTAGE, FAULT_TDIE_SHUTDOWN]


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
        self.state = self.BAT_STATE_NOT_CHARGING
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
        self.dp = 0.0
        '''USB data plus reading in volts'''
        self.dm = 0.0
        '''USB data minus reading in volts'''
        
        # initalize fault status variables
        self.fault_ibat_regulation = False
        self.fault_vbus_over_voltage = False
        self.fault_vbat_over_voltage = False
        self.fault_ibus_over_current = False
        self.fault_ibat_over_current = False
        self.fault_converter_over_current = False
        self.fault_vac2_over_voltage = False
        self.fault_vac1_over_voltage = False
        self.fault_vsys_short = False
        self.fault_vsys_over_voltage = False
        self.fault_otg_over_voltage = False
        self.fault_otg_under_voltage = False
        self.fault_tdie_shutdown = False
        
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
                self.state = self.battery_charge_stage()
            
            if self.state != previous_state:
                logging.info(f'State change: {self.state}')
                # state change callback
                if self.state_change_callback is not None:
                    self.state_change_callback(self.state)
            
            # update fault status
            previous_faults = {
                self.FAULT_IBAT_REGULATION: self.fault_ibat_regulation,
                self.FAULT_VBUS_OVER_VOLTAGE: self.fault_vbus_over_voltage,
                self.FAULT_VBAT_OVER_VOLTAGE: self.fault_vbat_over_voltage,
                self.FAULT_IBUS_OVER_CURRENT: self.fault_ibus_over_current,
                self.FAULT_IBAT_OVER_CURRENT: self.fault_ibat_over_current,
                self.FAULT_CONVERTER_OVER_CURRENT: self.fault_converter_over_current,
                self.FAULT_VAC2_OVER_VOLTAGE: self.fault_vac2_over_voltage,
                self.FAULT_VAC1_OVER_VOLTAGE: self.fault_vac1_over_voltage,
                self.FAULT_VSYS_SHORT: self.fault_vsys_short,
                self.FAULT_VSYS_OVER_VOLTAGE: self.fault_vsys_over_voltage,
                self.FAULT_OTG_OVER_VOLTAGE: self.fault_otg_over_voltage,
                self.FAULT_OTG_UNDER_VOLTAGE: self.fault_otg_under_voltage,
                self.FAULT_TDIE_SHUTDOWN: self.fault_tdie_shutdown
            }

            with self._fault_lock:
                self.update_fault_status()
            
            current_faults = {
                self.FAULT_IBAT_REGULATION: self.fault_ibat_regulation,
                self.FAULT_VBUS_OVER_VOLTAGE: self.fault_vbus_over_voltage,
                self.FAULT_VBAT_OVER_VOLTAGE: self.fault_vbat_over_voltage,
                self.FAULT_IBUS_OVER_CURRENT: self.fault_ibus_over_current,
                self.FAULT_IBAT_OVER_CURRENT: self.fault_ibat_over_current,
                self.FAULT_CONVERTER_OVER_CURRENT: self.fault_converter_over_current,
                self.FAULT_VAC2_OVER_VOLTAGE: self.fault_vac2_over_voltage,
                self.FAULT_VAC1_OVER_VOLTAGE: self.fault_vac1_over_voltage,
                self.FAULT_VSYS_SHORT: self.fault_vsys_short,
                self.FAULT_VSYS_OVER_VOLTAGE: self.fault_vsys_over_voltage,
                self.FAULT_OTG_OVER_VOLTAGE: self.fault_otg_over_voltage,
                self.FAULT_OTG_UNDER_VOLTAGE: self.fault_otg_under_voltage,
                self.FAULT_TDIE_SHUTDOWN: self.fault_tdie_shutdown
            }
            
            for fault, previous_status in previous_faults.items():
                current_status = current_faults[fault]
                if previous_status != current_status:
                    logging.info(f'Fault status change: {fault} {"active" if current_status else "inactive"}')
                    # fault change callback
                    if self.fault_change_callback is not None:
                        self.fault_change_callback(fault, current_status)
        
    def _read_register(self, register, register_length, start=None, end=None):
        '''Read register value.
        
        *start* and *end* bit positions are zero-based. For example, when reading the last bit in an 8-bit register, *start* = 7.

        Args:
            register (int): Register address to read (example: 0x20)
            register_length (int): Register size (number of bits), should be 8 or 16
            start (int): First bit for desired value, defaults to 0
            end (int): Last bit position for desired value, defaults to *register_length*-1
        
        Returns:
            bytes: Register value
        
        Raises:
            ValueError: Register length must be 8 or 16
        '''
        if register_length not in (8, 16):
            raise ValueError('Register length must be 8 or 16')

        if start is None:
            start = 0

        if end is None:
            end = length - 1

        if register_length == 8:
            value = self.bus.read_byte_data(self.address, register) # read 8 bits
        elif register_length == 16:
            value = self.bus.read_word_data(self.address, register) # read 16 bits

        return self._get_bit_value(value, register_length, start, end)
        
    def _write_register(self, register, value, register_length, value_length=None, start=0):
        '''Write register value

        Use *value_length* to indicate a value less than the full register length. *register_length* and *start* should be specified to indicate the size of the register and the starting bit for the value in the register. A typical use case is setting bits in a configuration register.

        *start* bit position is zero-based. For example, when writting the last bit in an 8-bit register, *start* = 7.
        
        Args:
            register (int): Register address to write to (example: 0x20)
            value (int): Value to write to register
            register_length (int): Register size (number of bits), should be 8 or 16
            value_length (int, None): Number of bits to write, defaults to *register_length*
            start (int): First bit position to write value, defaults to 0
        
        Raises:
            ValueError: Register length must be 8 or 16
        '''
        if register_length not in (8, 16):
            raise ValueError('Register length must be 8 or 16')

        if value_length is None:
            value_length = register_length

        if value_length < register_length:
            # set subset of bits
            register_value = self._read_register(register, register_length=register_length)
            value = self._set_bit_value(register_value, value, register_length, value_length, start)

        if register_length == 8:
            reg_value = self.bus.write_byte_data(self.address, register, value) # write 8 bits
        elif register_length == 16:
            reg_value = self.bus.write_word_data(self.address, register, value) # write 16 bits

    def _bit_mask(self, length):
        '''Generate a bit mask of specified length.

        Args:
            length (int): Number of bits in bit mask

        Returns:
            int: bit mask of specified length
        '''
        return (1 << length) - 1
    
    def _get_bit_value(self, register_value, register_length, start, end):
        '''Get subset of bits.
        
        *start* and *end* bit positions are zero-based. For example, when reading the last bit in an 8-bit register, *start* = 7.

        Args:
            register_value (int): Complete value of the register
            register_length (int): Register size (number of bits), should be 8 or 16
            start (int): First bit for desired value
            end (int): Last bit position for desired value
        
        Returns:
            int: Specified subset of register bits
        
        Raises:
            ValueError: Register length must be 8 or 16
        '''
        if start > 0:
            value = (register_value >> start)

        if end < (register_length - 1):
            value_length = end - start + 1
            value = value & self._bit_mask(value_length)

        return value

    def _set_bit_value(self, register_value, new_value, register_length, new_value_length, start=0):
        '''Set subset of bits.
        
        *start* bit position is zero-based. For example, when writting the last bit in an 8-bit register, *start* = 7.

        Args:
            register_value (int): Complete value of register to update
            new_value (int): Value of bit(s) to update in register value
            register_length (int): Register size (number of bits), should be 8 or 16
            new_value_length (int): Number of bits in *new_value*
            start (int): First bit position to write *new_value*, defaults to 0

        Returns:
            int: Register value with specified bits replaced

        Raises:
            ValueError: Value too large for register at specified position
        '''
        if start + new_value_length > register_length:
            raise ValueError('Value too large for register at specified position')

        mask = self._bit_mask(new_value_length) << start
        new_reg_value = (register_value & ~mask) | ((new_value << start) & mask) 
        return new_reg_value
    
    def close(self):
        '''Close the I2C bus connection.'''
        if self.bus:
            self.bus.close()
            print('I2C bus closed')
        
    
    ### ADC Functions ###
    def enable_adc(self):
        '''Set ADC enable bit.

        Reset by: power-on-reset, watchdog
        Reset to: disabled
        '''
        # write 0x01 to bit 7 of the ADC control register
        self._write_register(self.REG_ADC_CTRL, 0x01, register_length=8, value_length=1, start=7)
        logging.info('ADC enabled')
        
    def disable_adc(self):
        '''Clear ADC enable bit.

        Reset by: power-on-reset, watchdog
        Reset to: disabled
        '''
        # write 0x00 to bit 7 of the ADC control register
        self._write_register(self.REG_ADC_CTRL, 0x00, register_length=8, value_length=1, start=7)
        logging.info('ADC disabled')
        
    def adc_enabled(self):
        '''Check if ADC is enabled.
        
        Returns:
            bool: True if ADC is enabled, False otherwise
        '''
        # read bit 7 of the ADC control register
        return bool(self._read_register(self.REG_ADC_CTRL, register_length=8, start=7, end=7))
        
    def set_adc_one_shot_mode(self):
        '''Set ADC to one-shot mode.

        Reset by: power-on-reset
        Reset to: continuous mode
        '''
        # write 0x01 to bit 6 of the ADC control register
        self._write_register(self.REG_ADC_CTRL, 0x01, register_length=8, value_length=1, start=6)
        self.adc_mode = self.ADC_MODE_ONE_SHOT
        logging.info('ADC in one-shot mode')
        
    def set_adc_continuous_mode(self):
        '''Set the ADC to continuous mode.

        Reset by: power-on-reset
        Reset to: continuous mode
        '''
        # write 0x00 to bit 6 of the ADC control register
        self._write_register(self.REG_ADC_CTRL, 0x00, register_length=8, value_length=1, start=6)
        self.adc_mode = self.ADC_MODE_CONTINUOUS
        logging.info('ADC in continuous mode')
        
    def get_adc_mode(self):
        '''Get ADC operating mode.
        
        Returns:
            str: *ADC_MODE_ONE_SHOT* or *ADC_MODE_CONTINUOUS*
        '''
        # read bit 6 of the ADC control register
        register_value = self._read_register(self.REG_ADC_CTRL, register_length=8, start=6, end=6)
        
        if register_value == 1:
            return self.ADC_MODE_ONE_SHOT
        else:
            return self.ADC_MODE_CONTINUOUS

    #TODO
    # - check IBAT ADC register sign bit handling
    # - check TS ADC register value handling of % value throughout module
    def update_adc_readings(self):
        '''Update local ADC reading values.
        
        If ADC is in one-shot mode, ADC is enabled before updating readings and then disabled after updates are complete.
        '''
        if self.adc_mode == self.ADC_MODE_ONE_SHOT:
            self.enable_adc()
            time.sleep(0.1) # allow time for ADC to complete conversions and settle
        
        self.ibus = self._read_register(self.REG_ADC_IBUS, register_length=16) / 1000 # convert to A, 1mA per bit
        self.ibat = self._read_register(self.REG_ADC_IBAT, register_length=16) / 1000 # convert to A, 1mA per bit
        self.vbus = self._read_register(self.REG_ADC_VBUS, register_length=16) / 1000 # convert to V, 1mV per bit
        self.vac1 = self._read_register(self.REG_ADC_VAC1, register_length=16) / 1000 # convert to V, 1mV per bit
        self.vac2 = self._read_register(self.REG_ADC_VAC2, register_length=16) / 1000 # convert to V, 1mV per bit
        self.vbat = self._read_register(self.REG_ADC_VBAT, register_length=16) / 1000 # convert to V, 1mV per bit
        self.vsys = self._read_register(self.REG_ADC_VSYS, register_length=16) / 1000 # convert to V, 1mV per bit
        self.tbat = self._read_register(self.REG_ADC_TS,   register_length=16) * 0.0976563 # 0.0976563% per bit
        self.tdie = self._read_register(self.REG_ADC_TDIE, register_length=16) * 0.5 # convert to °C, 0.5°C per bit
        self.dp   = self._read_register(self.REG_ADC_DP,   register_length=16) / 1000 # convert to V, 1mV per bit
        self.dm   = self._read_register(self.REG_ADC_DM,   register_length=16) / 1000 # convert to V, 1mV per bit
        
        if self.adc_mode == self.ADC_MODE_ONE_SHOT:
            self.disable_adc()
        
    
    ### Solar MPPT Input ###
    def enable_mppt(self):
        '''Set MPPT (Maximum Power Point Tracking) enable bit.

        Reset by: power-on-reset
        Reset to: disabled
        '''
        # write 0x01 to bit 0 of the MPPT control register
        self._write_register(self.REG_MPPT_CTRL, 0x01, register_length=8, value_length=1, start=0, end=0)
        logging.info(f'MPPT enabled')
        
    def disable_mppt(self):
        '''Clear MPPT (Maximum Power Point Tracking) enable bit.

        Reset by: power-on-reset
        Reset to: disabled
        '''
        # write 0x00 to bit 0 of the MPPT control register
        self._write_register(self.REG_MPPT_CTRL, 0x00, register_length=8, value_length=1, start=0, end=0)
        logging.info('MPPT disabled')
        
    def mppt_enabled(self):
        '''Check if MPPT (Maximum Power Point Tracking) is enabled.
        
        Returns:
            bool: True if MPPT is enabled, False otherwise
        '''
        # read bit 1 of the MPPT control register
        return bool(self._read_register(self.REG_MPPT_CTRL, register_length=8, start=0, end=0))
        
    
    ### Battery Charging ###
    def get_charge_voltage(self):
        '''Get configured battery charge voltage.
        
        Returns:
            float: Battery charge voltage in volts
        '''
        return self._read_register(self.REG_CHARGE_VOLTAGE, register_length=16) / 100 # convert mV to V, 10mV per bit
        
    def set_charge_voltage(self, voltage_v):
        '''Set battery charge voltage.

        Reset by: power-on-reset
        Reset to: based on resistance at PROG pin

        Range: 3 - 18.8V
        Bit step size: 10mV
        
        Args:
            voltage_v (float): Battery charge voltage in volts
        '''
        voltage_reg = int((voltage_v * 1000) / 10) # convert V to mV, 10mV per bit
        self._write_register(self.REG_CHARGE_VOLTAGE, voltage_reg, register_length=16)
        self.charge_voltage = voltage_v
        logging.info(f'Battery charge voltage set to {voltage_v}V')
        
    def get_charge_current(self):
        '''Get configured battery charging current.
        
        Returns:
            float: Battery charge current in amps
        '''
        return self._read_register(self.REG_CHARGE_CURRENT, register_length=16) / 100 # convert mV to V, 10mV per bit
        
    def set_charge_current(self, current_a):
        '''Set battery charging current.

        Reset by: power-on-reset, watchdog
        Reset to: based on resistance at PROG pin

        Range: 0.05 - 5 A
        Bit step size: 10mA
        
        Args:
            current_a (float): Battery charge current in amps
        '''
        current_reg = int((current_a * 1000) / 10) # convert A to mA, 10mA per bit
        self._write_register(self.REG_CHARGE_CURRENT, current_reg, register_length=16)
        self.charge_current = current_a
        logging.info(f'Battery charge current set to {current_a}A')
        
    def enable_charging(self):
        '''Enable battery charging.

        Reset by: power-on-reset, watchdog
        Reset to: enabled
        '''
        # write 0x01 to bit 5 of charge control register 0
        self._write_register(self.REG_CHARGE_CTRL_0, 0x01, register_length=8, value_length=1, start=5, end=5)
        logging.info('Battery charging enabled')
        
    def disable_charging(self):
        '''Disable battery charging.

        Reset by: power-on-reset, watchdog
        Reset to: enabled
        '''
        # write 0x00 to bit 5 of charge control register 0
        self._write_register(self.REG_CHARGE_CTRL_0, 0x00, register_length=8, value_length=1, start=5, end=5)
        logging.info('Battery charging disabled')
        
    def charging_enabled(self):
        '''Whether battery charging is enabled.
        
        Returns:
            bool: True if charging is enabled, False otherwise
        '''
        # read bit 5 of charge control register 0
        return bool(self._read_register(self.REG_CHARGE_CTRL_0, register_length=8, start=5, end=5))

    #TODO
    # - check IBAT sign handling
    def battery_charging(self):
        '''Whether battery is currently being charged.'''
        return bool(self.charging_enabled() and self.ibat > 0)

    #TODO remove, should be managed by a fuel gauge ic
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
        ''' Get current battery charge stage.
        
        Returns:
            str: One of BQ25798.BAT_STATE_*
        '''
        charge_stage_map = {
            0x0: self.BAT_STATE_NOT_CHARGING,
            0x1: self.BAT_STATE_TRICKLE_CHARGE,
            0x2: self.BAT_STATE_PRECHARGE,
            0x3: self.BAT_STATE_FAST_CHARGE,
            0x4: self.BAT_STATE_TAPER_CHARGE,
            0x6: self.BAT_STATE_TOP_OFF_TIMER,
            0x7: self.BAT_STATE_CHARGED
        }
        
        # read bits 5-7 of charge status register 1
        charge_stage = self._read_register(self.REG_CHARGE_STATUS_1, register_length=8, start=5, end=7)
        return charge_stage_map[charge_stage]

    #TODO remove, should be managed by a fuel gauge ic
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
            
    
    ### USB Sourcing ###
    def get_usb_sourcing_voltage(self):
        '''Get configured USB-C sourcing voltage.
        
        Returns:
            float: USB-C sourcing voltage in volts
        '''
        return ((self._read_register(self.REG_USB_SOURCE_VOLTAGE, register_length=16) + 2800) * 10) / 1000 # 2800mV offset, 10mV per bit, convert mV to V

    def set_usb_sourcing_voltage(self, voltage_v):
        '''Set USB-C sourcing voltage.

        Reset by: power-on-reset, watchdog
        Reset to: 5V

        Range: 3 - 18.8V
        Bit step size: 10mV
        
        Args:
            voltage_v (float): USB-C sourcing voltage in volts
        '''
        voltage_reg = int((voltage_v * 1000 - 2800) / 10) # convert V to mV, 2800mV offset, 10mV per bit
        self._write_register(self.REG_USB_SOURCE_VOLTAGE, voltage_reg, register_length=16, value_length=10, start=0)
        logging.info(f'USB sourcing voltage set to {voltage_v}V')

    def get_usb_sourcing_current(self):
        '''Get configured USB-C sourcing current.
        
        Returns:
            float: USB-C sourcing current in amps
        '''
        return (self._read_register(self.REG_CHARGE_CURRENT, register_length=16) * 40) / 1000 # 40mA per bit, convert mA to A
        
    def set_usb_sourcing_current(self, current_a):
        '''Set USB-C sourcing current.

        Reset by: power-on-reset, watchdog
        Reset to: 3.04A

        Range: 0.160 - 3.36A
        Bit step size: 40mA
        
        Args:
            current_a (float): USB-C sourcing current in amps
        '''
        current_reg = int((current_a * 1000) / 40) # convert A to mA, 40mV per bit
        # write current to bits 0-6 of USB source current register
        self._write_register(self.REG_USB_SOURCE_CURRENT, current_reg, register_length=8, value_length=7, start=0)
        logging.info(f'USB sourcing current set to {current_a}A')

    def enable_usb_sourcing(self):
        '''Enable USB-C sourcing (OTG).
        
        Reset by: power-on-reset, watchdog
        Reset to: disabled
        '''
        # write 0x01 to bit 6 of charge control register 3
        self._write_register(self.REG_CHARGE_CTRL_3, 0x01, register_length=8, value_length=1, start=6)
        logging.info('USB sourcing enabled')
        
    def disable_usb_sourcing(self):
        '''Disable USB-C sourcing (OTG).

        Reset by: power-on-reset, watchdog
        Reset to: disabled
        '''
        # write 0x00 to bit 6 of charge control register 3
        self._write_register(self.REG_CHARGE_CTRL_3, 0x00, register_length=8, value_length=1, start=6)
        logging.info('USB sourcing disabled')
        
    def usb_sourcing_enabled(self):
        '''Whether USB-C sourcing (OTG) is enabled.
        
        Returns:
            bool: True if USB-C sourcing is enabled, False otherwise
        '''
        # read bit 6 of charge control register 3
        return bool(self._read_register(self.REG_CHARGE_CTRL_3, register_length=8, start=6, end=6))
        

    ### Ship and Shutdown Modes ###
    def set_ship_fet_present(self, fet_present=True):
        '''Set whether ship fet is present.

        Args:
            fet_present (bool): Whether ship FET is present (installed)
        '''
        reg_value = int(fet_present)
        # write bit 7 in charge control register 5
        self._write_register(self.REG_CHARGE_CTRL_5, reg_value, register_length=8, value_length=1, start=7)
        logging.info(f'Ship FET {"present" if fet_present else "not present"}')

    def enable_ship_mode(self, delay_enabled=True):
        '''Enable ship mode.

        Reset by: power-on-reset
        yyReset to: disabled

        Delay on entering ship mode is enabled by default on power-on-reset.
        
        Args:
            delay_enabled (bool): Whether to delay 10 seconds before entering ship mode, defaults to True
        '''
        if delay_enabled:
            # write 0x00 to bit 0 in charge control register 2
            self._write_register(self.REG_CHARGE_CTRL_2, 0x00, register_length=8, value_length=1, start=0)
        else:
            # write 0x01 to bit 0 in charge control register 2
            self._write_register(self.REG_CHARGE_CTRL_2, 0x01, register_length=8, value_length=1, start=0)
        
        logging.info(f'Entering ship mode{" after 10-second delay" if delay_enabled else ""}')
        # write 0x02 to bits 1-2 in charge control register 2
        self._write_register(self.REG_CHARGE_CTRL_2, 0x02, register_length=8, value_length=2, start=1)
        
    def disable_ship_mode(self):
        '''Disable ship mode.

        Reset by: power-on-reset
        Reset to: disabled
        '''
        # write 0x00 to bits 1-2 in charge control register 2
        self._write_register(self.REG_CHARGE_CTRL_2, 0x00, register_length=8, value_length=2, start=1)
        logging.info('Exiting ship mode')
        
    def ship_mode_enabled(self):
        '''Whether ship mode is enabled.
        
        Returns:
            bool: True if ship mode is enabled, False otherwise
        '''
        # read bits 1-2 of charge control register 2
        reg_value = self._read_register(self.REG_CHARGE_CTRL_2, register_length=8, start=1, end=2))
        if reg_value == 0x02:
            return True
        else:
            return False
        
    def enable_shutdown_mode(self):
        '''Enter shutdown mode.'''
        logging.info('Entering shutdown mode')
        # write 0x01 to bits 1-2 in charge control register 2
        self._write_register(self.REG_CHARGE_CTRL_2, 0x02, register_length=8, value_length=2, start=1)
        
    
    ### Fault Status ###
    def update_fault_status(self):
        '''Update local fault status variables.'''
        fault_reg_0 = self._read_register(self.REG_FAULT_STATUS_0, register_length=8)
        fault_reg_1 = self._read_register(self.REG_FAULT_STATUS_1, register_length=8)
        
        self.fault_ibat_regulation = bool(self._get_bit_value(fault_reg_0, register_length=8, start=7, end=7))
        self.fault_vbus_over_voltage = bool(self._get_bit_value(fault_reg_0, register_length=8, start=6, end=6))
        self.fault_vbat_over_voltage = bool(self._get_bit_value(fault_reg_0, register_length=8, start=5, end=5))
        self.fault_ibus_over_current = bool(self._get_bit_value(fault_reg_0, register_length=8, start=4, end=4))
        self.fault_ibat_over_current = bool(self._get_bit_value(fault_reg_0, register_length=8, start=3, end=3))
        self.fault_converter_over_current = bool(self._get_bit_value(fault_reg_0, register_length=8, start=2, end=2))
        self.fault_vac2_over_voltage = bool(self._get_bit_value(fault_reg_0, register_length=8, start=1, end=1))
        self.fault_vac1_over_voltage = bool(self._get_bit_value(fault_reg_0, register_length=8, start=0, end=0))
        self.fault_vsys_short = bool(self._get_bit_value(fault_reg_1, register_length=8, start=7, end=7))
        self.fault_vsys_over_voltage = bool(self._get_bit_value(fault_reg_1, register_length=8, start=6, end=6))
        self.fault_otg_over_voltage = bool(self._get_bit_value(fault_reg_1, register_length=8, start=5, end=5))
        self.fault_otg_under_voltage = bool(self._get_bit_value(fault_reg_1, register_length=8, start=4, end=4))
        self.fault_tdie_shutdown = bool(self._get_bit_value(fault_reg_1, register_length=8, start=2, end=2))
        
    
    ### Adapter and Input Port Management ###
    def adapter_connected(self, port=None):
        '''Whether an adapter is connected.

        If *port* is *None*, returns *True* if an adapter is connected to either port, or *False* if an adapter is not connect to either port.
        
        Args:
            port (int, None): Port number (1 or 2), defaults to None
        
        Returns:
            bool: True if an adapter is connected, False otherwise
        '''
        if port is not None and port not in (1, 2):
            raise ValueError('Invalid port number, must be 1 or 2')

        if port is None:
            reg_value = self._read_register(self.REG_CHARGE_CTRL_0, register_length=8)
            # read bit 1 from charge control register 0
            vac1 = bool(self._get_bit_value(reg_value, register_length=8, start=1, end=1))
            # read bit 2 from charge control register 0
            vac1 = bool(self._get_bit_value(reg_value, register_length=8, start=2, end=2))
            return any(vac1, vac2)

        # read bit 1 or 2 (depending on specified port) from charge control register 0
        return bool(self._read_register(self.REG_CHARGE_CTRL_0, register_length=8, start=port, end=port))

    #TODO pick up here
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
    
