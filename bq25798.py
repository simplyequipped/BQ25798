# Datasheet: https://www.ti.com/lit/ds/symlink/bq25798.pdf
#
#TODO
# - improve fault monitoring by storing previous fault state and checking for status change in job loop
# - improve docs


import time
import threading
import logging
from logging.handlers import RotatingFileHandler

import smbus2

# register definitions
REG_CHARGE_VOLTAGE = 0x04      # charge voltage control register
REG_CHARGE_CURRENT = 0x02      # charge current control register
REG_PRECHARGE_CURRENT = 0x06   # pre-charge and termination current control register
REG_ADC_CTRL = 0x2C            # ADC control register
REG_USB_SOURCE_VOLTAGE = 0x16  # USB-C sourcing voltage register
REG_USB_SOURCE_CURRENT = 0x18  # USB-C sourcing current register
REG_USB_SOURCE_CTRL = 0x19     # USB-C source control register
REG_OTG_CONFIG = 0x09          # OTG and ship mode control register
REG_MPPT_CTRL = 0x1A           # mppt control register
REG_INPUT_SOURCE_CTRL = 0x1B   # input source control register
REG_MIN_SYS_VOLTAGE = 0x0E     # minimum system voltage control register
REG_TIMER_CONTROL = 0x12       # safety timer and watchdog control register
REG_FAULT_STATUS = 0x20        # fault status register
REG_PROTECTION_CTRL = 0x1C     # protection control register

REG_ADC_IBUS = 0x34            # input current ADC result register
REG_ADC_IBAT = 0x36            # battery charge/discharge current ADC result register
REG_ADC_VBUS = 0x30            # input voltage ADC result register
REG_ADC_VPMID = 0x38           # PMID voltage ADC result register
REG_ADC_VBAT = 0x32            # battery voltage ADC result register
REG_ADC_VSYS = 0x3A            # system voltage ADC result register
REG_ADC_TS = 0x3C              # external temperature sensor ADC result register
REG_ADC_TDIE = 0x3E            # die temperature ADC result register

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
    # battery chemistry
    BAT_CHEMISTRY_LIFEPO4 = 'LiFePo4'
    BAT_CHEMISTRY_LI_ION  = 'Li-ion'
    BAT_CHEMISTRY_NIMH    = 'NiMH'

    BAT_CHEMISTRY = [BAT_CHEMISTRY_LIFEPO4, BAT_CHEMISTRY_LI_ION, BAT_CHEMISTRY_NIMH]

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

    # battery states
    BAT_STATE_DISCHARGING = 'discharging'
    BAT_STATE_CHARGING    = 'charging'
    BAT_STATE_CHARGED     = 'charged'

    BAT_STATES = [BAT_STATE_DISCHARGING, BAT_STATE_CHARGING, BAT_STATE_CHARGED]

    # adc modes
    ADC_MODE_ONE_SHOT     = 'one-shot'
    ADC_MODE_CONTINUOUS   = 'continuous'

    ADC_MODES = [ADC_MODE_ONE_SHOT, ADC_MODE_CONTINUOUS]

    def __init__(self, i2c_bus=1, i2c_address=0x6B, charge_voltage, charge_current, bat_series_cells, bat_chemistry, bat_capacity_ah):
        '''
        Initialize the BQ25798 module.

        Args:
            i2c_bus (int): I2C bus number, defaults to 1
            i2c_address (int): I2C address of the BQ25798, defaults 0x6B
            charge_voltage (float): Battery charge voltage in volts
            charge_current (float): Battery charge current in amps
            bat_series_cells (int): Number of series battery cells (typically 1-4)
            bat_chemistry (str): Battery chemistry (LiFePo4, Li-ion, NiMH)
            bat_capacity_ah (float): Battery capacity in amp hours
        '''
        self.bus = smbus2.SMBus(i2c_bus)
        self.address = i2c_address
        self.charge_voltage = None
        self.charge_current = None
        self.async_update_seconds = 3
        self.bat_series_cells = bat_series_cells
        self.bat_capacity_ah = bat_capacity_ah
        
        if bat_chemistry in self.BAT_CHEMISTRY:
            self.bat_chemistry = bat_chemistry
        else:
            raise ValueError('Unsupported battery chemistry')

        # validate battery charging parameters
        if not (0 < charge_voltage <= 18.8:
            raise ValueError('Invalid charge voltage')
        if not (0 < charge_current <= 5):
    raise ValueError('Invalid charge current')

        self.adc_mode = None
        self.state = self.BAT_STATE_DISCHARGING
        self.state_lock = threading.Lock()
        self.adc_lock = threading.Lock()
        self.fault_lock = threading.Lock()
        
        self.state_change_callback = None
        self.fault_change_callback = None

        # initialize ADC readings
        self.vbus = 0.0  # input voltage (volts)
        self.ibus = 0.0  # input current (amps)
        self.vbat = 0.0  # battery voltage (volts)
        self.ibat = 0.0  # battery current (amps)
        self.vpmid = 0.0  # PMID voltage (volts)
        self.vsys = 0.0  # system voltage (volts)
        self.ts = 0.0  # external temperature sensor (°C)
        self.tdie = 0.0  # die temperature (°C)

        # initalize fault status
        self.input_over_voltage_fault = False
        self.input_under_voltage_fault = False
        self.input_over_current_fault = False
        self.battery_over_voltage_fault = False
        self.battery_over_current_fault = False
        self.temperature_sensor_fault = False
        self.die_over_temperature_fault = False
        self.charging_fault = False

        logging.info(f'BQ25798 initalized on I2C address {self.address}')

        # initialize configuration
        self.set_charge_parameters(charge_voltage, charge_current)
        self.set_adc_one_shot_mode()
        self.disable_mppt()

        # initialize adc readings
        self.update_adc_readings()
        # initialize fault status
        self.update_fault_status()

        # initialize state
        # start async adc readings and charge state loop
        job_thread = threading.Thread(target=self._job_loop, daemon=True)
        job_thread.start()

    def _job_loop(self):
        # SMBus.fd is set to None after SMBus.close() is called
        while self.bus.fd is not None:
            time.sleep(self.async_update_seconds)

            # update adc readings
            with self.adc_lock:
                self.update_adc_readings()

            # update state
            previous_state = self.state

            with self.state_lock:
                if self.charging()
                    if self.battery_percentage() < 100:
                        self.state = self.BAT_STATE_CHARGING
                    else:
                        self.state = self.BAT_STATE_CHARGED
                else:
                    self.state = self.BAT_STATE_DISCHARGING

            if self.state != previous_state
                logging.info(f'State change: {self.state}')

                # state change callback
                if self.state_change_callback is not None:
                    self.state_change_callback(self.state)

            # update fault status
            with self.fault_lock:
                previous_faults = {
                    'input_over_voltage_fault': self.input_over_voltage_fault,
                    'input_under_voltage_fault': self.input_under_voltage_fault,
                    'input_over_current_fault': self.input_over_current_fault,
                    'battery_over_voltage_fault': self.battery_over_voltage_fault,
                    'battery_over_current_fault': self.battery_over_current_fault,
                    'temperature_sensor_fault': self.temperature_sensor_fault,
                    'die_over_temperature_fault': self.die_over_temperature_fault,
                    'charging_fault': self.charging_fault,
                }

                self.update_fault_status()

                for fault, previous_status in previous_faults.items():
                    current_status = getattr(self, fault)
                    if previous_status != current_status:
                        logging.info(f'Fault state change: {fault} {'active' if current_status else 'inactive'}')

                # fault change callback
                if self.fault_change_callback is not None:
                    self.fault_change_callback(fault, current_status)
        
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
            bool: True if ADC is enabled, False otherwise
        '''
        reg_value = self._read_register(REG_ADC_CTRL, reg_bits=8)
        return bool(reg_value & (1 << 0))  # check if ADC_EN bit is set
        
    def set_adc_one_shot_mode(self):
        '''Set ADC to one-shot mode for minimal quiescent current'''
        reg_value = self._read_register(REG_ADC_CTRL)
        reg_value |= (1 << 1)  # Set ADC_ONE_SHOT bit
        self._write_register(REG_ADC_CTRL, reg_value)
        self.adc_mode = self.ADC_MODE_ONE_SHOT
        logging.info('ADC in one-shot mode')

    def set_adc_continuous_mode(self):
        '''Set the ADC to continuous mode.'''
        reg_value = self._read_register(REG_ADC_CTRL, reg_bits=8)
        reg_value &= ~(1 << 1)  # clear the ADC_ONE_SHOT bit
        reg_value |= (1 << 0)  # set the ADC_EN bit
        self._write_register(REG_ADC_CTRL, reg_value, reg_bits=8)
        self.adc_mode = self.ADC_MODE_CONTINUOUS
        logging.info('ADC in continuous mode')

    def get_adc_mode(self):
        '''
        Get current ADC mode.
    
        Returns:
            str: 'one-shot' or 'continuious'
        '''
        reg_value = self._read_register(REG_ADC_CTRL, reg_bits=8)
        if reg_value & (1 << 1):  # check if ADC_ONE_SHOT bit is set
            return self.ADC_MODE_ONE_SHOT
        else:
            return self.ADC_MODE_CONTINUOUS

    def update_adc_readings(self):
        '''Update all local ADC values.'''
        if self.adc_mode == self.ADC_MODE_ONE_SHOT:
            self.enable_adc()
            time.sleep(0.1)  # allow time for ADC to complete conversions

        self.ibus = self._read_register(REG_ADC_IBUS) * 50 / 1000  # convert to A, 50mA per bit scaling factor
        self.ibat = self._read_register(REG_ADC_IBAT) * 64 / 1000  # convert to A, 64mA per bit scaling factor
        self.vbus = self._read_register(REG_ADC_VBUS) * 16 / 1000  # convert to V, 16mV per bit scaling factor
        self.vpmid = self._read_register(REG_ADC_VPMID) * 16 / 1000  # convert to V, 16mV per bit scaling factor
        self.vbat = self._read_register(REG_ADC_VBAT) * 16 / 1000  # convert to V, 16mV per bit scaling factor
        self.vsys = self._read_register(REG_ADC_VSYS) * 16 / 1000  # convert to V, 16mV per bit scaling factor
        self.ts = self._read_register(REG_ADC_TS) * 0.2  # convert to °C, 0.2°C per bit scaling factor
        self.tdie = self._read_register(REG_ADC_TDIE) * 0.2  # convert to °C, 0.2°C per bit scaling factor

        if self.adc_mode == self.ADC_MODE_ONE_SHOT:
            self.disable_adc()

    
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
        self.charge_voltage = voltage_v
        self.charge_current = current_a

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

    def battery_charging(self): 
        return bool(self.charging_enabled() and self.ibat > 0)

    def battery_percentage(self):
        '''
        Estimate battery percentage based on voltage and battery chemistry.
    
        Returns:
            float: Estimated battery percentage (0-100%)
        '''
        # per-cell voltage
        cell_voltage = self.vbat / self.num_cells
    
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
                return max(0, min(100, percentage))  # clamp to [0, 100]
    
        # if voltage out of range, clamp to 0% or 100%
        if cell_voltage < soc_points[0][0]:
            return 0
        if cell_voltage > soc_points[-1][0]:
            return 100

    def battery_charge_stage(self):
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
        percent = self.battery_percentage()

        # calculate charge time remaining
        if self.battery_charging() and percent < 100:
            remaining_capacity = (1 - (percent / 100)) * self.bat_capacity_ah
            return remaining_capacity / self.charge_current  # hours
        else:
            return 0

    
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
    def update_fault_status(self):
        '''
        Update local fault status.

        Updates the following class variables:
            - input_over_voltage_fault
            - input_under_voltage_fault
            - input_over_current_fault
            - battery_over_voltage_fault
            - battery_over_current_fault
            - temperature_sensor_fault
            - die_over_temperature_fault
            - charging_fault
        '''
        fault_reg = self._read_register(REG_FAULT_STATUS, reg_bits=8)

        self.input_over_voltage_fault = bool(fault_reg & (1 << 7))
        self.input_under_voltage_fault = bool(fault_reg & (1 << 6))
        self.input_over_current_fault = bool(fault_reg & (1 << 5))
        self.battery_over_voltage_fault = bool(fault_reg & (1 << 4))
        self.battery_over_current_fault = bool(fault_reg & (1 << 3))
        self.temperature_sensor_fault = bool(fault_reg & (1 << 2))
        self.die_over_temperature_fault = bool(fault_reg & (1 << 1))
        self.charging_fault = bool(fault_reg & (1 << 0))

    
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
    
