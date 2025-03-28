"""
BQ25798 - Complete I2C Driver for TI's BQ25798 Battery Charger IC

The BQ25798 is a highly integrated switch-mode battery charge management IC that supports
1-4 cell Li-ion and Li-polymer batteries with multiple input sources (USB-PD, solar, etc.).

Features:
- Battery charging configuration
- Multiple input source management
- Solar MPPT (Maximum Power Point Tracking)
- USB-C power delivery support
- Battery health monitoring
- Ship mode and power path management
- Fault detection and handling

Compatible with Raspberry Pi and other Linux-based systems with I2C support.
Requires the smbus2 library.
"""

import time
import atexit
import threading
import logging
from logging.handlers import RotatingFileHandler
from enum import Enum, IntEnum
import math
import smbus2

# Configure logging
logger = logging.getLogger('bq25798')
logger.setLevel(logging.INFO)

# Add rotating file handler (max size: 1MB, keep 3 backups)
file_handler = RotatingFileHandler('bq25798.log', maxBytes=1_000_000, backupCount=3)
file_handler.setLevel(logging.DEBUG)
file_format = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
file_handler.setFormatter(file_format)
logger.addHandler(file_handler)

# Add console handler
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.WARNING)
console_handler.setFormatter(file_format)
logger.addHandler(console_handler)


class BatteryChemistry(str, Enum):
    """Battery chemistry types supported by the driver."""
    LIFEPO4 = 'LiFePO4'
    LI_ION = 'Li-ion'
    NIMH = 'NiMH'
    
    @classmethod
    def list(cls):
        """Return a list of all supported battery chemistries."""
        return [c.value for c in cls]


class ChargeState(str, Enum):
    """Battery charge states from the BQ25798."""
    NOT_CHARGING = 'Not charging'
    TRICKLE_CHARGE = 'Trickle charging'
    PRECHARGE = 'Pre-charging'
    FAST_CHARGE = 'Fast charging (CC)'
    TAPER_CHARGE = 'Taper charging (CV)'
    RESERVED = 'Reserved'
    TOP_OFF = 'Top-off charging'
    CHARGE_DONE = 'Charge complete'


class AdcMode(str, Enum):
    """ADC operating modes for the BQ25798."""
    ONE_SHOT = 'One-shot'
    CONTINUOUS = 'Continuous'


class FaultType(str, Enum):
    """Fault types that can be reported by the BQ25798."""
    IBAT_REGULATION = 'Battery current regulation fault'
    VBUS_OVP = 'VBUS over-voltage protection'
    VBAT_OVP = 'Battery over-voltage protection'
    IBUS_OCP = 'Input current over-current protection'
    IBAT_OCP = 'Battery current over-current protection'
    CONVERTER_OCP = 'Converter over-current protection'
    VAC2_OVP = 'VAC2 over-voltage protection'
    VAC1_OVP = 'VAC1 over-voltage protection'
    VSYS_SHORT = 'System short protection'
    VSYS_OVP = 'System over-voltage protection'
    OTG_OVP = 'OTG over-voltage protection'
    OTG_UVP = 'OTG under-voltage protection'
    TDIE_SHUTDOWN = 'Die temperature shutdown'


class VocMpptPct(IntEnum):
    """MPPT percentages of open circuit voltage for the BQ25798."""
    PCT_56_25 = 0  # 56.25% of VOC
    PCT_62_50 = 1  # 62.50% of VOC
    PCT_68_75 = 2  # 68.75% of VOC
    PCT_75_00 = 3  # 75.00% of VOC
    PCT_81_25 = 4  # 81.25% of VOC
    PCT_87_50 = 5  # 87.50% of VOC (default)
    PCT_93_75 = 6  # 93.75% of VOC
    PCT_100_0 = 7  # 100.0% of VOC


class AdcSampleSpeed(IntEnum):
    """ADC sample speeds for the BQ25798."""
    RES_15_BIT = 0  # 15-bit effective resolution
    RES_14_BIT = 1  # 14-bit effective resolution
    RES_13_BIT = 2  # 13-bit effective resolution
    RES_12_BIT = 3  # 12-bit effective resolution (not recommended)


class Regs:
    """BQ25798 Register definitions."""
    # System config registers
    MIN_SYS_VOLTAGE = 0x00
    CHARGE_VOLTAGE = 0x01
    CHARGE_CURRENT = 0x03
    INPUT_VOLTAGE_LIMIT = 0x05
    INPUT_CURRENT_LIMIT = 0x06
    PRECHARGE_CONTROL = 0x08
    TERMINATION_CONTROL = 0x09
    RECHARGE_CONTROL = 0x0A
    OTG_VOLTAGE = 0x0B
    OTG_CURRENT = 0x0D
    TIMER_CONTROL = 0x0E
    
    # Charger control registers
    CHARGER_CTRL_0 = 0x0F
    CHARGER_CTRL_1 = 0x10
    CHARGER_CTRL_2 = 0x11
    CHARGER_CTRL_3 = 0x12
    CHARGER_CTRL_4 = 0x13
    CHARGER_CTRL_5 = 0x14
    MPPT_CONTROL = 0x15
    TEMPERATURE_CONTROL = 0x16
    NTC_CONTROL_0 = 0x17
    NTC_CONTROL_1 = 0x18
    ICO_CURRENT_LIMIT = 0x19
    
    # Status registers
    CHARGER_STATUS_0 = 0x1B
    CHARGER_STATUS_1 = 0x1C
    CHARGER_STATUS_2 = 0x1D
    CHARGER_STATUS_3 = 0x1E
    CHARGER_STATUS_4 = 0x1F
    FAULT_STATUS_0 = 0x20
    FAULT_STATUS_1 = 0x21
    
    # Flag registers
    CHARGER_FLAG_0 = 0x22
    CHARGER_FLAG_1 = 0x23
    CHARGER_FLAG_2 = 0x24
    CHARGER_FLAG_3 = 0x25
    FAULT_FLAG_0 = 0x26
    FAULT_FLAG_1 = 0x27
    
    # Mask registers
    CHARGER_MASK_0 = 0x28
    CHARGER_MASK_1 = 0x29
    CHARGER_MASK_2 = 0x2A
    CHARGER_MASK_3 = 0x2B
    FAULT_MASK_0 = 0x2C
    FAULT_MASK_1 = 0x2D
    
    # ADC registers
    ADC_CONTROL = 0x2E
    ADC_FUNCTION_DIS_0 = 0x2F
    ADC_FUNCTION_DIS_1 = 0x30
    ADC_IBUS = 0x31
    ADC_IBAT = 0x33
    ADC_VBUS = 0x35
    ADC_VAC1 = 0x37
    ADC_VAC2 = 0x39
    ADC_VBAT = 0x3B
    ADC_VSYS = 0x3D
    ADC_TS = 0x3F
    ADC_TDIE = 0x41
    ADC_DP = 0x43
    ADC_DM = 0x45
    
    # Miscellaneous registers
    DPDM_DRIVER = 0x47
    PART_INFO = 0x48


class BQ25798:
    """Driver for the TI BQ25798 Battery Charger IC."""
    
    # Default I2C address for the BQ25798
    DEFAULT_I2C_ADDRESS = 0x6B
    
    # State-of-charge (SoC) curve reference points
    BATTERY_SOC_CURVES = {
        BatteryChemistry.LIFEPO4: [
            (2.5, 0),    # (voltage_per_cell, percentage)
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
        BatteryChemistry.LI_ION: [
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
        BatteryChemistry.NIMH: [
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

    def __init__(self, 
                 charge_voltage, 
                 charge_current, 
                 battery_cells, 
                 battery_chemistry, 
                 battery_capacity_ah, 
                 i2c_bus=1, 
                 i2c_address=DEFAULT_I2C_ADDRESS,
                 auto_update=True,
                 update_interval=5.0):
        """Initialize the BQ25798 driver.
        
        Args:
            charge_voltage (float): Battery charge voltage in volts (3V-18.8V)
            charge_current (float): Battery charge current in amperes (0.05A-5A)
            battery_cells (int): Number of series battery cells (1-4)
            battery_chemistry (str): Battery chemistry type (see BatteryChemistry enum)
            battery_capacity_ah (float): Battery capacity in amp-hours
            i2c_bus (int): I2C bus number
            i2c_address (int): I2C device address
            auto_update (bool): Whether to automatically update status in background
            update_interval (float): How often to update status in seconds (if auto_update is True)
        
        Raises:
            ValueError: If parameters are invalid
            IOError: If communication with the device fails
        """
        # Validate parameters
        if not 3.0 <= charge_voltage <= 18.8:
            raise ValueError(f"Charge voltage must be between 3.0V and 18.8V, got {charge_voltage}V")
        
        if not 0.05 <= charge_current <= 5.0:
            raise ValueError(f"Charge current must be between 0.05A and 5.0A, got {charge_current}A")
        
        if not 1 <= battery_cells <= 4:
            raise ValueError(f"Battery cells must be between 1 and 4, got {battery_cells}")
        
        if battery_chemistry not in BatteryChemistry.list():
            raise ValueError(f"Unsupported battery chemistry: {battery_chemistry}. "
                            f"Supported types: {BatteryChemistry.list()}")
        
        if battery_capacity_ah <= 0:
            raise ValueError(f"Battery capacity must be positive, got {battery_capacity_ah}Ah")
        
        self.i2c_bus = i2c_bus
        self.i2c_address = i2c_address
        self.bus = smbus2.SMBus(i2c_bus)
        self.update_interval = update_interval
        
        # Battery configuration
        self.target_charge_voltage = charge_voltage
        self.target_charge_current = charge_current
        self.battery_cells = battery_cells
        self.battery_chemistry = battery_chemistry
        self.battery_capacity_ah = battery_capacity_ah
        
        # Register cleanup at exit
        atexit.register(self.close)
        
        # Status variables
        self._charge_state = ChargeState.NOT_CHARGING
        self._adc_mode = AdcMode.ONE_SHOT
        self._auto_update = False
        self._update_thread = None
        self._stop_event = threading.Event()
        
        # ADC readings
        self.vbus = 0.0      # Input voltage (V)
        self.ibus = 0.0      # Input current (A)
        self.vbat = 0.0      # Battery voltage (V)
        self.ibat = 0.0      # Battery current (A)
        self.vsys = 0.0      # System voltage (V)
        self.tdie = 0.0      # Die temperature (°C)
        self.vac1 = 0.0      # Input 1 voltage (V)
        self.vac2 = 0.0      # Input 2 voltage (V)
        self.ts = 0.0        # Temperature sensor reading (%)
        self.dp = 0.0        # USB D+ voltage (V)
        self.dm = 0.0        # USB D- voltage (V)
        
        # Fault status
        self.faults = {fault: False for fault in FaultType}
        
        # Callback functions
        self.state_change_callback = None
        self.fault_change_callback = None
        
        # Thread locks
        self._state_lock = threading.Lock()
        self._adc_lock = threading.Lock()
        self._fault_lock = threading.Lock()
        self._i2c_lock = threading.Lock()
        
        # Verify communication with the device
        try:
            device_id = self._read_register(Regs.PART_INFO, 8)
            # Extract part number from bits 5-3
            part_number = (device_id >> 3) & 0x07
            # Extract device revision from bits 2-0
            device_rev = device_id & 0x07
            
            if part_number != 3:  # 3 = BQ25798
                logger.warning(f"Unexpected part number: {part_number}, expected 3 (BQ25798)")
            
            logger.info(f"Connected to BQ25798 (rev {device_rev}) at address 0x{i2c_address:02X}")
        except Exception as e:
            logger.error(f"Failed to communicate with BQ25798: {e}")
            raise IOError(f"Communication with BQ25798 failed: {e}")
        
        # Initialize the device
        self._initialize_device()
        
        # Update status immediately
        self.update_all()
        
        # Start automatic updates if requested
        if auto_update:
            self.start_auto_updates()

    def _initialize_device(self):
        """Initialize the BQ25798 with default settings."""
        logger.info("Initializing BQ25798...")
        
        # Ensure ship mode is disabled
        if self.is_ship_mode_enabled():
            self.disable_ship_mode()
            logger.info("Disabled ship mode")
        
        # Set charge voltage and current
        self.set_charge_voltage(self.target_charge_voltage)
        self.set_charge_current(self.target_charge_current)
        
        # Set safety timers
        self.set_precharge_timeout(30)  # 30 minutes
        self.set_fast_charge_timeout(180)  # 3 hours
        
        # Configure the ADC
        self.set_adc_mode(AdcMode.ONE_SHOT)
        self.enable_battery_current_sense()
        
        # Disable MPPT by default (can be enabled later if needed)
        self.disable_mppt()
        
        # Set default USB OTG parameters
        self.set_otg_voltage(5.0)  # 5V
        self.set_otg_current(1.0)  # 1A
        
        logger.info("BQ25798 initialization complete")

    def close(self):
        """Close the I2C connection and stop background threads."""
        if self._auto_update:
            self.stop_auto_updates()
        
        if hasattr(self, 'bus') and self.bus:
            try:
                self.bus.close()
                logger.info("I2C bus closed")
            except Exception as e:
                logger.error(f"Error closing I2C bus: {e}")

    #######################
    # I2C Helper Methods  #
    #######################
    
    def _read_register(self, reg, length=8, start_bit=None, end_bit=None):
        """Read from a register.
        
        Args:
            reg (int): Register address
            length (int): Register length in bits (8 or 16)
            start_bit (int, optional): Start bit position
            end_bit (int, optional): End bit position
            
        Returns:
            int: Register value
            
        Raises:
            ValueError: If length is not 8 or 16
            IOError: If communication fails
        """
        if length not in (8, 16):
            raise ValueError(f"Register length must be 8 or 16, got {length}")
            
        with self._i2c_lock:
            try:
                if length == 8:
                    value = self.bus.read_byte_data(self.i2c_address, reg)
                else:  # length == 16
                    value = self.bus.read_word_data(self.i2c_address, reg)
                    
                # Extract bits if requested
                if start_bit is not None:
                    if end_bit is None:
                        end_bit = start_bit
                    
                    # Create mask for the specified bits
                    mask = ((1 << (end_bit - start_bit + 1)) - 1) << start_bit
                    value = (value & mask) >> start_bit
                
                return value
            except Exception as e:
                logger.error(f"Failed to read register 0x{reg:02X}: {e}")
                raise IOError(f"I2C read failed: {e}")

    def _write_register(self, reg, value, length=8, start_bit=None, end_bit=None):
        """Write to a register.
        
        Args:
            reg (int): Register address
            value (int): Value to write
            length (int): Register length in bits (8 or 16)
            start_bit (int, optional): Start bit position
            end_bit (int, optional): End bit position
            
        Raises:
            ValueError: If length is not 8 or 16
            IOError: If communication fails
        """
        if length not in (8, 16):
            raise ValueError(f"Register length must be 8 or 16, got {length}")
            
        with self._i2c_lock:
            try:
                # If only setting specific bits, read-modify-write
                if start_bit is not None:
                    if end_bit is None:
                        end_bit = start_bit
                    
                    # Read current value
                    current_value = self._read_register(reg, length)
                    
                    # Create mask for the specified bits
                    mask = ((1 << (end_bit - start_bit + 1)) - 1) << start_bit
                    
                    # Clear the bits to be modified and set the new value
                    new_value = (current_value & ~mask) | ((value << start_bit) & mask)
                    value = new_value
                
                # Write the value
                if length == 8:
                    self.bus.write_byte_data(self.i2c_address, reg, value)
                else:  # length == 16
                    self.bus.write_word_data(self.i2c_address, reg, value)
            except Exception as e:
                logger.error(f"Failed to write register 0x{reg:02X}: {e}")
                raise IOError(f"I2C write failed: {e}")

    #######################
    # Status Updates      #
    #######################
    
    def update_all(self):
        """Update all status information."""
        self.update_adc_readings()
        self.update_charge_state()
        self.update_fault_status()

    def start_auto_updates(self):
        """Start automatic status updates in background thread."""
        if self._auto_update:
            return
            
        self._auto_update = True
        self._stop_event.clear()
        self._update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()
        logger.info(f"Started automatic updates (interval: {self.update_interval}s)")

    def stop_auto_updates(self):
        """Stop automatic status updates."""
        if not self._auto_update:
            return
            
        self._auto_update = False
        self._stop_event.set()
        if self._update_thread and self._update_thread.is_alive():
            self._update_thread.join(timeout=1.0)
        logger.info("Stopped automatic updates")

    def _update_loop(self):
        """Background thread for automatic updates."""
        while not self._stop_event.is_set():
            try:
                # Update all status information
                old_state = self._charge_state
                old_faults = self.faults.copy()
                
                self.update_all()
                
                # Check for state changes
                if old_state != self._charge_state and self.state_change_callback:
                    self.state_change_callback(self._charge_state)
                
                # Check for fault changes
                for fault, status in self.faults.items():
                    if status != old_faults[fault] and self.fault_change_callback:
                        self.fault_change_callback(fault, status)
                
            except Exception as e:
                logger.error(f"Error in update loop: {e}")
                
            # Wait for the next update interval or until stop is requested
            self._stop_event.wait(self.update_interval)

    def update_adc_readings(self):
        """Update all ADC readings."""
        with self._adc_lock:
            # Enable ADC for one-shot mode
            one_shot_mode = (self._adc_mode == AdcMode.ONE_SHOT)
            if one_shot_mode:
                self.enable_adc()
                # Allow time for readings to stabilize
                time.sleep(0.1)
            
            try:
                # Read ADC registers
                self.vbus = self._read_register(Regs.ADC_VBUS, 16) / 1000.0  # mV to V
                self.vbat = self._read_register(Regs.ADC_VBAT, 16) / 1000.0  # mV to V
                self.vsys = self._read_register(Regs.ADC_VSYS, 16) / 1000.0  # mV to V
                self.vac1 = self._read_register(Regs.ADC_VAC1, 16) / 1000.0  # mV to V
                self.vac2 = self._read_register(Regs.ADC_VAC2, 16) / 1000.0  # mV to V
                self.ts = self._read_register(Regs.ADC_TS, 16) * 0.0976563  # 0.0976563% per bit
                self.dp = self._read_register(Regs.ADC_DP, 16) / 1000.0     # mV to V
                self.dm = self._read_register(Regs.ADC_DM, 16) / 1000.0     # mV to V
                
                # Die temperature (2's complement)
                tdie_raw = self._read_register(Regs.ADC_TDIE, 16)
                if tdie_raw & 0x8000:  # Negative temperature
                    self.tdie = -((~tdie_raw & 0xFFFF) + 1) * 0.5  # 0.5°C per bit
                else:
                    self.tdie = tdie_raw * 0.5  # 0.5°C per bit
                
                # Current readings (2's complement)
                ibus_raw = self._read_register(Regs.ADC_IBUS, 16)
                if ibus_raw & 0x8000:  # Negative current
                    self.ibus = -((~ibus_raw & 0xFFFF) + 1) / 1000.0  # mA to A
                else:
                    self.ibus = ibus_raw / 1000.0  # mA to A
                
                ibat_raw = self._read_register(Regs.ADC_IBAT, 16)
                if ibat_raw & 0x8000:  # Negative current
                    self.ibat = -((~ibat_raw & 0xFFFF) + 1) / 1000.0  # mA to A
                else:
                    self.ibat = ibat_raw / 1000.0  # mA to A
            
            finally:
                # Disable ADC if we were in one-shot mode
                if one_shot_mode:
                    self.disable_adc()
            
            logger.debug(f"ADC: VBUS={self.vbus:.2f}V, VBAT={self.vbat:.2f}V, "
                       f"IBUS={self.ibus:.3f}A, IBAT={self.ibat:.3f}A, "
                       f"TDIE={self.tdie:.1f}°C")

    def update_charge_state(self):
        """Update charge state information."""
        with self._state_lock:
            # Read charge state from register
            charge_status = self._read_register(Regs.CHARGER_STATUS_1, 8, start_bit=5, end_bit=7)
            
            # Map register value to charge state
            state_map = {
                0: ChargeState.NOT_CHARGING,
                1: ChargeState.TRICKLE_CHARGE,
                2: ChargeState.PRECHARGE,
                3: ChargeState.FAST_CHARGE,
                4: ChargeState.TAPER_CHARGE,
                5: ChargeState.RESERVED,
                6: ChargeState.TOP_OFF,
                7: ChargeState.CHARGE_DONE
            }
            
            new_state = state_map.get(charge_status, ChargeState.NOT_CHARGING)
            
            # Detect state changes
            if new_state != self._charge_state:
                logger.info(f"Charge state changed: {self._charge_state} -> {new_state}")
                self._charge_state = new_state
                
                # Call state change callback if registered
                if self.state_change_callback:
                    self.state_change_callback(new_state)

    def update_fault_status(self):
        """Update fault status information."""
        with self._fault_lock:
            # Read fault status registers
            fault_reg_0 = self._read_register(Regs.FAULT_STATUS_0, 8)
            fault_reg_1 = self._read_register(Regs.FAULT_STATUS_1, 8)
            
            # Update fault status
            old_faults = self.faults.copy()
            
            self.faults[FaultType.VSYS_SHORT] = bool(fault_reg_1 & (1 << 7))
            self.faults[FaultType.VSYS_OVP] = bool(fault_reg_1 & (1 << 6))
            self.faults[FaultType.OTG_OVP] = bool(fault_reg_1 & (1 << 5))
            self.faults[FaultType.OTG_UVP] = bool(fault_reg_1 & (1 << 4))
            self.faults[FaultType.TDIE_SHUTDOWN] = bool(fault_reg_1 & (1 << 2))
            
            # Log and notify on fault changes
            for fault, status in self.faults.items():
                if status != old_faults[fault]:
                    if status:
                        logger.warning(f"Fault detected: {fault}")
                    else:
                        logger.info(f"Fault cleared: {fault}")
                    
                    # Call fault change callback if registered
                    if self.fault_change_callback:
                        self.fault_change_callback(fault, status)

    def get_active_faults(self):
        """Get a list of active faults.
        
        Returns:
            list: Active fault types
        """
        return [fault for fault, status in self.faults.items() if status]

    #######################
    # Battery Information #
    #######################
    
    def get_charge_state(self):
        """Get the current charge state.
        
        Returns:
            ChargeState: Current charge state
        """
        return self._charge_state
        
    def is_battery_charging(self):
        """Check if the battery is currently charging.
        
        Returns:
            bool: True if battery is charging, False otherwise
        """
        charging_states = [
            ChargeState.TRICKLE_CHARGE,
            ChargeState.PRECHARGE,
            ChargeState.FAST_CHARGE,
            ChargeState.TAPER_CHARGE,
            ChargeState.TOP_OFF
        ]
        return self._charge_state in charging_states

    def estimate_battery_percentage(self):
        """Estimate battery state of charge (percentage).
        
        Note: This is an approximation based on the battery voltage and a reference curve.
        For more accurate SoC estimation, additional techniques like coulomb counting
        should be used.
        
        Returns:
            float: Estimated battery percentage (0-100)
        """
        if self.vbat <= 0:
            self.update_adc_readings()
            
        if self.vbat <= 0:
            logger.warning("Cannot estimate battery percentage: battery voltage reading is zero")
            return 0
            
        # Get voltage per cell
        voltage_per_cell = self.vbat / self.battery_cells
        
        # Get reference curve for this battery chemistry
        curve = self.BATTERY_SOC_CURVES.get(self.battery_chemistry)
        if not curve:
            logger.warning(f"Unknown battery chemistry: {self.battery_chemistry}")
            return 0
        
        # Find the closest reference points
        if voltage_per_cell <= curve[0][0]:
            return curve[0][1]  # Below minimum voltage
        
        if voltage_per_cell >= curve[-1][0]:
            return curve[-1][1]  # Above maximum voltage
        
        # Linear interpolation between reference points
        for i in range(len(curve) - 1):
            v1, p1 = curve[i]
            v2, p2 = curve[i + 1]
            
            if v1 <= voltage_per_cell <= v2:
                # Linear interpolation
                return p1 + (p2 - p1) * (voltage_per_cell - v1) / (v2 - v1)
        
        # Shouldn't reach here
        return 0

    def get_charging_metrics(self):
        """Get comprehensive charging metrics.
        
        Returns:
            dict: Charging metrics
        """
        self.update_all()
        
        # Calculate power values
        input_power = self.vbus * self.ibus
        battery_power = self.vbat * self.ibat
        
        # Estimate efficiency when charging
        efficiency = 0.0
        if input_power > 0 and battery_power > 0:
            efficiency = (battery_power / input_power) * 100.0
            
        # Estimate time to full
        time_to_full = float('inf')
        if self.is_battery_charging() and self.ibat > 0:
            # Very rough estimate based on battery capacity and current charge rate
            soc = self.estimate_battery_percentage()
            remaining_capacity = self.battery_capacity_ah * (100 - soc) / 100
            time_to_full = remaining_capacity / self.ibat  # hours
        
        return {
            'state': self._charge_state,
            'voltage': self.vbat,
            'current': self.ibat,
            'soc': self.estimate_battery_percentage(),
            'input_power': input_power,
            'battery_power': battery_power,
            'efficiency': efficiency,
            'time_to_full': time_to_full,
            'temperature': self.tdie,
            'faults': self.get_active_faults()
        }

    #######################
    # ADC Configuration   #
    #######################
    
    def enable_adc(self):
        """Enable the ADC.
        
        Returns:
            bool: True if successful
        """
        try:
            self._write_register(Regs.ADC_CONTROL, 1, start_bit=7)
            return True
        except Exception as e:
            logger.error(f"Failed to enable ADC: {e}")
            return False

    def disable_adc(self):
        """Disable the ADC.
        
        Returns:
            bool: True if successful
        """
        try:
            self._write_register(Regs.ADC_CONTROL, 0, start_bit=7)
            return True
        except Exception as e:
            logger.error(f"Failed to disable ADC: {e}")
            return False

    def is_adc_enabled(self):
        """Check if the ADC is enabled.
        
        Returns:
            bool: True if ADC is enabled
        """
        return bool(self._read_register(Regs.ADC_CONTROL, 8, start_bit=7))

    def set_adc_mode(self, mode):
        """Set the ADC mode.
        
        Args:
            mode (AdcMode): ADC mode (ONE_SHOT or CONTINUOUS)
            
        Returns:
            bool: True if successful
        """
        if mode not in (AdcMode.ONE_SHOT, AdcMode.CONTINUOUS):
            logger.error(f"Invalid ADC mode: {mode}")
            return False
            
        try:
            if mode == AdcMode.ONE_SHOT:
                # One-shot mode: bit 6 = 1
                self._write_register(Regs.ADC_CONTROL, 1, start_bit=6)
            else:  # CONTINUOUS
                # Continuous mode: bit 6 = 0
                self._write_register(Regs.ADC_CONTROL, 0, start_bit=6)
                
            self._adc_mode = mode
            logger.info(f"ADC mode set to {mode}")
            return True
        except Exception as e:
            logger.error(f"Failed to set ADC mode: {e}")
            return False

    def set_adc_sample_speed(self, speed):
        """Set the ADC sample speed.
        
        Args:
            speed (AdcSampleSpeed): ADC sample speed
            
        Returns:
            bool: True if successful
        """
        try:
            # Write to bits 4-5 of ADC control register
            self._write_register(Regs.ADC_CONTROL, speed.value, start_bit=4, end_bit=5)
            logger.info(f"ADC sample speed set to {speed.name}")
            return True
        except Exception as e:
            logger.error(f"Failed to set ADC sample speed: {e}")
            return False

    def enable_battery_current_sense(self):
        """Enable battery current sensing (discharge current).
        
        Returns:
            bool: True if successful
        """
        try:
            # Set bit 5 of charge control register 5
            self._write_register(Regs.CHARGER_CTRL_5, 1, start_bit=5)
            logger.info("Battery current sensing enabled")
            return True
        except Exception as e:
            logger.error(f"Failed to enable battery current sensing: {e}")
            return False

    def disable_battery_current_sense(self):
        """Disable battery current sensing (discharge current).
        
        Returns:
            bool: True if successful
        """
        try:
            # Clear bit 5 of charge control register 5
            self._write_register(Regs.CHARGER_CTRL_5, 0, start_bit=5)
            logger.info("Battery current sensing disabled")
            return True
        except Exception as e:
            logger.error(f"Failed to disable battery current sensing: {e}")
            return False

    #######################
    # Charging Control    #
    #######################
    
    def set_charge_voltage(self, voltage):
        """Set the battery charge voltage.
        
        Args:
            voltage (float): Charge voltage in volts (3.0-18.8V)
            
        Returns:
            bool: True if successful
            
        Raises:
            ValueError: If voltage is out of range
        """
        if not 3.0 <= voltage <= 18.8:
            raise ValueError(f"Charge voltage must be between 3.0V and 18.8V, got {voltage}V")
            
        try:
            # Convert to register value: 10mV per bit
            reg_value = int(voltage * 100)
            self._write_register(Regs.CHARGE_VOLTAGE, reg_value, length=16)
            self.target_charge_voltage = voltage
            logger.info(f"Charge voltage set to {voltage:.2f}V")
            return True
        except Exception as e:
            logger.error(f"Failed to set charge voltage: {e}")
            return False

    def get_charge_voltage(self):
        """Get the battery charge voltage setting.
        
        Returns:
            float: Charge voltage in volts
        """
        reg_value = self._read_register(Regs.CHARGE_VOLTAGE, length=16)
        return reg_value / 100.0  # 10mV per bit

    def set_charge_current(self, current):
        """Set the battery charge current.
        
        Args:
            current (float): Charge current in amperes (0.05-5.0A)
            
        Returns:
            bool: True if successful
            
        Raises:
            ValueError: If current is out of range
        """
        if not 0.05 <= current <= 5.0:
            raise ValueError(f"Charge current must be between 0.05A and 5.0A, got {current}A")
            
        try:
            # Convert to register value: 10mA per bit
            reg_value = int(current * 100)
            self._write_register(Regs.CHARGE_CURRENT, reg_value, length=16)
            self.target_charge_current = current
            logger.info(f"Charge current set to {current:.2f}A")
            return True
        except Exception as e:
            logger.error(f"Failed to set charge current: {e}")
            return False

    def get_charge_current(self):
        """Get the battery charge current setting.
        
        Returns:
            float: Charge current in amperes
        """
        reg_value = self._read_register(Regs.CHARGE_CURRENT, length=16)
        return reg_value / 100.0  # 10mA per bit

    def enable_charging(self):
        """Enable battery charging.
        
        Returns:
            bool: True if successful
        """
        try:
            # Set bit 5 of charge control register 0
            self._write_register(Regs.CHARGER_CTRL_0, 1, start_bit=5)
            logger.info("Battery charging enabled")
            return True
        except Exception as e:
            logger.error(f"Failed to enable charging: {e}")
            return False

    def disable_charging(self):
        """Disable battery charging.
        
        Returns:
            bool: True if successful
        """
        try:
            # Clear bit 5 of charge control register 0
            self._write_register(Regs.CHARGER_CTRL_0, 0, start_bit=5)
            logger.info("Battery charging disabled")
            return True
        except Exception as e:
            logger.error(f"Failed to disable charging: {e}")
            return False

    def is_charging_enabled(self):
        """Check if battery charging is enabled.
        
        Returns:
            bool: True if charging is enabled
        """
        return bool(self._read_register(Regs.CHARGER_CTRL_0, 8, start_bit=5))

    def set_input_current_limit(self, current):
        """Set the input current limit.
        
        Args:
            current (float): Input current limit in amperes (0.1-3.3A)
            
        Returns:
            bool: True if successful
            
        Raises:
            ValueError: If current is out of range
        """
        if not 0.1 <= current <= 3.3:
            raise ValueError(f"Input current must be between 0.1A and 3.3A, got {current}A")
            
        try:
            # Convert to register value: 10mA per bit
            reg_value = int(current * 100)
            self._write_register(Regs.INPUT_CURRENT_LIMIT, reg_value, length=16)
            logger.info(f"Input current limit set to {current:.2f}A")
            return True
        except Exception as e:
            logger.error(f"Failed to set input current limit: {e}")
            return False

    def get_input_current_limit(self):
        """Get the input current limit setting.
        
        Returns:
            float: Input current limit in amperes
        """
        reg_value = self._read_register(Regs.INPUT_CURRENT_LIMIT, length=16)
        return reg_value / 100.0  # 10mA per bit

    def set_input_voltage_limit(self, voltage):
        """Set the input voltage limit (VINDPM).
        
        Args:
            voltage (float): Input voltage limit in volts (3.6-22V)
            
        Returns:
            bool: True if successful
            
        Raises:
            ValueError: If voltage is out of range
        """
        if not 3.6 <= voltage <= 22.0:
            raise ValueError(f"Input voltage must be between 3.6V and 22.0V, got {voltage}V")
            
        try:
            # Convert to register value: 100mV per bit
            reg_value = int(voltage * 10)
            self._write_register(Regs.INPUT_VOLTAGE_LIMIT, reg_value, length=8)
            logger.info(f"Input voltage limit set to {voltage:.2f}V")
            return True
        except Exception as e:
            logger.error(f"Failed to set input voltage limit: {e}")
            return False

    def get_input_voltage_limit(self):
        """Get the input voltage limit (VINDPM) setting.
        
        Returns:
            float: Input voltage limit in volts
        """
        reg_value = self._read_register(Regs.INPUT_VOLTAGE_LIMIT, length=8)
        return reg_value / 10.0  # 100mV per bit

    def set_precharge_current(self, current):
        """Set the precharge current limit.
        
        Args:
            current (float): Precharge current in amperes (0.04-2.0A)
            
        Returns:
            bool: True if successful
            
        Raises:
            ValueError: If current is out of range
        """
        if not 0.04 <= current <= 2.0:
            raise ValueError(f"Precharge current must be between 0.04A and 2.0A, got {current}A")
            
        try:
            # Convert to register value: 40mA per bit
            reg_value = int(current * 1000 / 40)
            # Write to bits 0-5 of precharge control register
            self._write_register(Regs.PRECHARGE_CONTROL, reg_value, start_bit=0, end_bit=5)
            logger.info(f"Precharge current set to {current:.2f}A")
            return True
        except Exception as e:
            logger.error(f"Failed to set precharge current: {e}")
            return False

    def get_precharge_current(self):
        """Get the precharge current limit setting.
        
        Returns:
            float: Precharge current in amperes
        """
        reg_value = self._read_register(Regs.PRECHARGE_CONTROL, 8, start_bit=0, end_bit=5)
        return reg_value * 0.04  # 40mA per bit

    def set_termination_current(self, current):
        """Set the charge termination current.
        
        Args:
            current (float): Termination current in amperes (0.04-1.0A)
            
        Returns:
            bool: True if successful
            
        Raises:
            ValueError: If current is out of range
        """
        if not 0.04 <= current <= 1.0:
            raise ValueError(f"Termination current must be between 0.04A and 1.0A, got {current}A")
            
        try:
            # Convert to register value: 40mA per bit
            reg_value = int(current * 1000 / 40)
            # Write to bits 0-4 of termination control register
            self._write_register(Regs.TERMINATION_CONTROL, reg_value, start_bit=0, end_bit=4)
            logger.info(f"Termination current set to {current:.2f}A")
            return True
        except Exception as e:
            logger.error(f"Failed to set termination current: {e}")
            return False

    def get_termination_current(self):
        """Get the charge termination current setting.
        
        Returns:
            float: Termination current in amperes
        """
        reg_value = self._read_register(Regs.TERMINATION_CONTROL, 8, start_bit=0, end_bit=4)
        return reg_value * 0.04  # 40mA per bit

    def enable_termination(self):
        """Enable charge termination.
        
        Returns:
            bool: True if successful
        """
        try:
            # Set bit 1 of charge control register 0
            self._write_register(Regs.CHARGER_CTRL_0, 1, start_bit=1)
            logger.info("Charge termination enabled")
            return True
        except Exception as e:
            logger.error(f"Failed to enable termination: {e}")
            return False

    def disable_termination(self):
        """Disable charge termination.
        
        Returns:
            bool: True if successful
        """
        try:
            # Clear bit 1 of charge control register 0
            self._write_register(Regs.CHARGER_CTRL_0, 0, start_bit=1)
            logger.info("Charge termination disabled")
            return True
        except Exception as e:
            logger.error(f"Failed to disable termination: {e}")
            return False

    def is_termination_enabled(self):
        """Check if charge termination is enabled.
        
        Returns:
            bool: True if termination is enabled
        """
        return bool(self._read_register(Regs.CHARGER_CTRL_0, 8, start_bit=1))

    def set_minimum_system_voltage(self, voltage):
        """Set the minimum system voltage.
        
        Args:
            voltage (float): Minimum system voltage in volts (2.5-16.0V)
            
        Returns:
            bool: True if successful
            
        Raises:
            ValueError: If voltage is out of range
        """
        if not 2.5 <= voltage <= 16.0:
            raise ValueError(f"Minimum system voltage must be between 2.5V and 16.0V, got {voltage}V")
            
        try:
            # Convert to register value: 250mV per bit, 2.5V offset
            reg_value = int((voltage - 2.5) / 0.25)
            # Write to bits 0-5 of minimum system voltage register
            self._write_register(Regs.MIN_SYS_VOLTAGE, reg_value, start_bit=0, end_bit=5)
            logger.info(f"Minimum system voltage set to {voltage:.2f}V")
            return True
        except Exception as e:
            logger.error(f"Failed to set minimum system voltage: {e}")
            return False

    def get_minimum_system_voltage(self):
        """Get the minimum system voltage setting.
        
        Returns:
            float: Minimum system voltage in volts
        """
        reg_value = self._read_register(Regs.MIN_SYS_VOLTAGE, 8, start_bit=0, end_bit=5)
        return 2.5 + (reg_value * 0.25)  # 2.5V offset, 250mV per bit

    def set_recharge_threshold(self, threshold):
        """Set the battery recharge threshold.
        
        Args:
            threshold (float): Recharge threshold in volts (0.05-0.8V)
            
        Returns:
            bool: True if successful
            
        Raises:
            ValueError: If threshold is out of range
        """
        if not 0.05 <= threshold <= 0.8:
            raise ValueError(f"Recharge threshold must be between 0.05V and 0.8V, got {threshold}V")
            
        try:
            # Convert to register value: 50mV per bit
            reg_value = int(threshold / 0.05) - 1  # 0-based indexing
            # Write to bits 0-3 of recharge control register
            self._write_register(Regs.RECHARGE_CONTROL, reg_value, start_bit=0, end_bit=3)
            logger.info(f"Recharge threshold set to {threshold:.2f}V")
            return True
        except Exception as e:
            logger.error(f"Failed to set recharge threshold: {e}")
            return False

    def get_recharge_threshold(self):
        """Get the battery recharge threshold setting.
        
        Returns:
            float: Recharge threshold in volts
        """
        reg_value = self._read_register(Regs.RECHARGE_CONTROL, 8, start_bit=0, end_bit=3)
        return (reg_value + 1) * 0.05  # 50mV per bit, 1-based value

    #######################
    # Timer Configuration #
    #######################
    
    def set_precharge_timeout(self, minutes):
        """Set the precharge timeout.
        
        Args:
            minutes (int): Timeout in minutes (30 or 120)
            
        Returns:
            bool: True if successful
            
        Raises:
            ValueError: If minutes is not 30 or 120
        """
        if minutes != 30 and minutes != 120:
            raise ValueError(f"Precharge timeout must be 30 or 120 minutes, got {minutes}")
            
        try:
            # 0 = 120 minutes (2 hours), 1 = 30 minutes (0.5 hours)
            reg_value = 1 if minutes == 30 else 0
            # Write to bit 7 of OTG current register
            self._write_register(Regs.OTG_CURRENT, reg_value, start_bit=7)
            logger.info(f"Precharge timeout set to {minutes} minutes")
            return True
        except Exception as e:
            logger.error(f"Failed to set precharge timeout: {e}")
            return False

    def get_precharge_timeout(self):
        """Get the precharge timeout setting.
        
        Returns:
            int: Timeout in minutes
        """
        reg_value = self._read_register(Regs.OTG_CURRENT, 8, start_bit=7)
        return 30 if reg_value == 1 else 120

    def set_fast_charge_timeout(self, minutes):
        """Set the fast charge timeout.
        
        Args:
            minutes (int): Timeout in minutes (300, 480, 720, or 1440)
            
        Returns:
            bool: True if successful
            
        Raises:
            ValueError: If minutes is not one of the allowed values
        """
        allowed_values = {300: 0, 480: 1, 720: 2, 1440: 3}  # minutes: register value
        if minutes not in allowed_values:
            raise ValueError(f"Fast charge timeout must be one of {list(allowed_values.keys())}, got {minutes}")
            
        try:
            # Write to bits 1-2 of timer control register
            self._write_register(Regs.TIMER_CONTROL, allowed_values[minutes], start_bit=1, end_bit=2)
            logger.info(f"Fast charge timeout set to {minutes} minutes")
            return True
        except Exception as e:
            logger.error(f"Failed to set fast charge timeout: {e}")
            return False

    def get_fast_charge_timeout(self):
        """Get the fast charge timeout setting.
        
        Returns:
            int: Timeout in minutes
        """
        reg_value = self._read_register(Regs.TIMER_CONTROL, 8, start_bit=1, end_bit=2)
        timeout_map = {0: 300, 1: 480, 2: 720, 3: 1440}
        return timeout_map.get(reg_value, 300)

    def enable_watchdog(self, seconds=40):
        """Enable the I2C watchdog timer.
        
        Args:
            seconds (int): Watchdog timeout in seconds (0.5, 1, 2, 20, 40, 80, or 160)
            
        Returns:
            bool: True if successful
            
        Raises:
            ValueError: If seconds is not one of the allowed values
        """
        allowed_values = {0.5: 1, 1: 2, 2: 3, 20: 4, 40: 5, 80: 6, 160: 7}  # seconds: register value
        if seconds not in allowed_values:
            raise ValueError(f"Watchdog timeout must be one of {list(allowed_values.keys())}, got {seconds}")
            
        try:
            # Write to bits 0-2 of charge control register 1
            self._write_register(Regs.CHARGER_CTRL_1, allowed_values[seconds], start_bit=0, end_bit=2)
            logger.info(f"Watchdog timer enabled with {seconds}s timeout")
            return True
        except Exception as e:
            logger.error(f"Failed to enable watchdog: {e}")
            return False

    def disable_watchdog(self):
        """Disable the I2C watchdog timer.
        
        Returns:
            bool: True if successful
        """
        try:
            # Write 0 to bits 0-2 of charge control register 1
            self._write_register(Regs.CHARGER_CTRL_1, 0, start_bit=0, end_bit=2)
            logger.info("Watchdog timer disabled")
            return True
        except Exception as e:
            logger.error(f"Failed to disable watchdog: {e}")
            return False

    def reset_watchdog(self):
        """Reset the I2C watchdog timer.
        
        Returns:
            bool: True if successful
        """
        try:
            # Set bit 3 of charge control register 1
            self._write_register(Regs.CHARGER_CTRL_1, 1, start_bit=3)
            logger.debug("Watchdog timer reset")
            return True
        except Exception as e:
            logger.error(f"Failed to reset watchdog: {e}")
            return False

    #######################
    # USB-C PD (OTG) Mode #
    #######################
    
    def set_otg_voltage(self, voltage):
        """Set the USB-C sourcing (OTG) voltage.
        
        Args:
            voltage (float): OTG output voltage in volts (2.8-22.0V)
            
        Returns:
            bool: True if successful
            
        Raises:
            ValueError: If voltage is out of range
        """
        if not 2.8 <= voltage <= 22.0:
            raise ValueError(f"OTG voltage must be between 2.8V and 22.0V, got {voltage}V")
            
        try:
            # Convert to register value: 10mV per bit, 2.8V offset
            reg_value = int((voltage - 2.8) * 100)
            self._write_register(Regs.OTG_VOLTAGE, reg_value, length=16)
            logger.info(f"OTG voltage set to {voltage:.2f}V")
            return True
        except Exception as e:
            logger.error(f"Failed to set OTG voltage: {e}")
            return False

    def get_otg_voltage(self):
        """Get the USB-C sourcing (OTG) voltage setting.
        
        Returns:
            float: OTG output voltage in volts
        """
        reg_value = self._read_register(Regs.OTG_VOLTAGE, length=16)
        return 2.8 + (reg_value / 100.0)  # 2.8V offset, 10mV per bit

    def set_otg_current(self, current):
        """Set the USB-C sourcing (OTG) current limit.
        
        Args:
            current (float): OTG output current in amperes (0.16-3.36A)
            
        Returns:
            bool: True if successful
            
        Raises:
            ValueError: If current is out of range
        """
        if not 0.16 <= current <= 3.36:
            raise ValueError(f"OTG current must be between 0.16A and 3.36A, got {current}A")
            
        try:
            # Convert to register value: 40mA per bit
            reg_value = int(current * 1000 / 40)
            # Write to bits 0-6 of OTG current register
            self._write_register(Regs.OTG_CURRENT, reg_value, start_bit=0, end_bit=6)
            logger.info(f"OTG current set to {current:.2f}A")
            return True
        except Exception as e:
            logger.error(f"Failed to set OTG current: {e}")
            return False

    def get_otg_current(self):
        """Get the USB-C sourcing (OTG) current limit setting.
        
        Returns:
            float: OTG output current in amperes
        """
        reg_value = self._read_register(Regs.OTG_CURRENT, 8, start_bit=0, end_bit=6)
        return reg_value * 0.04  # 40mA per bitFaultType.IBAT_REGULATION] = bool(fault_reg_0 & (1 << 7))
