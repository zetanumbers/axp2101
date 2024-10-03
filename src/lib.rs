use bitfield_struct::bitfield;
use embedded_hal::i2c::I2c;

#[derive(Debug, Copy, Clone)]
#[repr(u8)]
pub enum Register {
    PmuStatus1 = 0x00,
    PmuStatus2 = 0x01,
    DataBuffer0 = 0x04,
    DataBuffer1 = 0x05,
    DataBuffer2 = 0x06,
    DataBuffer3 = 0x07,
    PmuCommonConfig = 0x10,
    BatfetControl = 0x12,
    DieTempControl = 0x13,
    MinSysVoltageControl = 0x14,
    InputVoltageLimitControl = 0x15,
    InputCurrentLimitControl = 0x16,
    ResetFuelGauge = 0x17,
    ChargerFuelGaugeWatchdogControl = 0x18,
    WatchdogControl = 0x19,
    LowBatteryWarningThreshold = 0x1A,
    PwrOnStatus = 0x20,
    PwrOffStatus = 0x21,
    PwrOffEnable = 0x22,
    PwrOffDcdcOvpUvpControl = 0x23,
    VsysVoltagePwrOffThreshold = 0x24,
    PwrOkSettingPwrOffSequenceControl = 0x25,
    SleepWakeupControl = 0x26,
    IrqLevelOffOnLevelSetting = 0x27,
    FastPwrOnSetting0 = 0x28,
    FastPwrOnSetting1 = 0x29,
    FastPwrOnSetting2 = 0x2A,
    FastPwrOnSetting3 = 0x2B,
    AdcChannelEnableControl = 0x30,
    VbatHigh = 0x34,
    VbatLow = 0x35,
    TsHigh = 0x36,
    TsLow = 0x37,
    VbusHigh = 0x38,
    VbusLow = 0x39,
    VsysHigh = 0x3A,
    VsysLow = 0x3B,
    TdieHigh = 0x3C,
    TdieLow = 0x3D,
    IrqEnable0 = 0x40,
    IrqEnable1 = 0x41,
    IrqEnable2 = 0x42,
    IrqStatus0 = 0x48,
    IrqStatus1 = 0x49,
    IrqStatus2 = 0x4A,
    TsPinControl = 0x50,
    VLTFCHGSetting = 0x54,
    VHTFCHGSetting = 0x55,
    LdosOnOffControl0 = 0x90,
    LdosOnOffControl1 = 0x91,
    Aldo1VoltageSetting = 0x92,
    Aldo2VoltageSetting = 0x93,
    Aldo3VoltageSetting = 0x94,
    Aldo4VoltageSetting = 0x95,
    Bldo1VoltageSetting = 0x96,
    Bldo2VoltageSetting = 0x97,
    CpuLdoVoltageSetting = 0x98,
    Dldo1VoltageSetting = 0x99,
    Dldo2VoltageSetting = 0x9A,
    BatteryParameter = 0xA1,
    FuelGaugeControl = 0xA2,
    BatteryPercentageData = 0xA4,
}

#[bitfield(u8)]
pub struct PmuStatus1 {
    pub current_limit_state: bool,       // bit 0
    pub thermal_regulation_status: bool, // bit 1
    pub battery_active_mode: bool,       // bit 2
    pub battery_present: bool,           // bit 3
    pub batfet_state: bool,              // bit 4
    pub vbus_good: bool,                 // bit 5
    #[bits(2)] // Reserved bits 6-7
    _reserved: u8,
}

#[bitfield(u8)]
pub struct PmuStatus2 {
    #[bits(3)]
    pub charging_status: ChargingStatus, // bits 0-2
    pub vindpm_status: bool, // bit 3
    pub system_status: bool, // bit 4
    #[bits(2)]
    pub battery_current_direction: BatteryCurrentDirection, // bits 5-6
    #[bits(1)]
    _reserved: u8, // bit 7
}

#[bitfield(u8)]
pub struct LdosOnOffControl0 {
    pub aldo1: bool,     // bit 0
    pub aldo2: bool,     // bit 1
    pub aldo3: bool,     // bit 2
    pub aldo4: bool,     // bit 3
    pub bldo1: bool,     // bit 4
    pub bldo2: bool,     // bit 5
    pub cpus_ldo1: bool, // bit 6
    pub dldo1: bool,     // bit 7
}

#[bitfield(u8)]
pub struct LdosOnOffControl1 {
    pub dldo2: bool, // bit 0
    #[bits(7)] // Reserved bits 1-7
    _reserved: u8,
}

#[bitfield(u8)]
pub struct InputVoltageLimitControl {
    #[bits(4)]
    pub vindpm_config: u8, // bits 0-3
    #[bits(4)]
    _reserved: u8, // bits 4-7
}

#[bitfield(u8)]
pub struct InputCurrentLimitControl {
    #[bits(3)]
    pub input_current_limit: u8, // bits 0-2
    #[bits(5)]
    _reserved: u8, // bits 3-7
}

#[bitfield(u8)]
pub struct WatchdogControl {
    #[bits(3)]
    pub watchdog_timer_config: u8, // bits 0-2
    pub watchdog_clear_signal: bool, // bit 3
    #[bits(2)]
    pub watchdog_reset_config: u8, // bits 4-5
    #[bits(2)]
    _reserved: u8,     // bits 6-7
}

#[bitfield(u8)]
pub struct IrqEnable0 {
    pub socwl2_irq_enable: bool, // bit 0
    pub socwl1_irq_enable: bool, // bit 1
    pub gwdt_irq_enable: bool,   // bit 2
    pub lowsoc_irq_enable: bool, // bit 3
    pub bcot_irq_enable: bool,   // bit 4
    pub bcut_irq_enable: bool,   // bit 5
    pub bwot_irq_enable: bool,   // bit 6
    pub bwut_irq_enable: bool,   // bit 7
}

#[bitfield(u8)]
pub struct IrqEnable1 {
    pub vbus_insert_irq_enable: bool,    // bit 0
    pub vbus_remove_irq_enable: bool,    // bit 1
    pub battery_insert_irq_enable: bool, // bit 2
    pub battery_remove_irq_enable: bool, // bit 3
    pub ponsp_irq_enable: bool,          // bit 4
    pub ponlp_irq_enable: bool,          // bit 5
    pub ponne_irq_enable: bool,          // bit 6
    pub ponpe_irq_enable: bool,          // bit 7
}

#[bitfield(u8)]
pub struct BatteryPercentageData {
    #[bits(8)]
    pub percentage: u8, // bits 0-7
}

#[bitfield(u8)]
pub struct PmuCommonConfig {
    pub soft_pwroff: bool,           // bit 0
    pub restart_soc_system: bool,    // bit 1
    pub pwron_shutdown_enable: bool, // bit 2
    pub pwrok_restart_enable: bool,  // bit 3
    #[bits(1)]
    pub reserved1: u8, // bit 4
    pub off_discharge_enable: bool,  // bit 5
    #[bits(2)]
    pub reserved2: u8, // bits 6-7
}

#[bitfield(u8)]
pub struct FuelGaugeControl {
    pub watchdog_enable: bool,              // bit 0
    pub cell_battery_charge_enable: bool,   // bit 1
    pub button_battery_charge_enable: bool, // bit 2
    pub gauge_module_enable: bool,          // bit 3
    #[bits(4)]
    pub reserved: u8,         // bits 4-7
}

#[bitfield(u8)]
pub struct BatteryParameter {
    #[bits(8)]
    pub parameter: u8, // bits 0-7
}

#[derive(Debug, Copy, Clone)]
#[repr(u8)]
pub enum BatteryCurrentDirection {
    Standby = 0,
    Charge = 1,
    Discharge = 2,
    Reserved = 3,
}

impl BatteryCurrentDirection {
    const fn into_bits(self) -> u8 {
        self as _
    }

    const fn from_bits(value: u8) -> Self {
        match value {
            0 => Self::Standby,
            1 => Self::Charge,
            2 => Self::Discharge,
            _ => Self::Reserved,
        }
    }
}

#[derive(Debug, Copy, Clone)]
#[repr(u8)]
pub enum ChargingStatus {
    Trickle = 0,
    PreCharge = 1,
    ConstantCurrent = 2,
    ConstantVoltage = 3,
    ChargeDone = 4,
    NotCharging = 5,
    Unknown = 6,
}

impl ChargingStatus {
    const fn into_bits(self) -> u8 {
        self as _
    }

    const fn from_bits(value: u8) -> Self {
        match value {
            0 => Self::Trickle,
            1 => Self::PreCharge,
            2 => Self::ConstantCurrent,
            3 => Self::ConstantVoltage,
            4 => Self::ChargeDone,
            5 => Self::NotCharging,
            6 => Self::Unknown,
            _ => Self::Unknown,
        }
    }
}

pub const ADDRESS: u8 = 0x34;

pub struct Axp2101<I2C>
where
    I2C: I2c,
{
    i2c: I2C,
}

impl<I2C> Axp2101<I2C>
where
    I2C: I2c,
{
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    /// Gets the PMU Status 1 register
    pub fn get_pmu_status1(&mut self) -> Result<PmuStatus1, I2C::Error> {
        let reg_value = self.read_u8(Register::PmuStatus1)?;
        Ok(PmuStatus1::from_bits(reg_value))
    }

    /// Gets the PMU Status 2 register
    pub fn get_pmu_status2(&mut self) -> Result<PmuStatus2, I2C::Error> {
        let reg_value = self.read_u8(Register::PmuStatus2)?;
        Ok(PmuStatus2::from_bits(reg_value))
    }

    /// Sets the PMU Common Config register
    pub fn set_pmu_common_config(&mut self, value: &PmuCommonConfig) -> Result<(), I2C::Error> {
        self.write_u8(Register::PmuCommonConfig, value.into_bits())
    }

    /// Gets the PMU Common Config register
    pub fn get_pmu_common_config(&mut self) -> Result<PmuCommonConfig, I2C::Error> {
        let reg_value = self.read_u8(Register::PmuCommonConfig)?;
        Ok(PmuCommonConfig::from_bits(reg_value))
    }

    /// Sets the Fuel Gauge Control register
    pub fn set_fuel_gauge_control(&mut self, value: &FuelGaugeControl) -> Result<(), I2C::Error> {
        self.write_u8(Register::FuelGaugeControl, value.into_bits())
    }

    /// Gets the Fuel Gauge Control register
    pub fn get_fuel_gauge_control(&mut self) -> Result<FuelGaugeControl, I2C::Error> {
        let reg_value = self.read_u8(Register::FuelGaugeControl)?;
        Ok(FuelGaugeControl::from_bits(reg_value))
    }

    /// Gets the Battery Percentage Data register
    pub fn get_battery_percentage_data(&mut self) -> Result<BatteryPercentageData, I2C::Error> {
        let reg_value = self.read_u8(Register::BatteryPercentageData)?;
        Ok(BatteryPercentageData::from_bits(reg_value))
    }

    /// Gets the Battery Parameter register
    pub fn get_battery_parameter(&mut self) -> Result<BatteryParameter, I2C::Error> {
        let reg_value = self.read_u8(Register::BatteryParameter)?;
        Ok(BatteryParameter::from_bits(reg_value))
    }

    /// Enables or disables the specified LDO from control register 0
    pub fn set_ldos_on_off_control0(
        &mut self,
        value: &LdosOnOffControl0,
    ) -> Result<(), I2C::Error> {
        self.write_u8(Register::LdosOnOffControl0, value.into_bits())
    }

    /// Reads whether the specified LDO is enabled from control register 0
    pub fn get_ldos_on_off_control0(&mut self) -> Result<LdosOnOffControl0, I2C::Error> {
        let reg_value = self.read_u8(Register::LdosOnOffControl0)?;
        Ok(LdosOnOffControl0::from_bits(reg_value))
    }

    /// Enables or disables the specified LDO from control register 1
    pub fn set_ldos_on_off_control1(
        &mut self,
        value: &LdosOnOffControl1,
    ) -> Result<(), I2C::Error> {
        self.write_u8(Register::LdosOnOffControl1, value.into_bits())
    }

    /// Reads whether the specified LDO is enabled from control register 1
    pub fn get_ldos_on_off_control1(&mut self) -> Result<LdosOnOffControl1, I2C::Error> {
        let reg_value = self.read_u8(Register::LdosOnOffControl1)?;
        Ok(LdosOnOffControl1::from_bits(reg_value))
    }

    /// Sets the input voltage limit control
    pub fn set_input_voltage_limit_control(
        &mut self,
        value: &InputVoltageLimitControl,
    ) -> Result<(), I2C::Error> {
        self.write_u8(Register::InputVoltageLimitControl, value.into_bits())
    }

    /// Gets the input voltage limit control
    pub fn get_input_voltage_limit_control(
        &mut self,
    ) -> Result<InputVoltageLimitControl, I2C::Error> {
        let reg_value = self.read_u8(Register::InputVoltageLimitControl)?;
        Ok(InputVoltageLimitControl::from_bits(reg_value))
    }

    /// Sets the input current limit control
    pub fn set_input_current_limit_control(
        &mut self,
        value: &InputCurrentLimitControl,
    ) -> Result<(), I2C::Error> {
        self.write_u8(Register::InputCurrentLimitControl, value.into_bits())
    }

    /// Gets the input current limit control
    pub fn get_input_current_limit_control(
        &mut self,
    ) -> Result<InputCurrentLimitControl, I2C::Error> {
        let reg_value = self.read_u8(Register::InputCurrentLimitControl)?;
        Ok(InputCurrentLimitControl::from_bits(reg_value))
    }

    /// Sets the watchdog control register
    pub fn set_watchdog_control(&mut self, value: &WatchdogControl) -> Result<(), I2C::Error> {
        self.write_u8(Register::WatchdogControl, value.into_bits())
    }

    /// Gets the watchdog control register
    pub fn get_watchdog_control(&mut self) -> Result<WatchdogControl, I2C::Error> {
        let reg_value = self.read_u8(Register::WatchdogControl)?;
        Ok(WatchdogControl::from_bits(reg_value))
    }

    /// Sets the IRQ Enable 0 register
    pub fn set_irq_enable0(&mut self, value: &IrqEnable0) -> Result<(), I2C::Error> {
        self.write_u8(Register::IrqEnable0, value.into_bits())
    }

    /// Gets the IRQ Enable 0 register
    pub fn get_irq_enable0(&mut self) -> Result<IrqEnable0, I2C::Error> {
        let reg_value = self.read_u8(Register::IrqEnable0)?;
        Ok(IrqEnable0::from_bits(reg_value))
    }

    /// Sets the IRQ Enable 1 register
    pub fn set_irq_enable1(&mut self, value: &IrqEnable1) -> Result<(), I2C::Error> {
        self.write_u8(Register::IrqEnable1, value.into_bits())
    }

    /// Gets the IRQ Enable 1 register
    pub fn get_irq_enable1(&mut self) -> Result<IrqEnable1, I2C::Error> {
        let reg_value = self.read_u8(Register::IrqEnable1)?;
        Ok(IrqEnable1::from_bits(reg_value))
    }

    /// Gets the Vsys voltage by reading the high and low register values
    pub fn get_vsys_voltage(&mut self) -> Result<u16, I2C::Error> {
        let value = self.read_u16(Register::VsysHigh)?;
        Ok(value)
    }

    /// Gets the Vbat voltage by reading the high and low register values
    pub fn get_vbat_voltage(&mut self) -> Result<u16, I2C::Error> {
        let value = self.read_u16(Register::VbatHigh)?;
        Ok(value)
    }

    /// Gets the Vbat voltage by reading the high and low register values
    pub fn get_vbus_voltage(&mut self) -> Result<u16, I2C::Error> {
        let value = self.read_u16(Register::VbusHigh)?;
        Ok(value)
    }

    // Writes u8 `value` from `register`
    fn write_u8(&mut self, register: Register, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(ADDRESS, &[register as u8, value])
    }

    // Reads u8 `value` from `register`
    fn read_u8(&mut self, register: Register) -> Result<u8, I2C::Error> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(ADDRESS, &[register as u8], &mut buf)?;
        Ok(buf[0])
    }

    // Reads u16 `value` from `register`
    fn read_u16(&mut self, register: Register) -> Result<u16, I2C::Error> {
        let mut buf = [0u8; 2];
        self.i2c.write_read(ADDRESS, &[register as u8], &mut buf)?;
        Ok(u16::from_be_bytes(buf))
    }
}
