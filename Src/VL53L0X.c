#include "VL53L0X.h"
#include "main.h"

// Pull in the I2C handle from main.c
extern I2C_HandleTypeDef hi2c1;

// Wrapper for writing I2C via STM32 HAL
uint8_t i2c_write(uint8_t address, uint8_t *data, uint8_t count) {
    return HAL_I2C_Master_Transmit(&hi2c1, address, data, count, 100);
}

// Wrapper for reading I2C via STM32 HAL
uint8_t i2c_read(uint8_t address, uint8_t *data, uint8_t count) {
    return HAL_I2C_Master_Receive(&hi2c1, address, data, count, 100);
}

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val) (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)


void VL53L0X_setAddress(struct VL53L0X* dev, uint8_t new_addr)
{
  VL53L0X_writeReg(dev, I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
  dev->address = new_addr;
}

bool VL53L0X_init(struct VL53L0X* dev)
{
  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  if (dev->io_2v8)
  {
    VL53L0X_writeReg(dev, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, VL53L0X_readReg(dev, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01 ); 
  }

  // "Set I2C standard mode"
  VL53L0X_writeReg(dev, 0x88, 0x00);

  VL53L0X_writeReg(dev, 0x80, 0x01);
  VL53L0X_writeReg(dev, 0xFF, 0x01);
  VL53L0X_writeReg(dev, 0x00, 0x00);
  dev->stop_variable = VL53L0X_readReg(dev, 0x91);
  VL53L0X_writeReg(dev, 0x00, 0x01);
  VL53L0X_writeReg(dev, 0xFF, 0x00);
  VL53L0X_writeReg(dev, 0x80, 0x00);

  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  VL53L0X_writeReg(dev, MSRC_CONFIG_CONTROL, VL53L0X_readReg(dev,  MSRC_CONFIG_CONTROL) | 0x12);

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  VL53L0X_setSignalRateLimit(dev, 0.25);

  VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xFF);

  uint8_t spad_count;
  bool spad_type_is_aperture;
  if (!VL53L0X_getSpadInfo(dev, &spad_count, &spad_type_is_aperture)) { return false; }

  uint8_t ref_spad_map[6];
  VL53L0X_readMulti(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  VL53L0X_writeReg(dev, 0xFF, 0x01);
  VL53L0X_writeReg(dev, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  VL53L0X_writeReg(dev, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  VL53L0X_writeReg(dev, 0xFF, 0x00);
  VL53L0X_writeReg(dev, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; 
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }

  VL53L0X_writeMulti(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  VL53L0X_writeReg(dev, 0xFF, 0x01);
  VL53L0X_writeReg(dev, 0x00, 0x00);

  VL53L0X_writeReg(dev, 0xFF, 0x00);
  VL53L0X_writeReg(dev, 0x09, 0x00);
  VL53L0X_writeReg(dev, 0x10, 0x00);
  VL53L0X_writeReg(dev, 0x11, 0x00);

  VL53L0X_writeReg(dev, 0x24, 0x01);
  VL53L0X_writeReg(dev, 0x25, 0xFF);
  VL53L0X_writeReg(dev, 0x75, 0x00);

  VL53L0X_writeReg(dev, 0xFF, 0x01);
  VL53L0X_writeReg(dev, 0x4E, 0x2C);
  VL53L0X_writeReg(dev, 0x48, 0x00);
  VL53L0X_writeReg(dev, 0x30, 0x20);

  VL53L0X_writeReg(dev, 0xFF, 0x00);
  VL53L0X_writeReg(dev, 0x30, 0x09);
  VL53L0X_writeReg(dev, 0x54, 0x00);
  VL53L0X_writeReg(dev, 0x31, 0x04);
  VL53L0X_writeReg(dev, 0x32, 0x03);
  VL53L0X_writeReg(dev, 0x40, 0x83);
  VL53L0X_writeReg(dev, 0x46, 0x25);
  VL53L0X_writeReg(dev, 0x60, 0x00);
  VL53L0X_writeReg(dev, 0x27, 0x00);
  VL53L0X_writeReg(dev, 0x50, 0x06);
  VL53L0X_writeReg(dev, 0x51, 0x00);
  VL53L0X_writeReg(dev, 0x52, 0x96);
  VL53L0X_writeReg(dev, 0x56, 0x08);
  VL53L0X_writeReg(dev, 0x57, 0x30);
  VL53L0X_writeReg(dev, 0x61, 0x00);
  VL53L0X_writeReg(dev, 0x62, 0x00);
  VL53L0X_writeReg(dev, 0x64, 0x00);
  VL53L0X_writeReg(dev, 0x65, 0x00);
  VL53L0X_writeReg(dev, 0x66, 0xA0);

  VL53L0X_writeReg(dev, 0xFF, 0x01);
  VL53L0X_writeReg(dev, 0x22, 0x32);
  VL53L0X_writeReg(dev, 0x47, 0x14);
  VL53L0X_writeReg(dev, 0x49, 0xFF);
  VL53L0X_writeReg(dev, 0x4A, 0x00);

  VL53L0X_writeReg(dev, 0xFF, 0x00);
  VL53L0X_writeReg(dev, 0x7A, 0x0A);
  VL53L0X_writeReg(dev, 0x7B, 0x00);
  VL53L0X_writeReg(dev, 0x78, 0x21);

  VL53L0X_writeReg(dev, 0xFF, 0x01);
  VL53L0X_writeReg(dev, 0x23, 0x34);
  VL53L0X_writeReg(dev, 0x42, 0x00);
  VL53L0X_writeReg(dev, 0x44, 0xFF);
  VL53L0X_writeReg(dev, 0x45, 0x26);
  VL53L0X_writeReg(dev, 0x46, 0x05);
  VL53L0X_writeReg(dev, 0x40, 0x40);
  VL53L0X_writeReg(dev, 0x0E, 0x06);
  VL53L0X_writeReg(dev, 0x20, 0x1A);
  VL53L0X_writeReg(dev, 0x43, 0x40);

  VL53L0X_writeReg(dev, 0xFF, 0x00);
  VL53L0X_writeReg(dev, 0x34, 0x03);
  VL53L0X_writeReg(dev, 0x35, 0x44);

  VL53L0X_writeReg(dev, 0xFF, 0x01);
  VL53L0X_writeReg(dev, 0x31, 0x04);
  VL53L0X_writeReg(dev, 0x4B, 0x09);
  VL53L0X_writeReg(dev, 0x4C, 0x05);
  VL53L0X_writeReg(dev, 0x4D, 0x04);

  VL53L0X_writeReg(dev, 0xFF, 0x00);
  VL53L0X_writeReg(dev, 0x44, 0x00);
  VL53L0X_writeReg(dev, 0x45, 0x20);
  VL53L0X_writeReg(dev, 0x47, 0x08);
  VL53L0X_writeReg(dev, 0x48, 0x28);
  VL53L0X_writeReg(dev, 0x67, 0x00);
  VL53L0X_writeReg(dev, 0x70, 0x04);
  VL53L0X_writeReg(dev, 0x71, 0x01);
  VL53L0X_writeReg(dev, 0x72, 0xFE);
  VL53L0X_writeReg(dev, 0x76, 0x00);
  VL53L0X_writeReg(dev, 0x77, 0x00);

  VL53L0X_writeReg(dev, 0xFF, 0x01);
  VL53L0X_writeReg(dev, 0x0D, 0x01);

  VL53L0X_writeReg(dev, 0xFF, 0x00);
  VL53L0X_writeReg(dev, 0x80, 0x01);
  VL53L0X_writeReg(dev, 0x01, 0xF8);

  VL53L0X_writeReg(dev, 0xFF, 0x01);
  VL53L0X_writeReg(dev, 0x8E, 0x01);
  VL53L0X_writeReg(dev, 0x00, 0x01);
  VL53L0X_writeReg(dev, 0xFF, 0x00);
  VL53L0X_writeReg(dev, 0x80, 0x00);

  VL53L0X_writeReg(dev, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  VL53L0X_writeReg(dev, GPIO_HV_MUX_ACTIVE_HIGH, VL53L0X_readReg(dev,  GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); 
  VL53L0X_writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);

  dev->measurement_timing_budget_us = VL53L0X_getMeasurementTimingBudget(dev);

  VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xE8);

  VL53L0X_setMeasurementTimingBudget(dev, dev->measurement_timing_budget_us);

  VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!VL53L0X_performSingleRefCalibration(dev, 0x40)) { return false; }

  VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!VL53L0X_performSingleRefCalibration(dev, 0x00)) { return false; }

  VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xE8);

  return true;
}

void VL53L0X_writeReg(struct VL53L0X* dev, uint8_t reg, uint8_t value)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = value;
    dev->last_status = i2c_write(dev->address, buf, 2);
}

void VL53L0X_writeReg16Bit(struct VL53L0X* dev, uint8_t reg, uint16_t value)
{
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (uint8_t) (value >> 8);
    buf[2] = (uint8_t) (value & 0xFF);
    dev->last_status = i2c_write(dev->address, buf, 3);
}

void VL53L0X_writeReg32Bit(struct VL53L0X* dev, uint8_t reg, uint32_t value)
{
    uint8_t buf[5];
    buf[0] = reg;
    buf[1] = (uint8_t) (value >> 24);
    buf[2] = (uint8_t) (value >> 16);
    buf[3] = (uint8_t) (value >> 8);
    buf[4] = (uint8_t) (value & 0xFF);
    dev->last_status = i2c_write(dev->address, buf, 5);
}

uint8_t VL53L0X_readReg(struct VL53L0X* dev, uint8_t reg)
{
  uint8_t value;
  i2c_write(dev->address, &reg, 1);
  dev->last_status = i2c_read(dev->address, &value, 1);
  return value;
}

uint16_t VL53L0X_readReg16Bit(struct VL53L0X* dev, uint8_t reg)
{
  uint16_t value;
  uint8_t buf[2];
  i2c_write(dev->address, &reg, 1);
  dev->last_status = i2c_read(dev->address, buf, 2);
  value = (uint16_t) ( buf[0] << 8 );
  value |= (uint16_t) buf[1];
  return value;
}

uint32_t VL53L0X_readReg32Bit(struct VL53L0X* dev, uint8_t reg)
{
  uint32_t value;
  uint8_t buf[4];
  i2c_write(dev->address, &reg, 1);
  dev->last_status = i2c_read(dev->address, buf, 4);
  value = (uint32_t) ( buf[0] << 24 );
  value |= (uint32_t) ( buf[1] << 16 );
  value |= (uint32_t) ( buf[2] << 8 );
  value |= (uint32_t) buf[3];
  return value;
}

void VL53L0X_writeMulti(struct VL53L0X* dev, uint8_t reg, uint8_t* src, uint8_t count)
{
    i2c_write(dev->address, &reg, 1);
    dev->last_status = i2c_write(dev->address, src, count);
}

void VL53L0X_readMulti(struct VL53L0X* dev, uint8_t reg, uint8_t * dst, uint8_t count)
{
    i2c_write(dev->address, &reg, 1);
    dev->last_status = i2c_read(dev->address, dst, count);
}

bool VL53L0X_setSignalRateLimit(struct VL53L0X* dev, float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }
  VL53L0X_writeReg16Bit(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
  return true;
}

float VL53L0X_getSignalRateLimit(struct VL53L0X* dev)
{
  return (float)VL53L0X_readReg16Bit(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

bool VL53L0X_setMeasurementTimingBudget(struct VL53L0X* dev, uint32_t budget_us)
{
  struct VL53L0X_SequenceStepEnables enables;
  struct VL53L0X_SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320; 
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;
  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  VL53L0X_getSequenceStepEnables(dev, &enables);
  VL53L0X_getSequenceStepTimeouts(dev, &enables, &timeouts);

  if (enables.tcc) {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss) {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  } else if (enables.msrc) {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range) {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range) {
    used_budget_us += FinalRangeOverhead;

    if (used_budget_us > budget_us) { return false; }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;
    uint16_t final_range_timeout_mclks = VL53L0X_timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range) {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    VL53L0X_writeReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, VL53L0X_encodeTimeout(final_range_timeout_mclks));
    dev->measurement_timing_budget_us = budget_us; 
  }
  return true;
}

uint32_t VL53L0X_getMeasurementTimingBudget(struct VL53L0X* dev)
{
  struct VL53L0X_SequenceStepEnables enables;
  struct VL53L0X_SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910; 
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t budget_us = StartOverhead + EndOverhead;

  VL53L0X_getSequenceStepEnables(dev, &enables);
  VL53L0X_getSequenceStepTimeouts(dev, &enables, &timeouts);

  if (enables.tcc) {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss) {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  } else if (enables.msrc) {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range) {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range) {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  dev->measurement_timing_budget_us = budget_us; 
  return budget_us;
}

bool VL53L0X_setVcselPulsePeriod(struct VL53L0X* dev, enum VL53L0X_vcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);
  struct VL53L0X_SequenceStepEnables enables;
  struct VL53L0X_SequenceStepTimeouts timeouts;

  VL53L0X_getSequenceStepEnables(dev, &enables);
  VL53L0X_getSequenceStepTimeouts(dev, &enables, &timeouts);

  if (type == VcselPeriodPreRange) {
    switch (period_pclks) {
      case 12: VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18); break;
      case 14: VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30); break;
      case 16: VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40); break;
      case 18: VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50); break;
      default: return false;
    }
    VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
    VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    uint16_t new_pre_range_timeout_mclks = VL53L0X_timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);
    VL53L0X_writeReg16Bit(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, VL53L0X_encodeTimeout(new_pre_range_timeout_mclks));

    uint16_t new_msrc_timeout_mclks = VL53L0X_timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);
    VL53L0X_writeReg(dev, MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

  } else if (type == VcselPeriodFinalRange) {
    switch (period_pclks) {
      case 8:
        VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        VL53L0X_writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        VL53L0X_writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        VL53L0X_writeReg(dev, 0xFF, 0x01);
        VL53L0X_writeReg(dev, ALGO_PHASECAL_LIM, 0x30);
        VL53L0X_writeReg(dev, 0xFF, 0x00);
        break;
      case 10:
        VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        VL53L0X_writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        VL53L0X_writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        VL53L0X_writeReg(dev, 0xFF, 0x01);
        VL53L0X_writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
        VL53L0X_writeReg(dev, 0xFF, 0x00);
        break;
      case 12:
        VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        VL53L0X_writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        VL53L0X_writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        VL53L0X_writeReg(dev, 0xFF, 0x01);
        VL53L0X_writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
        VL53L0X_writeReg(dev, 0xFF, 0x00);
        break;
      case 14:
        VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        VL53L0X_writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        VL53L0X_writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        VL53L0X_writeReg(dev, 0xFF, 0x01);
        VL53L0X_writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
        VL53L0X_writeReg(dev, 0xFF, 0x00);
        break;
      default: return false;
    }

    VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
    uint16_t new_final_range_timeout_mclks = VL53L0X_timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range) {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    VL53L0X_writeReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, VL53L0X_encodeTimeout(new_final_range_timeout_mclks));
  } else {
    return false;
  }

  VL53L0X_setMeasurementTimingBudget(dev, dev->measurement_timing_budget_us);

  uint8_t sequence_config = VL53L0X_readReg(dev,  SYSTEM_SEQUENCE_CONFIG);
  VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x02);
  VL53L0X_performSingleRefCalibration(dev, 0x0);
  VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, sequence_config);

  return true;
}

uint8_t VL53L0X_getVcselPulsePeriod(struct VL53L0X* dev, enum VL53L0X_vcselPeriodType type)
{
  if (type == VcselPeriodPreRange) {
    return decodeVcselPeriod(VL53L0X_readReg(dev,  PRE_RANGE_CONFIG_VCSEL_PERIOD));
  } else if (type == VcselPeriodFinalRange) {
    return decodeVcselPeriod(VL53L0X_readReg(dev,  FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  } else { return 255; }
}

void VL53L0X_startContinuous(struct VL53L0X* dev, uint32_t period_ms)
{
  VL53L0X_writeReg(dev, 0x80, 0x01);
  VL53L0X_writeReg(dev, 0xFF, 0x01);
  VL53L0X_writeReg(dev, 0x00, 0x00);
  VL53L0X_writeReg(dev, 0x91, dev->stop_variable);
  VL53L0X_writeReg(dev, 0x00, 0x01);
  VL53L0X_writeReg(dev, 0xFF, 0x00);
  VL53L0X_writeReg(dev, 0x80, 0x00);

  if (period_ms != 0) {
    uint16_t osc_calibrate_val = VL53L0X_readReg16Bit(dev, OSC_CALIBRATE_VAL);
    if (osc_calibrate_val != 0) {
      period_ms *= osc_calibrate_val;
    }
    VL53L0X_writeReg32Bit(dev, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
    VL53L0X_writeReg(dev, SYSRANGE_START, 0x04); 
  } else {
    VL53L0X_writeReg(dev, SYSRANGE_START, 0x02); 
  }
}

void VL53L0X_stopContinuous(struct VL53L0X* dev)
{
  VL53L0X_writeReg(dev, SYSRANGE_START, 0x01); 
  VL53L0X_writeReg(dev, 0xFF, 0x01);
  VL53L0X_writeReg(dev, 0x00, 0x00);
  VL53L0X_writeReg(dev, 0x91, 0x00);
  VL53L0X_writeReg(dev, 0x00, 0x01);
  VL53L0X_writeReg(dev, 0xFF, 0x00);
}

uint16_t VL53L0X_readRangeContinuousMillimeters(struct VL53L0X* dev)
{
  VL53L0X_startTimeout(dev);
  while ((VL53L0X_readReg(dev,  RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    if (VL53L0X_checkTimeoutExpired(dev)) {
      dev->did_timeout = true;
      return 65535;
    }
  }

  uint16_t range = VL53L0X_readReg16Bit(dev, RESULT_RANGE_STATUS + 10);
  VL53L0X_writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);
  return range;
}

uint16_t VL53L0X_readRangeSingleMillimeters(struct VL53L0X* dev)
{
  VL53L0X_writeReg(dev, 0x80, 0x01);
  VL53L0X_writeReg(dev, 0xFF, 0x01);
  VL53L0X_writeReg(dev, 0x00, 0x00);
  VL53L0X_writeReg(dev, 0x91, dev->stop_variable);
  VL53L0X_writeReg(dev, 0x00, 0x01);
  VL53L0X_writeReg(dev, 0xFF, 0x00);
  VL53L0X_writeReg(dev, 0x80, 0x00);

  VL53L0X_writeReg(dev, SYSRANGE_START, 0x01);

  VL53L0X_startTimeout(dev);
  while (VL53L0X_readReg(dev,  SYSRANGE_START) & 0x01) {
    if (VL53L0X_checkTimeoutExpired(dev)) {
      dev->did_timeout = true;
      return 65535;
    }
  }

  return VL53L0X_readRangeContinuousMillimeters(dev);
}

bool VL53L0X_timeoutOccurred(struct VL53L0X* dev)
{
  bool tmp = dev->did_timeout;
  dev->did_timeout = false;
  return tmp;
}

bool VL53L0X_getSpadInfo(struct VL53L0X* dev, uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;
  VL53L0X_writeReg(dev, 0x80, 0x01);
  VL53L0X_writeReg(dev, 0xFF, 0x01);
  VL53L0X_writeReg(dev, 0x00, 0x00);

  VL53L0X_writeReg(dev, 0xFF, 0x06);
  VL53L0X_writeReg(dev, 0x83, VL53L0X_readReg(dev,  0x83) | 0x04);
  VL53L0X_writeReg(dev, 0xFF, 0x07);
  VL53L0X_writeReg(dev, 0x81, 0x01);

  VL53L0X_writeReg(dev, 0x80, 0x01);

  VL53L0X_writeReg(dev, 0x94, 0x6b);
  VL53L0X_writeReg(dev, 0x83, 0x00);
  VL53L0X_startTimeout(dev);
  while (VL53L0X_readReg(dev,  0x83) == 0x00) {
    if (VL53L0X_checkTimeoutExpired(dev)) { return false; }
  }
  VL53L0X_writeReg(dev, 0x83, 0x01);
  tmp = VL53L0X_readReg(dev,  0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  VL53L0X_writeReg(dev, 0x81, 0x00);
  VL53L0X_writeReg(dev, 0xFF, 0x06);
  VL53L0X_writeReg(dev, 0x83, VL53L0X_readReg(dev,  0x83)  & ~0x04);
  VL53L0X_writeReg(dev, 0xFF, 0x01);
  VL53L0X_writeReg(dev, 0x00, 0x01);

  VL53L0X_writeReg(dev, 0xFF, 0x00);
  VL53L0X_writeReg(dev, 0x80, 0x00);

  return true;
}

void VL53L0X_getSequenceStepEnables(struct VL53L0X* dev, struct VL53L0X_SequenceStepEnables* enables)
{
  uint8_t sequence_config = VL53L0X_readReg(dev,  SYSTEM_SEQUENCE_CONFIG);
  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

void VL53L0X_getSequenceStepTimeouts(struct VL53L0X* dev, struct VL53L0X_SequenceStepEnables* enables, struct VL53L0X_SequenceStepTimeouts* timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(dev, VcselPeriodPreRange);
  timeouts->msrc_dss_tcc_mclks = VL53L0X_readReg(dev,  MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us = VL53L0X_timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks = VL53L0X_decodeTimeout(VL53L0X_readReg16Bit(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us = VL53L0X_timeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(dev, VcselPeriodFinalRange);
  timeouts->final_range_mclks = VL53L0X_decodeTimeout(VL53L0X_readReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range) {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }
  timeouts->final_range_us = VL53L0X_timeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

uint16_t VL53L0X_decodeTimeout(uint16_t reg_val)
{
  return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

uint16_t VL53L0X_encodeTimeout(uint16_t timeout_mclks)
{
  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0) {
    ls_byte = timeout_mclks - 1;
    while ((ls_byte & 0xFFFFFF00) > 0) {
      ls_byte >>= 1;
      ms_byte++;
    }
    return (ms_byte << 8) | (ls_byte & 0xFF);
  } else { return 0; }
}

uint32_t VL53L0X_timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

uint32_t VL53L0X_timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

bool VL53L0X_performSingleRefCalibration(struct VL53L0X* dev, uint8_t vhv_init_byte)
{
  VL53L0X_writeReg(dev, SYSRANGE_START, 0x01 | vhv_init_byte); 
  VL53L0X_startTimeout(dev);
  while ((VL53L0X_readReg(dev,  RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    if (VL53L0X_checkTimeoutExpired(dev)) { return false; }
  }
  VL53L0X_writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);
  VL53L0X_writeReg(dev, SYSRANGE_START, 0x00);
  return true;
}

// Fixed Timeout functions using STM32 HAL_GetTick()
void VL53L0X_startTimeout(struct VL53L0X* dev){
    dev->timeout_start_ms = HAL_GetTick();
}

bool VL53L0X_checkTimeoutExpired(struct VL53L0X* dev){
    return (dev->io_timeout > 0 && (HAL_GetTick() - dev->timeout_start_ms) > dev->io_timeout);
}