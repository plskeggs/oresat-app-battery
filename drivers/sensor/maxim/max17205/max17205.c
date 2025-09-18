/*
 * Copyright 2021 Matija Tudan
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(max17205, CONFIG_SENSOR_LOG_LEVEL);

#include "max17205.h"

#define DT_DRV_COMPAT maxim_max17205
#define I2C_DEV(cfg, reg) (((reg) > 0xFFU) ? &(cfg)->i2c_aux : &(cfg)->i2c)
#define REG_ADDR(reg) ((reg) & 0xFFU)

/**
 * @brief Read a register value
 *
 * Registers have an address and a 16-bit value
 *
 * @param dev MAX17205 device to access
 * @param reg_addr Register address to read
 * @param valp Place to put the value on success
 * @return 0 if successful, or negative error code from I2C API
 */
static int max17205_reg_read(const struct device *dev, uint16_t reg_addr, int16_t *valp)
{
    const struct max17205_config *cfg = dev->config;
    uint8_t i2c_data[2];
    int rc;

    rc = i2c_burst_read_dt(I2C_DEV(cfg, reg_addr), REG_ADDR(reg_addr), i2c_data, 2);
    if (rc < 0) {
        LOG_ERR("Unable to read register 0x%02x", reg_addr);
        return rc;
    }
    *valp = ((int16_t)i2c_data[1] << 8) | i2c_data[0];

    return 0;
}

/**
 * @brief Write a register value
 *
 * Registers have an address and a 16-bit value
 *
 * @param dev MAX17205 device to access
 * @param reg_addr Register address to write to
 * @param val Register value to write
 * @return 0 if successful, or negative error code from I2C API
 */
static int max17205_reg_write(const struct device *dev, uint16_t reg_addr, int16_t val)
{
    const struct max17205_config *cfg = dev->config;
    uint8_t i2c_data[3] = {REG_ADDR(reg_addr), val & 0xFF, (uint16_t)val >> 8};

    return i2c_write_dt(I2C_DEV(cfg, reg_addr), i2c_data, sizeof(i2c_data));
}

/**
 * @brief Convert sensor value from millis
 *
 * @param val Where to store converted value in sensor_value format
 * @param val_millis Value in millis
 */
static void convert_millis(struct sensor_value *val, int32_t val_millis)
{
    val->val1 = val_millis / 1000;
    val->val2 = (val_millis % 1000) * 1000;
}

static void convert_capacity(uint16_t rsense_mohms, const uint16_t raw, struct sensor_value *valp)
{
    // Reference datasheet table 1: Capacity LSB is 5.0μVh/RSENSE where Vh/R=Ah, unsigned.
    valp->val1 = ((uint32_t)raw * 5000U) / rsense_mohms;
    valp->val2 = 0;
}

uint16_t encode_capacity(uint16_t rsense_mohms, uint32_t dest_mAh)
{
    // Reference datasheet table 1: Capacity LSB is 5.0μVh/RSENSE where Vh/R=Ah, unsigned.
    buf = (uint16_t)((dest_mAh * rsense_mohms) / 5000U);
    return buf;
}

/**
 * @brief   Converts an MAX17205 percentage value from a
 *          register in 1% increments to be between 0% and 255%.
 *
 * @api
 */
static void convert_percentage(uint16_t raw, struct sensor_value *valp)
{
    // Reference datasheet table 1: Percentage LSB is 1/256%, unsigned.
    valp->val1 = raw / 256U;
    valp->val2 = 0;
}

/* dest_count will be between 0 and 10485 cycles */
static void convert_cycles(uint16_t raw, struct sensor_value *valp)
{
    // Reference datasheet page 33: LSB is 16%, unsigned.
    valp->val1 = raw * 16U / 100U;
    valp->val2 = 0;
}

/**
 * @brief   converts a MAX17205 voltage value from a register to
 *          mV.
 */
static void convert_voltage(uint16_t raw, struct sensor_value *valp)
{
    // Reference datasheet Table 1: Voltage LSB is 0.078125mV, unsigend.
    // Converting to mV using integer math would be buf * 78125 / 1_000_000, but that could
    // overflow a uint32_t so remove the common 5^6 factor from both numbers first.
    valp->val1 = raw * 5U / 64U;
    valp->val2 = 0;
}

/**
 * dest_mV will be between 0 and 81,918mV but realistically can be
 * at most 76.8V in a 15 cell configuration.
 */
static void convert_batt_voltage(uint16_t raw, struct sensor_value *valp)
{
    //Reference datasheet page 71: Voltage LSB is 1.25mV, unsigned.
    valp->val1 = raw * 125U / 100U;
    valp->val2 = 0;
}

/**
 * max_mV can be between 0mV and 5100mV, starting at 0.
 */
static void convert_max_voltage(uint16_t raw, struct sensor_value *valp)
{
    // Reference datasheet page 71: register MaxMinVolt is two 8 bit unsigned values with
    // 20mV/LSB, the upper byte being max and lower byte min.
    valp->val1 = (raw >> 8) * 20;   // max
    valp->val2 = 0;
}

/**
 * min_mV can be between 0mV and 5100mV, starting at 5100.
 */
static void convert_min_voltage(uint16_t raw, struct sensor_value *valp)
{
    // Reference datasheet page 71: register MaxMinVolt is two 8 bit unsigned values with
    // 20mV/LSB, the upper byte being max and lower byte min.
    valp->val1 = (raw & 0xFF) * 20; // min
    valp->val2 = 0;
}

/**
 * @brief Converts current value from a register to mA
 * @details  dest_mA can be between -51,200,000mA and 51,198,437mA but typically
 *           between -5,120mA and 5,119mA depending on rsense_uOhm.
 */
static void convert_current(uint16_t raw, struct sensor_value *valp)
{
    // Referencce datasheet table 1: Current LSB is 1.5625μV/RSENSE, signed.
    valp->val1 = (int16_t)((((int32_t)((int16_t)raw)) * 15625) / (devp->rsense_uOhm * 10));
    valp->val2 = 0;
}

/**
 * Both max_mA and min_mA can be between -102,400,000mA and 102,000,000mA but typically
 * between -10,240mA and 10,200mA depending on rsense_mohhm.
 * max_mA starts at the minimum value.
 */
static void convert_max_current(uint16_t rsense_mohms, uint16_t raw, struct sensor_value *valp)
{
    // Reference datasheet page 73: MaxMinCurrent is two packed signed 1 byte values with
    // LSB of 0.40mV/RSENSE.
    int32_t max_raw = ((raw >> 8) * 0xFFU) & 0xFFU;

    valp->val1 = (int16_t)((max_raw * 400) / rsense_mohms);
    valp->val2 = 0;
}

/**
 * Both max_mA and min_mA can be between -102,400,000mA and 102,000,000mA but typically
 * between -10,240mA and 10,200mA depending on rsense_mohhm.
 * min_mA starts at the maximum value.
 */
static void convert_min_current(uint16_t rsense_mohms, uint16_t raw, struct sensor_value *valp)
{
    // Reference datasheet page 73: MaxMinCurrent is two packed signed 1 byte values with
    // LSB of 0.40mV/RSENSE.
    int32_t min_raw = raw & 0xFFU;

    valp->val1 = (int16_t)((min_raw * 400) / rsense_mohms);
    valp->val2 = 0;
}

/**
 * @brief   Converts an MAX17205 temperature value from a
 *          register in 0.001C increments.
 * @details  dest_mC will be between -256,000mC and 255,996mC.
 */
static void convert_temperature(uint16_t raw, struct sensor_value *valp)
{
    // Reference Datasheet table 1: Temperature LSB is 1/256C, signed.
    valp->val1 = (int16_t)((int32_t)raw * 1000 / 256);
    valp->val2 = 0;
}

/**
 * @brief   Converts an MAX17205 temperature value from a
 *          register in 0.1K increments to result in C.
 */
static void convert_average_temperature(uint16_t raw, struct sensor_value *valp)
{
    // Reference datasheet page 77: "These registers display temperature in degrees
    // Celsius starting at absolute zero, -273ºC or 0ºK with an LSb of 0.1ºC."
    // FIXME: Does that mean signed Celsius or unsigned Kelvin? The converison below
    // assumes signed(!?) Kelvin but this should be checked on a live chip.
    valp->val1 = (int16_t)raw / 10 - 273;
    valp->val2 = 0;
}

/**
 * max_C will be between -128C and 127C.
 */
static void convert_max_temperature(uint16_t raw, struct sensor_value *valp)
{
    // Reference datahseet page 76: Two packed signed 1 byte Celceus values with 1C/LSB, with
    // the upper byte max and lower byte min.
    valp->val1 = raw >> 8;   // max
    valp->val2 = 0;
}

/**
 * min_C will be between -128C and 127C.
 */
static void convert_min_temperature(uint16_t raw, struct sensor_value *valp)
{
    // Reference datahseet page 76: Two packed signed 1 byte Celceus values with 1C/LSB, with
    // the upper byte max and lower byte min.
    valp->val1 = raw & 0xFF; // min
    valp->val2 = 0;
}

/**
 * @brief Converts an MAX17205 resistance value from a register
 *          in mOhms to a value between 0mOhm and 15,999mOhm.
 */

static void convert_resistance(uint16_t raw, uint16_t *dest_mohm) {
    // Reference datasheet table 1: Resistance LSB is 1/4096Ω, Unsigned
    *dest_mohm = (uint16_t)(((uint32_t)raw * 1000U) / 4096U);
}

/**
 * @brief   Reads an MAX17205 time value from a register to
 *          seconds between 0S and 368,634S.
 */
static void convert_time(uint16_t raw, struct sensor_value *valp)
{
    // Reference datasheet table 1: Time LSB is 5.625s, unsigned.
    valp->val1 = (uint32_t)raw * 5625U / 1000U;
    valp->val2 = 0;
}

static int16_t convert_learn_state(uint16_t raw, struct sensor_value *valp)
{
    valp->val1 = _FLD2VAL(MAX17205_LEARNCFG_LS, raw) & 0x0FFFFU;
    valp->val2 = 0;
}

static uint16_t encode_learn_state(uint8_t state)
{
    return MAX17205_SETVAL(MAX17205_AD_NLEARNCFG, _VAL2FLD(MAX17205_LEARNCFG_LS, state));
}

/**
 * The MAX17205 allows for the NV ram to be written no more then 7 times on a single chip.
 * Each time, an additional bit is set in a flash register which can be queried. 
 * See page 85 of the data sheet.
 */
static int max17205_read_writes_remaining(const struct device *dev, uint32_t *number_of_writes_left)
{
    int rc;
    uint16_t buf;

    LOG_DBG("Reading NV Masking register...");

    rc = max17205_reg_write(dev, MAX17205_AD_COMMAND, 0xE2FA);
    if (rc ) {
        return rc;
    }

    k_msleep(MAX17205_T_RECAL_MS);

    rc = max17205_reg_read(dev, 0x1ED, &buf);
    if (rc) {
        return rc;
    }

    *number_of_writes_left = 7;
    for (uint8_t i = 0; i < 8; i++) {
        if ((buf & (1 << i)) && (*number_of_writes_left > 0)) {
            *number_of_writes_left--;
        }
    }

    return 0;
}

/**
 * See page 85 of the data sheet
 */
static int max17205_nv_program(const struct device *dev) {
    int rc;
    uint16_t buf = 0;

    //  1. Write desired memory locations to new values.
    //should be done prior to calling this function

    LOG_DBG("Clearing CommStat.NVError bit");
    //  2. Clear CommStat.NVError bit.
    rc = max17205_reg_write(dev, MAX17205_AD_COMMSTAT, MAX17205_COMMSTAT_NVERROR);
    if (rc) {
        LOG_ERR("Error cleanring NVError bit: %d", rc);
        return rc;
    }

    //  3. Write 0xE904 to the Command register 0x060 to initiate a block copy.
    rc = max17205_reg_write(dev, MAX17205_AD_COMMAND, 0xE904);
    if (rc) {
        LOG_ERR("Failed to send command to initiate block copy: %d", rc);
        return rc;
    }
    LOG_DBG("Initiated MAX17205 block copy....");

    //  4. Wait t BLOCK for the copy to complete.
    LOG_DBG("Waiting %u ms for block copy to complete....", MAX17205_T_BLOCK_MS);
    // tBlock(max) is specified as 7360ms in the data sheet, page 16
    k_msleep(MAX17205_T_BLOCK_MS);

    //  5. Check the CommStat.NVError bit. If set, repeat the process. If clear, continue.
    rc = max17205_reg_read(dev, MAX17205_AD_COMMSTAT, &buf);
    if (rc) {
        LOG_ERR("Failed to query COMMSTAT register: %d", rc);
        return rc;
    }

    LOG_DBG("Read MAX17205_AD_COMMSTAT register 0x%X as 0x%X", MAX17205_AD_COMMSTAT, buf);
    if (buf & MAX17205_COMMSTAT_NVERROR) {
        //Error bit is set
        LOG_ERR("Block copy failed.");
        return -EIO;
    }
    LOG_DBG("Block copy successful");

    //  6. Write 0x000F to the Command register 0x060 to POR the IC.
    //  7. Wait t POR for the IC to reset.
    //  8. Write 0x0001 to Counter Register 0x0BB to reset firmware.
    //  9. Wait t POR for the firmware to restart.
    LOG_DBG("Resetting MAX17205...");
    if ((rc = max17205_hardware_reset(dev)) != 0) {
        LOG_ERR("Failed to reset MAX17205: %d", rc);
        return rc;
    }

    return 0;
}

/**
 * @brief   Firmware reset the chip. Allows it to utilize any
 *          changes written to the shadow RAM register.
 *
 * @param[in] devp      pointer to the @p MAX17205Driver object
 *
 * @api
 */
int max17205_firmware_reset(const struct device *dev) {

    int rc;

    LOG_DBG("Performing MAX17205 firmware reset");
    // Now firmware-reset the chip so it will use the values written to various registers
    rc = max17205_reg_write(dev, MAX17205_AD_CONFIG2, MAX17205_CONFIG2_POR_CMD);
    k_msleep(MAX17205_T_POR_MS);

    return rc;
}

/**
 * @brief   Hardware reset the chip. Reloads nonvolatile
 *          registers into shadow RAM.
 *
 * @param[in] devp      pointer to the @p MAX17205Driver object
 *
 * @api
 */
int max17205_hardware_reset(const struct device *dev) {
    int rc;

    rc = max17205_reg_write(dev, MAX17205_AD_STATUS, 0);
    if (rc) {
        LOG_ERR("Unable to clear POR status bit: %d", rc);
    }

    LOG_DBG("Performing MAX17205 hardware reset");
    rc = max17205_reg_write(dev, MAX17205_AD_COMMAND, MAX17205_COMMAND_HARDWARE_RESET);
    if (rc) {
        LOG_ERR("Failed to write hardware reset command: %d", rc);
        return rc;
    }

    k_msleep(MAX17205_T_POR_MS);

    // Not sure this polling loop is needed; datasheet only mentions waiting tPOR.
    uint32_t check_count = 0;
    uint16_t status = 0;
    do {
        rc = max17205_reg_read(dev, MAX17205_AD_STATUS, &status);
        if (rc) {
            LOG_ERR("Failed to read status after hardware reset: %d", rc);
            return rc;
        }
        k_msleep(1);
        check_count++;
    } while (!(status & MAX17205_STATUS_POR) && check_count < 20); /* While still resetting */

    if (!(status & MAX17205_STATUS_POR)) {
        LOG_ERR("POR bit is not yet set. Timing out.");
        return -ETIMEDOUT;
    }

    rc = max17205_reg_write(dev, MAX17205_AD_STATUS, 0);
    if (rc) {
        LOG_ERR("Unable to clear POR status bit: %d", rc);
    }

    rc = max17205_firmware_reset(dev);
    if (rc) {
        LOG_ERR("Failed to perform firmware reset: %d", rc);
        return rc;
    }

    // Not sure this polling loop is needed; datasheet only mentions waiting tPOR.
    check_count = 0;
    status = 0;
    do {
        rc = max17205_reg_read(dev, MAX17205_AD_STATUS, &status);
        if (rc) {
            LOG_ERR("Failed to read status after firmware reset: %d", rc);
            return rc;
        }
        k_msleep(1);
        check_count++;
    } while (!(status & MAX17205_STATUS_POR) && check_count < 20); /* While still resetting */

    if (!(status & MAX17205_STATUS_POR)) {
        LOG_ERR("POR bit is not yet set. Timing out.");
        return -ETIMEDOUT;
    }

    return 0;
}

/**
 * @brief Convert raw register values for specific channel
 *
 * @param dev MAX17205 device to access
 * @param chan Channel number to read
 * @param valp Returns the sensor value read on success
 * @return 0 if successful
 * @return -ENOTSUP for unsupported channels
 */
static int max17205_channel_get(const struct device *dev, enum sensor_channel chan,
                                struct sensor_value *valp)
{
    const struct max17205_config *const config = dev->config;
    struct max17205_data *const data = dev->data;
    int16_t raw;

    switch (chan) {
    case MAX17205_CHAN_TEMP_1:
        raw = data->temp_1;
        convert_average_temperature(raw, valp);
        break;
    case MAX17205_CHAN_TEMP_2:
        raw = data->temp_2;
        convert_average_temperature(raw, valp);
        break;
    case SENSOR_CHAN_GAUGE_TEMP:
        raw = data->int_temp;
        convert_average_temperature(raw, valp);
        break;
    case MAX17205_CHAN_AVG_TEMP_1:
        raw = data->avg_temp_1;
        convert_average_temperature(raw, valp);
        break;
    case MAX17205_CHAN_AVG_TEMP_2:
        raw = data->avg_temp_2;
        convert_average_temperature(raw, valp);
        break;
    case MAX17205_CHAN_AVG_INT_TEMP:
        raw = data->avg_int_temp;
        convert_average_temperature(raw, valp);
        break;
    case MAX17205_CHAN_TEMP_MAX:
        raw = data->temp_max_C;
        convert_max_temperature(raw, valp);
        break;
    case MAX17205_CHAN_TEMP_MIN:
        raw = data->temp_min_C;
        convert_min_temperature(raw, valp);
        break;
    case MAX17205_CHAN_V_CELL_1:
        raw = data->v_cell_1;
        convert_voltage(raw, valp);
        break;
    case MAX17205_CHAN_V_CELL_2:
        raw = data->v_cell_2;
        convert_voltage(raw, valp);
        break;
    case MAX17205_CHAN_V_CELL_AVG:
        raw = data->v_cell_avg;
        convert_voltage(raw, valp);
        break;
    case MAX17205_CHAN_V_CELL:
        raw = data->v_cell;
        convert_voltage(raw, valp);
        break;
    case SENSOR_CHAN_GAUGE_VOLTAGE:
        raw = data->v_batt;
        convert_batt_voltage(raw, valp);
        break;
    case MAX17205_CHAN_V_CELL_MAX:
        raw = data->v_cell_max;
        convert_max_voltage(raw, valp);
        break;
    case MAX17205_CHAN_V_CELL_MIN:
        raw = data->v_cell_min;
        convert_min_voltage(raw, valp);
        break;
    case SENSOR_CHAN_CURRENT:
        raw = data->current;
        convert_current(raw, valp);
        break;
    case SENSOR_CHAN_GAUGE_AVG_CURRENT:
        raw = data->avg_current;
        convert_current(raw, valp);
        break;
    case MAX17205_CHAN_CURRENT_MAX:
        raw = data->current_max;
        convert_max_current(config->rsense_mohms, raw, valp);
        break;
    case MAX17205_CHAN_CURRENT_MIN:
        raw = data->current_min;
        convert_min_current(config->rsense_mohms, raw, valp);
        break;
    case SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY:
        raw = data->full_cap;
        convert_capacity(config->rsense_mohms, raw, valp);
        break;
    case SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY:
        raw = data->available_cap;
        convert_capacity(config->rsense_mohms, raw, valp);
        break;
    case MAX17205_CHAN_MIX_CAPACITY:
        raw = data->mix_cap;
        convert_capacity(config->rsense_mohms, raw, valp);
        break;
    case SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY:
        raw = data->reported_cap;
        convert_capacity(config->rsense_mohms, raw, valp);
        break;
    case SENSOR_CHAN_GAUGE_TIME_TO_EMPTY:
        raw = data->time_to_empty;
        convert_time(raw, valp);
        break;
    case SENSOR_CHAN_GAUGE_TIME_TO_FULL:
        raw = data->time_to_full;
        convert_time(raw, valp);
        break;
    case MAX17205_CHAN_AVAILABLE_SOC:
        raw = data->available_soc;
        convert_percentage(raw, valp);
        break;
    case MAX17205_CHAN_PRESENT_SOC:
        raw = data->present_soc;
        convert_percentage(raw, valp);
        break;
    case MAX17205_CHAN_REPORTED_SOC:
        raw = data->reported_soc;
        convert_percentage(raw, valp);
        break;
    case SENSOR_CHAN_GAUGE_CYCLE_COUNT:
        raw = data->cycle_count;
        convert_cycles(raw, valp);
        break;
    case MAX17205_CHAN_LEARN_STAGE:
        raw = data->learn_stage;
        convert_learn_state(raw, valp);
    default:
        LOG_ERR("Unsupported channel %d!", chan);
        return -ENOTSUP;
    }

    return 0;
}

/**
 * @brief Read register values for supported channels
 *
 * @param dev MAX17205 device to access
 * @return 0 if successful, or negative error code from I2C API
 */
static int max17205_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct max17205_data *data = dev->data;
    int rc;
    int reg_addr;
    int16_t *dest;

    switch (chan) {
    case MAX17205_CHAN_TEMP_1:
        reg_addr = MAX17205_AD_TEMP1;
        dest = &data->temp_1;
        break;
    case MAX17205_CHAN_TEMP_2:
        reg_addr = MAX17205_AD_TEMP2;
        dest = &data->temp_2;
        break;
    case SENSOR_CHAN_GAUGE_TEMP:
        reg_addr = MAX17205_AD_INTTEMP;
        dest = &data->int_temp;
        break;
    case MAX17205_CHAN_AVG_TEMP_1:
        reg_addr = MAX17205_AD_AVGTEMP1;
        dest = &data->avg_temp_1;
        break;
    case MAX17205_CHAN_AVG_TEMP_2:
        reg_addr = MAX17205_AD_AVGTEMP2;
        dest = &data->avg_temp_2;
        break;
    case MAX17205_CHAN_AVG_INT_TEMP:
        reg_addr = MAX17205_AD_AVGINTTEMP;
        dest = &data->avg_int_temp;
        break;
    case MAX17205_CHAN_TEMP_MAX:
        reg_addr = MAX17205_AD_MAXMINTEMP;
        dest = &data->temp_max_C;
        break;
    case MAX17205_CHAN_TEMP_MIN:
        reg_addr = MAX17205_AD_MAXMINTEMP;
        dest = &data->temp_min_C;
        break;
    case MAX17205_CHAN_V_CELL_1:
        reg_addr = MAX17205_AD_AVGCELL1;
        dest = &data->v_cell_1;
        break;
    case MAX17205_CHAN_V_CELL_2:
        {
            rc = max17205_reg_read(dev, MAX17205_AD_BATT, &data->v_batt);
            if (!rc) {
                rc = max17205_reg_read(dev, MAX17205_AD_AVGCELL1, &data->v_cell_1);
                if (!rc) {
                    data->v_cell_2 = data->v_batt - data->v_cell_1;
                }
            }
            return rc;
        }
    case MAX17205_CHAN_V_CELL_AVG:
        reg_addr = MAX17205_AD_AVGVCELL;
        dest = &data->v_cell_avg;
        break;
    case MAX17205_CHAN_V_CELL:
        reg_addr = MAX17205_AD_VCELL;
        dest = &data->v_cell;
        break;
    case SENSOR_CHAN_GAUGE_VOLTAGE:
        reg_addr = MAX17205_AD_BATT;
        dest = &data->v_batt;
        break;
    case MAX17205_CHAN_V_CELL_MAX:
        reg_addr = MAX17205_AD_MAXMINVOLT;
        dest = &data->v_cell_max;
        break;
    case MAX17205_CHAN_V_CELL_MIN:
        reg_addr = MAX17205_AD_MAXMINVOLT;
        dest = &data->v_cell_min;
        break;
    case SENSOR_CHAN_CURRENT:
        reg_addr = MAX17205_AD_CURRENT;
        dest = &data->current;
        break;
    case SENSOR_CHAN_GAUGE_AVG_CURRENT:
        reg_addr = MAX17205_AD_AVGCURRENT;
        dest = &data->avg_current;
        break;
    case MAX17205_CHAN_CURRENT_MAX:
        reg_addr = MAX17205_AD_MAXMINCURR;
        data = &data->current_max;
        break;
    case MAX17205_CHAN_CURRENT_MIN:
        reg_addr = MAX17205_AD_MAXMINCURR;
        data = &data->current_min;
        break;
    case SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY:
        reg_addr = MAX17205_AD_FULLCAPREP;
        data = &data->full_cap;
        break;
    case SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY:
        reg_addr = MAX17205_AD_AVCAP;
        data = &data->available_cap;
        break;
    case MAX17205_CHAN_MIX_CAPACITY:
        reg_addr = MAX17205_AD_MIXCAP;
        data = &data->mix_cap;
        break;
    case SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY:
        reg_addr = MAX17205_AD_REPCAP;
        data = &data->reported_cap;
        break;
    case SENSOR_CHAN_GAUGE_TIME_TO_EMPTY:
        reg_addr = MAX17205_AD_TTE;
        data = &data->time_to_empty;
        break;
    case SENSOR_CHAN_GAUGE_TIME_TO_FULL:
        reg_addr = MAX17205_AD_TTF;
        data = &data->time_to_full;
        break;
    case MAX17205_CHAN_AVAILABLE_SOC:
        reg_addr = MAX17205_AD_AVSOC;
        data = &data->available_soc;
        break;
    case MAX17205_CHAN_PRESENT_SOC:
        reg_addr = MAX17205_AD_VFSOC;
        data = &data->present_soc;
        break;
    case MAX17205_CHAN_REPORTED_SOC:
        reg_addr = MAX17205_AD_REPSOC;
        data = &data->reported_soc;
        break;
    case SENSOR_CHAN_GAUGE_CYCLE_COUNT:
        reg_addr = MAX17205_AD_CYCLES;
        data = &data->cycle_count;
        break;
    case MAX17205_CHAN_LEARN_STAGE:
        reg_addr = MAX17205_AD_LEARNCFG;
        data = &data->learn_stage;
    default:
        LOG_ERR("Unknown channel: %d", chan);
        return -ENOTSUP;
    }
    rc = max17205_reg_read(dev, reg_addr, dest);
    if (rc != 0) {
        LOG_ERR("Failed to read channel %d", chan);
        return rc;
    }

    return 0;
}

static int wait_for_data_ready(const struct device *dev)
{
    int rc;
    int16_t tmp;

    rc = max17205_reg_read(dev, MAX17205_AD_FSTAT, &tmp);
    if (rc) {
        return rc;
    }

    /* Do not continue until FSTAT.DNR bit is cleared */
    while (tmp & MAX17205_FSTAT_DNR) {
        k_msleep(10);
        rc = max17205_reg_read(dev, MAX17205_AD_FSTAT, &tmp);
        if (rc) {
            return rc;
        }
    }
    return 0;
}

static int max17205_attr_set(const struct device *dev,
                             enum sensor_channel chan,
                             enum sensor_attribute attr,
                             const struct sensor_value *val)
{
    struct max17205_config *config = (struct max17205_config *)dev->config;
    int16_t raw;
    int rc = 0;

    switch (attr) {
    case MAX17205_ATTR_HW_RESET:
        rc = max17205_hardware_reset(dev);
        break;
    case MAX17205_ATTR_FW_RESET:
        rc = max17205_firmware_reset(dev);
        break;
    case MAX17205_ATTR_CAPACITY:
        {
            uint16_t reg_addr;

            raw = encode_capacity(config->rsense_mohms, val->val1);

            switch (chan) {
            case SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY:
                reg_addr = MAX17205_AD_FULLCAPREP;
                break;
            case SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY:
                reg_addr = MAX17205_AD_AVCAP;
                break;
            case MAX17205_CHAN_MIX_CAPACITY:
                reg_addr = MAX17205_AD_MIXCAP;
                break;
            case SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY:
                reg_addr = MAX17205_AD_REPCAP;
                break;
            default:
                rc = -EINVAL;
            }
            if (!rc) {
                rc = max17205_reg_write(dev, reg_addr, raw);
            }
        }
        break;
    case MAX17205_ATTR_LEARN_STAGE:
        raw = encode_learn_state(val->val1);
        rc = max17205_reg_write(dev, MAX17205_AD_NLEARNCFG, raw);
        break;
    case MAX17205_ATTR_NV_BLOCK_PROGRAM:
        rc = max17205_nv_program(dev);
        break;
    default:
        if ((unsigned int)attr >= MAX17205_ATTR_REGS) {
            uint16_t reg_addr = (unsigned int)attr - MAX17205_ATTR_REGS;

            if (reg_addr > MAX17205_AD_MAXVALUE) {
                return -EINVAL;
            }
            rc = max17205_reg_write(dev, reg_addr, val->val1 & 0x0ffffU);
        } else {
            rc = -ENOTSUP;
        }
    }
    return rc;
}

static int max17205_attr_get(const struct device *dev,
                             enum sensor_channel chan,
                             enum sensor_attribute attr,
                             struct sensor_value *val)
{
    const struct max17205_config *const config = dev->config;
    int rc;

    switch (attr) {
    case MAX17205_ATTR_CAPACITY:
        {
            switch (chan) {
            case SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY:
            case SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY:
            case MAX17205_CHAN_MIX_CAPACITY:
            case SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY:
                rc = max17205_sample_fetch(dev, chan)
                if (!rc) {
                    rc = max17205_channel_get(dev, chan, val);
                }
                break;
            }
        }
        break;
    case MAX17205_ATTR_LEARN_STAGE:
        chan = MAX17205_CHAN_LEARN_STAGE;
        rc = max17205_sample_fetch(dev, chan)
        if (!rc) {
            rc = max17205_channel_get(dev, chan, val);
        }
        break;
    case MAX17205_ATTR_NV_WRITES_LEFT:
        rc = max17205_read_writes_remaining(dev, &val->val1);
        val->val2 = 0;
        break;
    default:
        if ((unsigned int)attr >= MAX17205_ATTR_REGS) {
            uint16_t reg_addr = (unsigned int)attr - MAX17205_ATTR_REGS;
            int16_t rd_val;

            if (reg_addr <= MAX17205_AD_MAXVALUE) {
                rc = max17205_reg_read(dev, reg_addr, &rd_val);
                val->val1 = ((int32_t)rd_val & 0x0ffffU);
                val->val2 = 0;
            } else {
                rc = -EINVAL;
            }
        } else {
            rc = -ENOTSUP;
        }
    }
    return rc;
}

/**
 * @brief Initialise the fuel gauge
 *
 * @param dev MAX17205 device to access
 * @return 0 for success
 * @return -EINVAL if the I2C controller could not be found
 */
static int max17205_init(const struct device *dev)
{
    /* Kludge below: we are overriding the normally const dev->config so we can
     * modify the i2c_aux address to our secondary slave address.
     */
    struct max17205_config *config = (struct max17205_config *)dev->config;
    int16_t tmp;
    int rc;

    k_msleep(MAX17205_T_POR_MS);

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("Bus device is not ready");
        return -ENODEV;
    }

    /* We later will chose which device to use based on the register address:
     * config->i2c or config->i2c_aux.
     */
    config->i2c_aux.addr = config->aux_addr;

    /* Read Status register */
    rc = max17205_reg_read(dev, MAX17205_AD_STATUS, &tmp);
    if (rc) {
        return rc;
    }

    if (!(tmp & MAX17205_STATUS_POR)) {
        /*
         * Status.POR bit is set to 1 when MAX17205 detects that
         * a software or hardware POR event has occurred and
         * therefore a custom configuration needs to be set...
         * If POR event did not happen (Status.POR == 0), skip
         * init and continue with measurements.
         */
        LOG_DBG("No POR event detected - skip device configuration");
        return 0;
    }
    LOG_DBG("POR detected, setting custom device configuration...");

    const struct bal {
        uint16_t mv,
        uint16_t val
    } const bal_table[] = {
        {2, MAX17205_PACKCFG_BALCFG_2_5},
        {5, MAX17205_PACKCFG_BALCFG_5},
        {10, MAX17205_PACKCFG_BALCFG_10},
        {20, MAX17205_PACKCFG_BALCFG_20},
        {40, MAX17205_PACKCFG_BALCFG_40},
        {80, MAX17205_PACKCFG_BALCFG_80},
        {160, MAX17205_PACKCFG_BALCFG_160},
        {0, 0}
    };
    int i;
    uint16_t bal_val = 0;

    for (i = 0; bal_table[i].mv != 0; i++) {
        if (config->cell_bal_thresh_voltage <= bal_table[i].mv) {
            bal_val = bal_table[i].val;
            break;
        }
    }

    uint16_t packcfg = _VAL2FLD(MAX17205_PACKCFG_NCELLS, config->num_cells) |
                       bal_val | config->pack_flags;

    rc = max17205_reg_write(dev, MAX17205_AD_NPACKCFG, packcfg);
    if (rc) {
        return rc;
    }

    rc = max17205_reg_write(dev, MAX17205_AD_NRSENSE, config->rsense_mohms);
    if (rc) {
        return rc;
    }

    rc = max17205_firmware_reset(dev);
    if (rc) {
        return rc;
    }

    return 0;
}

static DEVICE_API(sensor, max17205_battery_driver_api) = {
    .attr_set = max17205_attr_set, 
    .attr_get = max17205_attr_get,
    .sample_fetch = max17205_sample_fetch,
    .channel_get = max17205_channel_get,
};

#define MAX17205_INIT(n)                                                                           \
    static struct max17205_data max17205_data_##n;                                             \
                                                                                                   \
    static const struct max17205_config max17205_config_##n = {                                \
        .i2c = I2C_DT_SPEC_INST_GET(n),
        .i2c_aux = I2C_DT_SPEC_INST_GET(n), /* Slave addr changed in init */
        .pack_flags = DT_INST_PROP(n, pack_flags),
        .num_cells = DT_INST_PROP(n, num_cells),
        .cel_bal_thresh_voltage = DT_INST_PROP(n, cel_bal_thresh_voltage),
        .rsense_mohms = DT_INST_PROP(n, rsense_mohms),
    };                                                                                         \
                                                                                                   \
    SENSOR_DEVICE_DT_INST_DEFINE(n, &max17205_init, NULL, &max17205_data_##n,            \
                     &max17205_config_##n, POST_KERNEL,                            \
                     CONFIG_SENSOR_INIT_PRIORITY, &max17205_battery_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX17205_INIT)
