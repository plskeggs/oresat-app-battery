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
static int max17205_reg_read(const struct device *dev, uint8_t reg_addr, int16_t *valp)
{
    const struct max17205_config *cfg = dev->config;
    uint8_t i2c_data[2];
    int rc;

    rc = i2c_burst_read_dt(&cfg->i2c, reg_addr, i2c_data, 2);
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
static int max17205_reg_write(const struct device *dev, uint8_t reg_addr, int16_t val)
{
    const struct max17205_config *cfg = dev->config;
    uint8_t i2c_data[3] = {reg_addr, val & 0xFF, (uint16_t)val >> 8};

    return i2c_write_dt(&cfg->i2c, i2c_data, sizeof(i2c_data));
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
    int32_t tmp;

    switch (chan) {
    case SENSOR_CHAN_GAUGE_VOLTAGE:
        /* Get voltage in uV */
        tmp = data->voltage * VOLTAGE_MULTIPLIER_UV;
        /* Convert to V */
        valp->val1 = tmp / 1000000;
        valp->val2 = tmp % 1000000;
        break;
    case SENSOR_CHAN_GAUGE_AVG_CURRENT: {
        int current;
        /* Get avg current in nA */
        current = data->avg_current * CURRENT_MULTIPLIER_NA;
        /* Convert to mA */
        valp->val1 = current / 1000000;
        valp->val2 = current % 1000000;
        break;
    }

    /** Standby current, in amps (negative=discharging) **/
    case SENSOR_CHAN_GAUGE_STDBY_CURRENT:
        break;
    /** Max load current, in amps (negative=discharging) **/
    case SENSOR_CHAN_GAUGE_MAX_LOAD_CURRENT:
        break;

    case SENSOR_CHAN_GAUGE_TEMP:
        valp->val1 = data->internal_temp / 256;
        valp->val2 = data->internal_temp % 256 * 1000000 / 256;
        break;
    case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
        valp->val1 = data->state_of_charge / 256;
        valp->val2 = data->state_of_charge % 256 * 1000000 / 256;
        break;
    case SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY:
        convert_millis(valp, data->full_cap);
        break;
    case SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY:
        convert_millis(valp, data->remaining_cap);
        break;
    case SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY:
        convert_millis(valp, data->design_cap);
        break;

	/** Full Available Capacity in mAh **/
    case SENSOR_CHAN_GAUGE_FULL_AVAIL_CAPACITY:
        break;
    /** Average power in mW **/
    case SENSOR_CHAN_GAUGE_AVG_POWER:
        break;
    /** State of health measurement in % **/
    case SENSOR_CHAN_GAUGE_STATE_OF_HEALTH:
        break;

    case SENSOR_CHAN_GAUGE_TIME_TO_EMPTY:
        /* Get time in ms */
        if (data->time_to_empty == 0xffff) {
            valp->val1 = 0;
            valp->val2 = 0;
        } else {
            tmp = data->time_to_empty * TIME_MULTIPLIER_MS;
            convert_millis(valp, tmp);
        }
        break;
    case SENSOR_CHAN_GAUGE_TIME_TO_FULL:
        /* Get time in ms */
        if (data->time_to_full == 0xffff) {
            valp->val1 = 0;
            valp->val2 = 0;
        } else {
            tmp = data->time_to_full * TIME_MULTIPLIER_MS;
            convert_millis(valp, tmp);
        }
        break;
    case SENSOR_CHAN_GAUGE_CYCLE_COUNT:
        valp->val1 = data->cycle_count / 100;
        valp->val2 = data->cycle_count % 100 * 10000;
        break;
    case SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE:
        convert_millis(valp, config->design_voltage);
        break;
    case SENSOR_CHAN_GAUGE_DESIRED_VOLTAGE:
        convert_millis(valp, config->desired_voltage);
        break;
    case SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT:
        valp->val1 = data->ichg_term;
        valp->val2 = 0;
        break;
    case MAX17205_COULOMB_COUNTER:
        /* Get spent capacity in mAh */
        data->coulomb_counter = 0xffff - data->coulomb_counter;
        valp->val1 = data->coulomb_counter / 2;
        valp->val2 = data->coulomb_counter % 2 * 10 / 2;
        break;
    default:
        LOG_ERR("Unsupported channel!");
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
        reg_addr = MAX17205_AD_CELL1;
        break;
    case MAX17205_CHAN_V_CELL_2:
        reg_addr = MAX17205_AD_BATT; // post processing uses batt and cell1 to compute cell2
        dest = &data->v_cell_2;
        break;
    case MAX17205_CHAN_V_CELL_AVG:
        reg_addr = MAX17205_AD_AVGVCELL;
        dest = &data->v_cell_avg;
        break;
    case MAX17205_CHAN_V_CELL:
        reg_addr = MAX17205_AD_VCELL;
        dest = &data->v_cell;
        break;
    case SENSOR_CHAN_GAUGE_VOLTAGE:
        reg_addr = MAX17205_AD_BATT; // post processing uses batt and cell1 to compute cell2
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

/**
 * @brief Initialise the fuel gauge
 *
 * @param dev MAX17205 device to access
 * @return 0 for success
 * @return -EINVAL if the I2C controller could not be found
 */
static int max17205_init(const struct device *dev)
{
    const struct max17205_config *const config = dev->config;
    int16_t tmp, hibcfg;
    int rc;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("Bus device is not ready");
        return -ENODEV;
    }

    /* Read Status register */
    rc = max17205_reg_read(dev, STATUS, &tmp);
    if (rc) {
        return rc;
    }

    if (!(tmp & STATUS_POR)) {
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

    /** STEP 1 */
    rc = max17205_reg_read(dev, FSTAT, &tmp);
    if (rc) {
        return rc;
    }

    /* Do not continue until FSTAT.DNR bit is cleared */
    while (tmp & FSTAT_DNR) {
        k_sleep(K_MSEC(10));
        rc = max17205_reg_read(dev, FSTAT, &tmp);
        if (rc) {
            return rc;
        }
    }

    /** STEP 2 */
    /* Store original HibCFG value */
    rc = max17205_reg_read(dev, HIBCFG, &hibcfg);
    if (rc) {
        return rc;
    }

    /* Exit Hibernate Mode step 1 */
    rc = max17205_reg_write(dev, SOFT_WAKEUP, 0x0090);
    if (rc) {
        return rc;
    }

    /* Exit Hibernate Mode step 2 */
    rc = max17205_reg_write(dev, HIBCFG, 0x0000);
    if (rc) {
        return rc;
    }

    /* Exit Hibernate Mode step 3 */
    rc = max17205_reg_write(dev, SOFT_WAKEUP, 0x0000);
    if (rc) {
        return rc;
    }

    /** STEP 2.1 --> OPTION 1 EZ Config (No INI file is needed) */
    /* Write DesignCap */
    rc = max17205_reg_write(dev, DESIGN_CAP, config->design_cap);
    if (rc) {
        return rc;
    }

    /* Write IChgTerm */
    rc = max17205_reg_write(dev, ICHG_TERM, config->desired_charging_current);
    if (rc) {
        return rc;
    }

    /* Write VEmpty */
    rc = max17205_reg_write(dev, VEMPTY,
                ((config->empty_voltage / 10) << 7) |
                    ((config->recovery_voltage / 40) & 0x7F));
    if (rc) {
        return rc;
    }

    /* Write ModelCFG */
    if (config->charge_voltage > 4275) {
        rc = max17205_reg_write(dev, MODELCFG, 0x8400);
    } else {
        rc = max17205_reg_write(dev, MODELCFG, 0x8000);
    }

    if (rc) {
        return rc;
    }

    /*
     * Read ModelCFG.Refresh (highest bit),
     * proceed to Step 3 when ModelCFG.Refresh == 0
     */
    rc = max17205_reg_read(dev, MODELCFG, &tmp);
    if (rc) {
        return rc;
    }

    /* Do not continue until ModelCFG.Refresh == 0 */
    while (tmp & MODELCFG_REFRESH) {
        k_sleep(K_MSEC(10));
        rc = max17205_reg_read(dev, MODELCFG, &tmp);
        if (rc) {
            return rc;
        }
    }

    /* Restore Original HibCFG value */
    rc = max17205_reg_write(dev, HIBCFG, hibcfg);
    if (rc) {
        return rc;
    }

    /** STEP 3 */
    /* Read Status register */
    rc = max17205_reg_read(dev, STATUS, &tmp);
    if (rc) {
        return rc;
    }

    /* Clear PowerOnReset bit */
    tmp &= ~STATUS_POR;
    rc = max17205_reg_write(dev, STATUS, tmp);
    if (rc) {
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
    k_sleep(K_MSEC(MAX17205_T_POR_MS));

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

    k_sleep(K_MSEC(MAX17205_T_POR_MS));

    // Not sure this polling loop is needed; datasheet only mentions waiting tPOR.
    uint32_t check_count = 0;
    uint16_t status = 0;
    do {
        rc = max17205_reg_read(dev, MAX17205_AD_STATUS, &status);
        if (rc) {
            LOG_ERR("Failed to read status after hardware reset: %d", rc);
            return rc;
        }
        k_sleep(K_MSEC(1));
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
        k_sleep(K_MSEC(1));
        check_count++;
    } while (!(status & MAX17205_STATUS_POR) && check_count < 20); /* While still resetting */

    if (!(status & MAX17205_STATUS_POR)) {
        LOG_ERR("POR bit is not yet set. Timing out.");
        return -ETIMEDOUT;
    }

    return 0;
}

static DEVICE_API(sensor, max17205_battery_driver_api) = {
    .sample_fetch = max17205_sample_fetch,
    .channel_get = max17205_channel_get,
};

#define MAX17205_INIT(n)                                                                           \
    static struct max17205_data max17205_data_##n;                                             \
                                                                                                   \
    static const struct max17205_config max17205_config_##n = {                                \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
        .design_voltage = DT_INST_PROP(n, design_voltage),                                 \
        .desired_voltage = DT_INST_PROP(n, desired_voltage),                               \
        .desired_charging_current = DT_INST_PROP(n, desired_charging_current),             \
        .design_cap = DT_INST_PROP(n, design_cap),                                         \
        .empty_voltage = DT_INST_PROP(n, empty_voltage),                                   \
        .recovery_voltage = DT_INST_PROP(n, recovery_voltage),                             \
        .charge_voltage = DT_INST_PROP(n, charge_voltage),                                 \
    };                                                                                         \
                                                                                                   \
    SENSOR_DEVICE_DT_INST_DEFINE(n, &max17205_init, NULL, &max17205_data_##n,            \
                     &max17205_config_##n, POST_KERNEL,                            \
                     CONFIG_SENSOR_INIT_PRIORITY, &max17205_battery_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX17205_INIT)
