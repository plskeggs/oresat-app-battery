#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

#include "calib.h"
#include "batt.h"

TODO:
- replace DEBUG_PRINT, ENABLE_NV_WRITE_PROMPT, ENABLE_LEARN_COMPLETE with Kconfigs

LOG_MODULE_REGISTER(max17205, CONFIG_SENSOR_LOG_LEVEL);

int max17205_reg_read(const struct device *dev, uint16_t addr, int16_t *val)
{
    struct sensor_value sensor_val;
    int rc;

    rc = sensor_attr_get(dev, 0, MAX17205_ATTR_REGS + addr, &sensor_val);
    if (!rc) {
        *val = sensor_val.val1;
    }
    return rc;
}

int max17205_reg_write(const struct device *dev, uint16_t addr, int16_t val)
{
    struct sensor_value sensor_val = {.val1 = (uint16_t)val, .val2 = 0};

    return sensor_attr_set(dev, 0, MAX17205_ATTR_REGS + addr, &sensor_val);
}

static int max17205_firmware_reset(const struct device *dev)
{
    return sensor_attr_set(dev, 0, MAX17205_ATTR_FW_RESET, NULL);
}

static int max17205_hardware_reset(const struct device *dev)
{
    return sensor_attr_set(dev, 0, MAX17205_ATTR_HW_RESET, NULL);
}

static int max17205_read_writes_remaining(const struct device *dev, uint8_t *num_left)
{
    struct sensor_value sensor_val;
    int rc;

    rc = sensor_attr_get(dev, 0, MAX17205_ATTR_NV_WRITES_LEFT, &sensor_val);
    if (!rc) {
        *num_left = sensor_val.val1 & 0xFFU;
    }
    return rc;
}

static int max17205_nv_program(const struct device *dev)
{
    return sensor_attr_set(dev, 0, MAX17205_ATTR_NV_BLOCK_PROGRAM, NULL);
}

static int max17205_read_learn_stage(const struct device *dev, uint16_t *stage)
{
    struct sensor_value sensor_val;
    int rc;

    rc = sensor_attr_get(dev, 0, MAX17205_ATTR_LEARN_STAGE, &sensor_val);
    if (!rc) {
        *stage = sensor_val.val1 & 0xFFFFU;
    }
    return rc;
}

static int max17205_write_learn_stage(const struct device *dev, uint16_t stage)
{
    struct sensor_value sensor_val = {.val1 = stage, .val2 = 0};

    return sensor_attr_set(dev, 0, MAX17205_ATTR_LEARN_STAGE, &sensor_val);
}

#if 0
/* alternate way to read a capacity -- other is directly with chan as defined in calib.h */
int max17205_read_capacity(const struct device *dev, enum sensor_channel chan, uint32_t *cap)
{
    struct sensor_value sensor_val;

    return sensor_attr_get(dev, 0, MAX17205_ATTR_CAPACITY, &sensor_val);
}
#endif

static int max17205_write_capacity(const struct device *dev, enum sensor_channel chan, uint32_t cap)
{
    struct sensor_value sensor_val = {.val1 = cap, .val2 = 0};

    return sensor_attr_set(dev, 0, MAX17205_ATTR_CAPACITY, &sensor_val);
}

int read_channel_int8_t(const struct device *dev, enum sensor_channel type, int8_t *dest)
{
    struct sensor_value val;
    int rc = sensor_sample_fetch_chan(dev, type);

    if (!rc) {
        rc = sensor_channel_get(dev, type, &val);
        if (!rc) {
            *dest = val.val1 & 0xffU;
        }
    }
    return rc;
}

int read_channel_int16_t(const struct device *dev, enum sensor_channel type, int16_t *dest)
{
    struct sensor_value val;
    int rc = sensor_sample_fetch_chan(dev, type);

    if (!rc) {
        rc = sensor_channel_get(dev, type, &val);
        if (!rc) {
            *dest = val.val1 & 0xffffU;
        }
    }
    return rc;
}

int read_channel_int32_t(const struct device *dev, enum sensor_channel type, int32_t *dest)
{
    struct sensor_value val;
    int rc = sensor_sample_fetch_chan(dev, type);

    if (!rc) {
        rc = sensor_channel_get(dev, type, &val);
        if (!rc) {
            *dest = val.val1;
        }
    }
    return rc;
}

static int max17205_validate_registers(const struct device *dev, const max17205_regval_t * list, size_t len, bool * valid)
{
    bool matches = true;
    LOG_DBG("Current and expected NV settings:");
    for (size_t i = 0; i < len; ++i) {
        uint16_t buf = 0;
        int rc = max17205_reg_read(dev, list[i].reg, &buf);
        if (rc) {
            return rc;
        }
        if (buf != list[i].value) {
            LOG_WRN("   %-30s register 0x%04X is 0x%04X     NOT the expected  0x%04X",
                max17205_reg_to_str(list[i].reg), list[i].reg, buf, list[i].value);
           matches = false;
        } else {
            LOG_DBG("   %-30s register 0x%04X is 0x%04X     CORRECT",
                max17205_reg_to_str(list[i].reg), list[i].reg, buf);
        }
    }
    *valid = matches;
    return 0;
}

static int max17205_write_registers(const struct device *dev, const max17205_regval_t * list, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        int rc = max17205_reg_write(dev, list[i].reg, list[i].value);
        if (rc) {
            return rc;
        }
    }
    return 0;
}

int max17205_print_volatile_memory(const struct device *dev)
{
    int rc;

    LOG_DBG("MAX17205 *Volatile* Registers");
    uint16_t volatile_reg_list[] = {
        MAX17205_AD_PACKCFG,
        MAX17205_AD_DESIGNCAP,
        MAX17205_AD_LEARNCFG,
        MAX17205_AD_QRTABLE00,
        MAX17205_AD_QRTABLE10,
        MAX17205_AD_QRTABLE20,
        MAX17205_AD_QRTABLE30,
        MAX17205_AD_CYCLES,
        MAX17205_AD_FULLCAPNOM,
        MAX17205_AD_RCOMP0,
        MAX17205_AD_TEMPCO,
        MAX17205_AD_IAVGEMPTY,
        MAX17205_AD_FULLCAPREP,
        MAX17205_AD_MAXMINCURR,
        MAX17205_AD_MAXMINVOLT,
        MAX17205_AD_MAXMINTEMP,
        MAX17205_AD_MIXSOC,
        MAX17205_AD_VFSOC,
        MAX17205_AD_VEMPTY
    };

    for(size_t i = 0; i < ARRAY_LEN(volatile_reg_list); ++i) {
        uint16_t buf = 0;
        rc = max17205_reg_read(dev, volatile_reg_list[i], &buf);
        if (rc) {
            return rc;
        }
        LOG_DBG("   %-30s register 0x%04X is 0x%04X", max17205_reg_to_str(volatile_reg_list[i]), volatile_reg_list[i], buf);
    }
    return 0;
}

int max17205_print_nonvolatile_memory(const struct device *dev)
{
    uint8_t num_left = 0;
    int rc;

    rc = max17205_read_writes_remaining(dev, &num_left);
    if (rc) {
        return rc;
    }

    LOG_DBG("NV writes remaining: %u", num_left);

    // See table 19 on page 83 of the data sheet to see the list of non-volatile registers
    LOG_DBG("MAX17205 Non-Volatile Registers");
    uint16_t reg_list[] = {
        MAX17205_AD_NXTABLE0,
        MAX17205_AD_NXTABLE1,
        MAX17205_AD_NXTABLE2,
        MAX17205_AD_NXTABLE3,
        MAX17205_AD_NXTABLE4,
        MAX17205_AD_NXTABLE5,
        MAX17205_AD_NXTABLE6,
        MAX17205_AD_NXTABLE7,
        MAX17205_AD_NXTABLE8,
        MAX17205_AD_NXTABLE9,
        MAX17205_AD_NXTABLE10,
        MAX17205_AD_NXTABLE11,
        MAX17205_AD_NUSER18C,
        MAX17205_AD_NUSER18D,
        MAX17205_AD_NODSCTH,
        MAX17205_AD_NODSCCFG,
        MAX17205_AD_NOCVTABLE0,
        MAX17205_AD_NOCVTABLE1,
        MAX17205_AD_NOCVTABLE2,
        MAX17205_AD_NOCVTABLE3,
        MAX17205_AD_NOCVTABLE4,
        MAX17205_AD_NOCVTABLE5,
        MAX17205_AD_NOCVTABLE6,
        MAX17205_AD_NOCVTABLE7,
        MAX17205_AD_NOCVTABLE8,
        MAX17205_AD_NOCVTABLE9,
        MAX17205_AD_NOCVTABLE10,
        MAX17205_AD_NOCVTABLE11,
        MAX17205_AD_NICHGTERM,
        MAX17205_AD_NFILTERCFG,
        MAX17205_AD_NVEMPTY,
        MAX17205_AD_NLEARNCFG,
        MAX17205_AD_NQRTABLE00,
        MAX17205_AD_NQRTABLE10,
        MAX17205_AD_NQRTABLE20,
        MAX17205_AD_NQRTABLE30,
        MAX17205_AD_NCYCLES,
        MAX17205_AD_NFULLCAPNOM,
        MAX17205_AD_NRCOMP0,
        MAX17205_AD_NTEMPCO,
        MAX17205_AD_NIAVGEMPTY,
        MAX17205_AD_NFULLCAPREP,
        MAX17205_AD_NVOLTTEMP,
        MAX17205_AD_NMAXMINCURR,
        MAX17205_AD_NMAXMINVOLT,
        MAX17205_AD_NMAXMINTEMP,
        MAX17205_AD_NSOC,
        MAX17205_AD_NTIMERH,
        MAX17205_AD_NCONFIG,
        MAX17205_AD_NRIPPLECFGCFG,
        MAX17205_AD_NMISCCFG,
        MAX17205_AD_NDESIGNCAP,
        MAX17205_AD_NHIBCFG,
        MAX17205_AD_NPACKCFG,
        MAX17205_AD_NRELAXCFG,
        MAX17205_AD_NCONVGCFG,
        MAX17205_AD_NNVCFG0,
        MAX17205_AD_NNVCFG1,
        MAX17205_AD_NNVCFG2,
        MAX17205_AD_NVALRTTH,
        MAX17205_AD_NTALRTTH,
        MAX17205_AD_NSALRTTH,
        MAX17205_AD_NIALRTTH,
        MAX17205_AD_NFULLSOCTHR,
        MAX17205_AD_NTTFCFG,
        MAX17205_AD_NCGAIN,
        MAX17205_AD_NTCURVE,
        MAX17205_AD_NTGAIN,
        MAX17205_AD_NTOFF,
        MAX17205_AD_NMANFCTRNAME0,
        MAX17205_AD_NMANFCTRNAME1,
        MAX17205_AD_NMANFCTRNAME2,
        MAX17205_AD_NRSENSE,
        MAX17205_AD_NUSER1D0,
        MAX17205_AD_NUSER1D1,
        MAX17205_AD_NAGEFCCFG,
        MAX17205_AD_NDESIGNVOLTAGE,
        MAX17205_AD_NUSER1D4,
        MAX17205_AD_NRFASTVSHDN,
        MAX17205_AD_NMANFCTRDATE,
        MAX17205_AD_NFIRSTUSED,
        MAX17205_AD_NSERIALNUMBER0,
        MAX17205_AD_NSERIALNUMBER1,
        MAX17205_AD_NSERIALNUMBER2,
        MAX17205_AD_NDEVICENAME0,
        MAX17205_AD_NDEVICENAME1,
        MAX17205_AD_NDEVICENAME2,
        MAX17205_AD_NDEVICENAME3,
        MAX17205_AD_NDEVICENAME4,
    };

    for(size_t i = 0; i < ARRAY_LEN(reg_list); ++i) {
        uint16_t buf = 0;
        rc = max17205_reg_read(dev, reg_list[i], &buf);
        if (rc) {
            return rc;
        }
        LOG_DBG("   %-30s register 0x%04X is 0x%04X", max17205_reg_to_str(reg_list[i]), reg_list[i], buf);
    }
    return 0;
}

int max17205_read_history(const struct device *dev)
{
    int i;
    int rc;
    uint16_t write_flags[26];
    uint16_t valid_flags[26];
    uint8_t page_good[203];

    //Read all flag information from the IC
    rc = max17205_reg_write(dev, MAX17205_AD_COMMAND, 0xE2FB);
    if (rc) {
        LOG_ERR("Failed to send command to read the lower history write flags.");
        return rc;
    }
    k_msleep(MAX17205_T_RECAL_MS);

    for (i = 0; i < 15; i++) {
        rc = max17205_reg_read(dev, 0x1E1 + i, &write_flags[i]); // first set starts at 0x1E1, not 0x1E0 as the remaining sets do
        if (rc) {
            return rc;
        }
    }

    rc = max17205_reg_write(dev, MAX17205_AD_COMMAND, 0xE2FC);
    if (rc) {
        LOG_ERR("Failed to send command to read the upper history write flags.");
        return rc;
    }
    k_msleep(MAX17205_T_RECAL_MS);
    for (i = 0; i < 11; i++) {
        rc = max17205_reg_read(dev, 0x1E0 + i, &write_flags[i + 15]);
        if (rc) {
            return rc;
        }
    }
    for (i = 0; i < 5; i++) {
        rc = max17205_reg_read(dev, 0x1EB + i, &valid_flags[i]);
        if (rc) {
            return rc;
        }
    }

    rc = max17205_reg_write(dev, MAX17205_AD_COMMAND, 0xE2FD);
    if (rc) {
        LOG_ERR("Failed to send command to read the lower history valid flags.");
        return rc;
    }
    k_msleep(MAX17205_T_RECAL_MS);
    for (i = 0; i < 16; i++) {
        rc = max17205_reg_read(dev, 0x1E0 + i, &valid_flags[i + 5]);
        if (rc) {
            return rc;
        }
    }

    rc = max17205_reg_write(dev, MAX17205_AD_COMMAND, 0xE2FE);
    if (rc) {
        LOG_ERR("Failed to send command to read the upper history valid flags.");
        return rc;
    }
    k_msleep(MAX17205_T_RECAL_MS);
    for (i = 0; i < 5; i++) {
        rc = max17205_reg_read(dev, 0x1E0 + i, &valid_flags[i + 21]);
        if (rc) {
            return rc;
        }
    }

    int loop;
    int word;
    int position;
    int flag1;
    int flag2;
    int flag3;
    int flag4;

    //Determine which history pages contain valid data
    for (loop = 0; loop < 202; loop++)
    {
        word = loop / 8;
        position = loop % 8;
        flag1 = (write_flags[word] >> position) & 0x0001;
        flag2 = (write_flags[word] >> (position + 8)) & 0x0001;
        flag3 = (valid_flags[word] >> position) & 0x0001;
        flag4 = (valid_flags[word] >> (position + 8)) & 0x0001;
        if ((flag1 || flag2) && (flag3 || flag4)) {
            page_good[loop] = true;
        } else {
            page_good[loop] = false;
        }
    }

    //Read all the history data from the IC
    uint16_t history_data;
    uint8_t history_length = 0;

    LOG_DBG("History:");
    for(loop = 0; loop < 202; loop++)
    {
        if (!page_good[loop]) {
            continue;
        }
        rc = max17205_reg_write(dev, MAX17205_AD_COMMAND, 0xE226 + loop);
        if (rc) {
            LOG_ERR("Failed to send command to read the history entry %d", loop);
            return rc;
        }
        k_msleep(MAX17205_T_RECAL_MS);
        LOG_DBG(" Entry %d:", history_length);
        for (i = 0; i < 16; i++) {
            rc = max17205_reg_read(dev, 0x1E0 + i, &history_data);
            if (rc) {
                return rc;
            }
            LOG_DBG("   %-30s register 0x%04X is 0x%04X", max17205_reg_to_str(0x1A0 + i), 0x1A0 + i, history_data);
        }
        history_length++;
    }
    return 0;
}

/**
 * Helper function to trigger write of volatile memory on MAX71205 chip.
 * Returns true if NV RAM was written, false otherwise.
 */
bool nv_ram_write(const struct device *dev, const char *pack_str)
{
#if DEBUG_PRINT
    (void)pack_str;
#endif
    LOG_DBG("\r\nEnsure NV RAM settings are correct for %s", pack_str);

    bool all_elements_match = false;
    int rc = max17205_validate_registers(dev, batt_nv_programing_cfg, ARRAY_LEN(batt_nv_programing_cfg),
                                         &all_elements_match);
    if (rc) {
        return false;
    }

    if (all_elements_match) {
        LOG_DBG("All NV RAM elements already match expected values...");
        return false;
    } else {
        LOG_DBG("One or more NV RAM elements don't match expected values...");
    }

    rc = max17205_write_registers(dev, batt_nv_programing_cfg, ARRAY_LEN(batt_nv_programing_cfg));
    if (rc) {
        LOG_DBG("Failed to write new NV RAM reg values\n");
        return false;
    }
    LOG_DBG("Successfully wrote new NV RAM reg values");

    all_elements_match = false;
    rc = max17205_validate_registers(dev, batt_nv_programing_cfg, ARRAY_LEN(batt_nv_programing_cfg),
                                     &all_elements_match);
    if (rc) {
        return false;
    }
    if (!all_elements_match) {
        LOG_DBG("NV RAM elements failed to update after write.");
        return false;
    }

    LOG_DBG("All NV RAM elements now match expected values.");

    //Now make the chip use the changes written to the shadow registers.
    max17205_firmware_reset(dev);
    return true;
}

/**
 * Helper function to trigger write of volatile memory on MAX71205 chip.
 * Returns true if NV was written, false otherwise.
 */
#if ENABLE_NV_WRITE_PROMPT
static bool prompt_nv_write(const struct device *dev, const char *pack_str)
{
    LOG_DBG("Write NV RAM to NV%s", pack_str);

    uint8_t num_writes_left = 0;
    if (max17205_read_writes_remaining(dev, &num_writes_left) == 0) {
        LOG_DBG("Num_writes_left = %u", num_writes_left);
    }

    if (num_writes_left > 0) {
        // Answer n to just use the changes in the volatile registers
        LOG_DBG("Write NV memory on MAX17205 for %s ? y/n? ", pack_str);
        uint8_t ch = console_getchar(); // TODO: Zephyr console does not have a read timeout; switch to shell

        LOG_DBG("");

        if (ch == 'y') {
            LOG_DBG("Attempting to write non volatile memory on MAX17205...");
            k_msleep(50);

            if (max17205_nv_program(dev) == 0) {
                LOG_DBG("Successfully wrote non volatile memory on MAX17205...");
            } else {
                LOG_DBG("Failed to write non volatile memory on MAX17205...");
            }
            return true; // NV changes made
        } else {
            LOG_DBG("Update skipped.");
        }
    } else {
        LOG_DBG("No more NV writes remain.");
    }

    return false; // no NV changes made
}
#endif // ENABLE_NV_WRITE_PROMPT

//If state of charge is known to be full, set LS bits D6-D0 of LearnCfg register to 0b111
//and write MixCap and RepCap registers to 2600.
#if ENABLE_LEARN_COMPLETE && DEBUG_PRINT
static bool update_learning_complete(const struct device *dev, pack_t *pack)
{
    batt_pack_data_t *pack_data = &pack->data;
    bool ret = false;

    if ((pack_data->batt_mV > BATT_FULL_THRESHOLD_MV) &&
        (pack_data->avg_current_mA < EOC_THRESHOLD_MA) &&
        (pack_data->avg_current_mA >= 0) &&
        (pack_data->full_capacity_mAh >= CELL_CAPACITY_MAH) ) {

        LOG_DBG("Pack %d seems full", pack->pack_number);
        uint16_t state;
        int rc = max17205_read_learn_stage(dev, &state);

        if (rc) {
            LOG_DBG("Error reading learn state");
            return ret;
        }
        LOG_DBG("Learning state = %u", state);
        if (state == MAX17205_LEARN_COMPLETE) {
            LOG_DBG("Learning is already complete.");
        } else {
            rc = max17205_write_learn_stage(dev, MAX17205_LEARN_COMPLETE);
            if (rc) {
                LOG_DBG("Error writing learn state");
                return ret;
            }
            rc = max17205_read_learn_stage(dev, &state);
            if (rc) {
                LOG_DBG("Error checking learn state");
                return ret;
            }
            if (state != MAX17205_LEARN_COMPLETE) {
                LOG_DBG("Error setting state = %u; is %u", MAX17205_LEARN_COMPLETE, state);
                return ret;
            }
            LOG_DBG("Learning state set = %u", state);
            ret = true;
        }
        pack_data->mix_capacity_mAh = pack_data->reported_capacity_mAh = pack_data->full_capacity_mAh;
        if ( (rc = max17205_write_capacity(dev, MAX17205_CHAN_MIX_CAPACITY, pack_data->mix_capacity_mAh)) != MSG_OK ) {
            LOG_DBG("Failed to write MIXCAP");
        } else if ( (rc = max17205_write_capacity(dev, SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY, pack_data->reported_capacity_mAh)) != MSG_OK ) {
            LOG_DBG("Failed to write REPCAP");
        } else {
            LOG_DBG("Mixcap and repcap set to %u", pack_data->full_capacity_mAh);
        }
        return ret;
    }
    return false;
}
#endif // ENABLE_LEARN_COMPLETE

void manage_calibration(void)
{
#if DEBUG_PRINT
    unsigned int i;
#if ENABLE_NV_WRITE_PROMPT
    bool nv_written = false;
#endif
    pack_t *pack;

    for (i = 0; i < NPACKS; i++) {
        pack = get_pack(i);
        LOG_DBG("%s:", pack->name);
        max17205_print_volatile_memory(pack->dev);

#if ENABLE_LEARN_COMPLETE
        // If ENABLE_LEARN_COMPLETE=1, ENABLE_NV_MEMORY_UPDATE_CODE=1 and DEBUG_PRINT are all enabled, we will only prompt to update
        // NV when learning is complete. If ENABLE_LEARN_COMPLETE is not 1 but the others are, then we will only prompt to update
        // NV if there is a change to NV RAM required (done prior to the main loop).
        pack->updated = update_learning_complete(pack->dev, pack);
#endif
#if ENABLE_NV_WRITE_PROMPT
        if (pack->init && pack->updated) {
            nv_written |= prompt_nv_write(pack->dev, pack->name);
            pack->updated = false;
        }
#endif
    }

#if ENABLE_NV_WRITE_PROMPT
    if (nv_written) {
        LOG_DBG("Done with NV RAM update code, set ENABLE_NV_MEMORY_UPDATE_CODE=0 and re-write firmware.");
        for (;;) {
            LOG_DBG(".");
            k_msleep(1000);
        }
    }
#endif
#endif
}

// TODO: replace with X macros-generated table / fn
const char* max17205_reg_to_str(const uint16_t reg) {
    switch (reg) {
        case MAX17205_AD_AVGCELL1:
            return "MAX17205_AD_AVGCELL1";
        case MAX17205_AD_AVGCELL2:
            return "MAX17205_AD_AVGCELL2";
        case MAX17205_AD_AVGVCELL:
            return "MAX17205_AD_AVGVCELL";
        case MAX17205_AD_VCELL:
            return "MAX17205_AD_VCELL";
        case MAX17205_AD_AVCAP:
            return "MAX17205_AD_AVCAP";
        case MAX17205_AD_MIXCAP:
            return "MAX17205_AD_MIXCAP";
        case MAX17205_AD_TTE:
            return "MAX17205_AD_TTE";
        case MAX17205_AD_TTF:
            return "MAX17205_AD_TTF";
        case MAX17205_AD_AVSOC:
            return "MAX17205_AD_AVSOC";
        case MAX17205_AD_TEMP:
            return "MAX17205_AD_TEMP";
        case MAX17205_AD_CURRENT:
            return "MAX17205_AD_CURRENT";
        case MAX17205_AD_AVGCURRENT:
            return "MAX17205_AD_AVGCURRENT";
        case MAX17205_AD_LEARNCFG:
            return "MAX17205_AD_LEARNCFG";
        case MAX17205_AD_QRTABLE00:
            return "MAX17205_AD_QRTABLE00";
        case MAX17205_AD_QRTABLE10:
            return "MAX17205_AD_QRTABLE10";
        case MAX17205_AD_QRTABLE20:
            return "MAX17205_AD_QRTABLE20";
        case MAX17205_AD_QRTABLE30:
            return "MAX17205_AD_QRTABLE30";
        case MAX17205_AD_CYCLES:
            return "MAX17205_AD_CYCLES";
        case MAX17205_AD_FULLCAPNOM:
            return "MAX17205_AD_FULLCAPNOM";
        case MAX17205_AD_RCOMP0:
            return "MAX17205_AD_RCOMP0";
        case MAX17205_AD_TEMPCO:
            return "MAX17205_AD_TEMPCO";
        case MAX17205_AD_IAVGEMPTY:
            return "MAX17205_AD_IAVGEMPTY";
        case MAX17205_AD_FULLCAPREP:
            return "MAX17205_AD_FULLCAPREP";
        case MAX17205_AD_MAXMINCURR:
            return "MAX17205_AD_MAXMINCURR";
        case MAX17205_AD_MAXMINVOLT:
            return "MAX17205_AD_MAXMINVOLT";
        case MAX17205_AD_MAXMINTEMP:
            return "MAX17205_AD_MAXMINTEMP";
        case MAX17205_AD_MIXSOC:
            return "MAX17205_AD_MIXSOC";
        case MAX17205_AD_VFSOC:
            return "MAX17205_AD_VFSOC";
        case MAX17205_AD_TIMERH:
            return "MAX17205_AD_TIMERH";
        case MAX17205_AD_BATT:
            return "MAX17205_AD_BATT";
        case MAX17205_AD_TEMP1:
            return "MAX17205_AD_TEMP1";
        case MAX17205_AD_TEMP2:
            return "MAX17205_AD_TEMP2";
        case MAX17205_AD_INTTEMP:
            return "MAX17205_AD_INTTEMP";
        case MAX17205_AD_AVGTEMP1:
            return "MAX17205_AD_AVGTEMP1";
        case MAX17205_AD_AVGTEMP2:
            return "MAX17205_AD_AVGTEMP2";
        case MAX17205_AD_AVGINTTEMP:
            return "MAX17205_AD_AVGINTTEMP";
        case MAX17205_AD_REPCAP:
            return "MAX17205_AD_REPCAP";
        case MAX17205_AD_VFREMCAP:
            return "MAX17205_AD_VFREMCAP";
        case MAX17205_AD_PACKCFG:
            return "MAX17205_AD_PACKCFG";
        case MAX17205_AD_DESIGNCAP:
            return "MAX17205_AD_DESIGNCAP";
        case MAX17205_AD_REPSOC:
            return "MAX17205_AD_REPSOC";
        case MAX17205_AD_NXTABLE0:
            return "MAX17205_AD_NXTABLE0";
        case MAX17205_AD_NXTABLE1:
            return "MAX17205_AD_NXTABLE1";
        case MAX17205_AD_NXTABLE2:
            return "MAX17205_AD_NXTABLE2";
        case MAX17205_AD_NXTABLE3:
            return "MAX17205_AD_NXTABLE3";
        case MAX17205_AD_NXTABLE4:
            return "MAX17205_AD_NXTABLE4";
        case MAX17205_AD_NXTABLE5:
            return "MAX17205_AD_NXTABLE5";
        case MAX17205_AD_NXTABLE6:
            return "MAX17205_AD_NXTABLE6";
        case MAX17205_AD_NXTABLE7:
            return "MAX17205_AD_NXTABLE7";
        case MAX17205_AD_NXTABLE8:
            return "MAX17205_AD_NXTABLE8";
        case MAX17205_AD_NXTABLE9:
            return "MAX17205_AD_NXTABLE9";
        case MAX17205_AD_NXTABLE10:
            return "MAX17205_AD_NXTABLE10";
        case MAX17205_AD_NXTABLE11:
            return "MAX17205_AD_NXTABLE11";
        case MAX17205_AD_NUSER18C:
            return "MAX17205_AD_NUSER18C";
        case MAX17205_AD_NUSER18D:
            return "MAX17205_AD_NUSER18D";
        case MAX17205_AD_NODSCTH:
            return "MAX17205_AD_NODSCTH";
        case MAX17205_AD_NODSCCFG:
            return "MAX17205_AD_NODSCCFG";
        case MAX17205_AD_NOCVTABLE0:
            return "MAX17205_AD_NOCVTABLE0";
        case MAX17205_AD_NOCVTABLE1:
            return "MAX17205_AD_NOCVTABLE1";
        case MAX17205_AD_NOCVTABLE2:
            return "MAX17205_AD_NOCVTABLE2";
        case MAX17205_AD_NOCVTABLE3:
            return "MAX17205_AD_NOCVTABLE3";
        case MAX17205_AD_NOCVTABLE4:
            return "MAX17205_AD_NOCVTABLE4";
        case MAX17205_AD_NOCVTABLE5:
            return "MAX17205_AD_NOCVTABLE5";
        case MAX17205_AD_NOCVTABLE6:
            return "MAX17205_AD_NOCVTABLE6";
        case MAX17205_AD_NOCVTABLE7:
            return "MAX17205_AD_NOCVTABLE7";
        case MAX17205_AD_NOCVTABLE8:
            return "MAX17205_AD_NOCVTABLE8";
        case MAX17205_AD_NOCVTABLE9:
            return "MAX17205_AD_NOCVTABLE9";
        case MAX17205_AD_NOCVTABLE10:
            return "MAX17205_AD_NOCVTABLE10";
        case MAX17205_AD_NOCVTABLE11:
            return "MAX17205_AD_NOCVTABLE11";
        case MAX17205_AD_NICHGTERM:
            return "MAX17205_AD_NICHGTERM";
        case MAX17205_AD_NFILTERCFG:
            return "MAX17205_AD_NFILTERCFG";
        case MAX17205_AD_NVEMPTY:
            return "MAX17205_AD_NVEMPTY";
        case MAX17205_AD_VEMPTY:
            return "MAX17205_AD_VEMPTY";
        case MAX17205_AD_NLEARNCFG:
            return "MAX17205_AD_NLEARNCFG";
        case MAX17205_AD_NQRTABLE00:
            return "MAX17205_AD_NQRTABLE00";
        case MAX17205_AD_NQRTABLE10:
            return "MAX17205_AD_NQRTABLE10";
        case MAX17205_AD_NQRTABLE20:
            return "MAX17205_AD_NQRTABLE20";
        case MAX17205_AD_NQRTABLE30:
            return "MAX17205_AD_NQRTABLE30";
        case MAX17205_AD_NCYCLES:
            return "MAX17205_AD_NCYCLES";
        case MAX17205_AD_NFULLCAPNOM:
            return "MAX17205_AD_NFULLCAPNOM";
        case MAX17205_AD_NRCOMP0:
            return "MAX17205_AD_NRCOMP0";
        case MAX17205_AD_NTEMPCO:
            return "MAX17205_AD_NTEMPCO";
        case MAX17205_AD_NIAVGEMPTY:
            return "MAX17205_AD_NIAVGEMPTY";
        case MAX17205_AD_NFULLCAPREP:
            return "MAX17205_AD_NFULLCAPREP";
        case MAX17205_AD_NVOLTTEMP:
            return "MAX17205_AD_NVOLTTEMP";
        case MAX17205_AD_NMAXMINCURR:
            return "MAX17205_AD_NMAXMINCURR";
        case MAX17205_AD_NMAXMINVOLT:
            return "MAX17205_AD_NMAXMINVOLT";
        case MAX17205_AD_NMAXMINTEMP:
            return "MAX17205_AD_NMAXMINTEMP";
        case MAX17205_AD_NSOC:
            return "MAX17205_AD_NSOC";
        case MAX17205_AD_NTIMERH:
            return "MAX17205_AD_NTIMERH";
        case MAX17205_AD_NCONFIG:
            return "MAX17205_AD_NCONFIG";
        case MAX17205_AD_NRIPPLECFGCFG:
            return "MAX17205_AD_NRIPPLECFGCFG";
        case MAX17205_AD_NMISCCFG:
            return "MAX17205_AD_NMISCCFG";
        case MAX17205_AD_NDESIGNCAP:
            return "MAX17205_AD_NDESIGNCAP";
        case MAX17205_AD_NHIBCFG:
            return "MAX17205_AD_NHIBCFG";
        case MAX17205_AD_NPACKCFG:
            return "MAX17205_AD_NPACKCFG";
        case MAX17205_AD_NRELAXCFG:
            return "MAX17205_AD_NRELAXCFG";
        case MAX17205_AD_NCONVGCFG:
            return "MAX17205_AD_NCONVGCFG";
        case MAX17205_AD_NNVCFG0:
            return "MAX17205_AD_NNVCFG0";
        case MAX17205_AD_NNVCFG1:
            return "MAX17205_AD_NNVCFG1";
        case MAX17205_AD_NNVCFG2:
            return "MAX17205_AD_NNVCFG2";
        case MAX17205_AD_NVALRTTH:
            return "MAX17205_AD_NVALRTTH";
        case MAX17205_AD_NTALRTTH:
            return "MAX17205_AD_NTALRTTH";
        case MAX17205_AD_NSALRTTH:
            return "MAX17205_AD_NSALRTTH";
        case MAX17205_AD_NIALRTTH:
            return "MAX17205_AD_NIALRTTH";
        case MAX17205_AD_NFULLSOCTHR:
            return "MAX17205_AD_NFULLSOCTHR";
        case MAX17205_AD_NTTFCFG:
            return "MAX17205_AD_NTTFCFG";
        case MAX17205_AD_NCGAIN:
            return "MAX17205_AD_NCGAIN";
        case MAX17205_AD_NTCURVE:
            return "MAX17205_AD_NTCURVE";
        case MAX17205_AD_NTGAIN:
            return "MAX17205_AD_NTGAIN";
        case MAX17205_AD_NTOFF:
            return "MAX17205_AD_NTOFF";
        case MAX17205_AD_NMANFCTRNAME0:
            return "MAX17205_AD_NMANFCTRNAME0";
        case MAX17205_AD_NMANFCTRNAME1:
            return "MAX17205_AD_NMANFCTRNAME1";
        case MAX17205_AD_NMANFCTRNAME2:
            return "MAX17205_AD_NMANFCTRNAME2";
        case MAX17205_AD_NRSENSE:
            return "MAX17205_AD_NRSENSE";
        case MAX17205_AD_NUSER1D0:
            return "MAX17205_AD_NUSER1D0";
        case MAX17205_AD_NUSER1D1:
            return "MAX17205_AD_NUSER1D1";
        case MAX17205_AD_NAGEFCCFG:
            return "MAX17205_AD_NAGEFCCFG";
        case MAX17205_AD_NDESIGNVOLTAGE:
            return "MAX17205_AD_NDESIGNVOLTAGE";
        case MAX17205_AD_NUSER1D4:
            return "MAX17205_AD_NUSER1D4";
        case MAX17205_AD_NRFASTVSHDN:
            return "MAX17205_AD_NRFASTVSHDN";
        case MAX17205_AD_NMANFCTRDATE:
            return "MAX17205_AD_NMANFCTRDATE";
        case MAX17205_AD_NFIRSTUSED:
            return "MAX17205_AD_NFIRSTUSED";
        case MAX17205_AD_NSERIALNUMBER0:
            return "MAX17205_AD_NSERIALNUMBER0";
        case MAX17205_AD_NSERIALNUMBER1:
            return "MAX17205_AD_NSERIALNUMBER1";
        case MAX17205_AD_NSERIALNUMBER2:
            return "MAX17205_AD_NSERIALNUMBER2";
        case MAX17205_AD_NDEVICENAME0:
            return "MAX17205_AD_NDEVICENAME0";
        case MAX17205_AD_NDEVICENAME1:
            return "MAX17205_AD_NDEVICENAME1";
        case MAX17205_AD_NDEVICENAME2:
            return "MAX17205_AD_NDEVICENAME2";
        case MAX17205_AD_NDEVICENAME3:
            return "MAX17205_AD_NDEVICENAME3";
        case MAX17205_AD_NDEVICENAME4:
            return "MAX17205_AD_NDEVICENAME4";
    }
    return "[reg?]";
}

