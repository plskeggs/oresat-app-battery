#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/console/console.h>
#include <canopennode.h>
#include <OD.h>
#include <oresat.h>

#include "batt.h"
#include "calib.h"
#include "hist.h"

//#include "max17205.h"
//#include "CANopen.h"
//#include "chtime.h"
//#include "oresat_f0.h"
//#include "flash_f0.h"
#include "crc.h"
#include <sys/param.h>
#include <string.h>

TODO:
- replace #defines that affect build below with Kconfigs:
   DEBUG_PRINT, VERBOSE_DEBUG, ENABLE_NV_MEMORY_UPDATE_CODE, ENABLE_LEARN_COMPLETE, ENABLE_HEATERS,
   ENABLE_CHARGING_CONTROL, HIST_STORE_PROMPT
- re-review writes to N-version of registers vs. non-N
- do we need to still deal with PACKCFG here? done in the driver now
- rewrite code that uses GPIOs (for heaters, charge/discharge status / control, and LED)

LOG_MODULE_DECLARE(app_battery, LOG_LEVEL_DBG);

// Dump complete battery history
#if !defined(VERBOSE_DEBUG)
#define VERBOSE_DEBUG 0
#endif
#if VERBOSE_DEBUG
#pragma message("Verbose debug messages enabled")
#else
#pragma message("Verbose debug messages disabled")
#endif

// If batt_nv_programing_cfg registers do not match current, rewrite the RAM shadow then prompt to write to NV.
#if !defined(ENABLE_NV_MEMORY_UPDATE_CODE)
#define ENABLE_NV_MEMORY_UPDATE_CODE 0
#endif
#if ENABLE_NV_MEMORY_UPDATE_CODE
#pragma message("NV memory update code enabled")
#else
#pragma message("NV memory update code disabled")
#endif

// If state of charge is known to be full, set LS bits D6-D0 of LearnCfg register to 0b111
// and write MixCap and RepCap registers to 2600.
#if !defined(ENABLE_LEARN_COMPLETE)
#define ENABLE_LEARN_COMPLETE 0
#endif
#if ENABLE_LEARN_COMPLETE
#pragma message("Enable learn complete enabled")
#else
#pragma message("Enable learn complete disabled")
#endif

// Recommend setting ENABLE_HEADERS to 0 for battery board v2.1. Otherwise, brownouts can occur, causing
// the C3 to reboot the battery board.
#if !defined(ENABLE_HEATERS)
#define ENABLE_HEATERS 1
#endif
#if ENABLE_HEATERS
#pragma message("Heaters enabled")
#else
#pragma message("Heaters disabled")
#endif

#define NV_WRITE_PROMPT_TIMEOUT_S 15

// This was disabled per discussion in Slack on April 25, 2021.
#define ENABLE_CHARGING_CONTROL 0

#if CONFIG_LOG_DEFAULT_LEVEL >= LOG_LEVEL_INF
#define DEBUG_PRINT 1
#else
#define DEBUG_PRINT 0
#endif

// Simplify conditionals below -- DEBUG_PRINT is required for ENABLE_NV_MEMORY_UPDATE_CODE to do anything
#if DEBUG_PRINT && ENABLE_NV_MEMORY_UPDATE_CODE
#define ENABLE_NV_WRITE_PROMPT 1
#else
#define ENABLE_NV_WRITE_PROMPT 0
#endif

// Uncomment to only store within 3 seconds on 'y', or erase hist store flash on 'e'

//if DEBUG_PRINT
//#define HIST_STORE_PROMPT 1
//#else
#define HIST_STORE_PROMPT 0
//#endif

// Voltage below which we should stop everything until charging starts
#define SHUTDOWN_MV 2850

// Subtask periods
#define CALIB_INTERVAL_MS 120000 // 2 minutes
#define RUNTIME_BATT_STORE_INTERVAL_MS 300000U // 5 minutes
#define LED_TOGGLE_INTERVAL_MS 500 // 0.5 seconds
#define BATT_TASK_SLEEP_INTERVAL_MS 10

typedef enum {
    BATTERY_OD_ERROR_INFO_CODE_NONE = 0,
    BATTERY_OD_ERROR_INFO_CODE_PACK_1_COMM_ERROR,
    BATTERY_OD_ERROR_INFO_CODE_PACK_2_COMM_ERROR,
    BATTERY_OD_ERROR_INFO_CODE_PACK_FAIL_SAFE_HEATING,
    BATTERY_OD_ERROR_INFO_CODE_PACK_FAIL_SAFE_CHARGING,
} battery_od_error_info_code_t;

typedef enum {
    BATTERY_STATE_MACHINE_STATE_NOT_HEATING = 0,
    BATTERY_STATE_MACHINE_STATE_HEATING,
} battery_heating_state_machine_state_t;

typedef enum {
    STATUS_BIT_HEATER = 0,
    STATUS_BIT_DCHG_DIS,
    STATUS_BIT_CHG_DIS,
    STATUS_BIT_DCHG_STAT,
    STATUS_BIT_CHG_STAT
} status_bits;

/*
  The values for batt_nv_programing_cfg are detailed in the google document "MAX17205 Register Values"
  These values were generated using the windows tool from Maxim and while they are probably not totally
  correct yield generally reasonable read back values from the MAX17 chip.
 */
static const uint16_t PACKCFG = (_VAL2FLD(MAX17205_PACKCFG_NCELLS, NCELLS) | // 2 cells
                                 MAX17205_PACKCFG_BALCFG_40 | // 40 mV cell balance threshold
                                 MAX17205_PACKCFG_BTEN | // enable Vbatt channel
                                 MAX17205_PACKCFG_CHEN | // enable voltage measurements cell1, cell2, Vbatt
                                 MAX17205_PACKCFG_TDEN | // enable die temperature measurement
                                 MAX17205_PACKCFG_A1EN | // enable AIN1 channel temperature measurement
                                 MAX17205_PACKCFG_A2EN); // enable AIN2 channel temperature measurement

static const max17205_regval_t batt_nv_programing_cfg[] = {
    {MAX17205_AD_NPACKCFG,     PACKCFG },
    {MAX17205_AD_NNVCFG0,      0x09A0 }, // Wizard: enCfg=1, enRCfg=1, enLCfg=1, enICT=1, enVE=1
    {MAX17205_AD_NNVCFG1,      0x8006 }, // Wizard: enTGO=1, enCrv=1, enCTE=1
    {MAX17205_AD_NNVCFG2,      0xFF0A }, // factory default; life logging every 10 cycles,
                                         //  enIAvg=1, enFC=1, enVT=1, enMMC=1, enMMV=1, enMMT=1, enSOC=1, enT=1
    {MAX17205_AD_NICHGTERM,    0x014D }, // Wizard: 52.0 mA
    {MAX17205_AD_NVEMPTY,      0x965A }, // Wizard: VEmpty = 0x12C * 10mV = 3.0v; VRecovery = 0x5A * 40mV = 3.6v
    {MAX17205_AD_NTCURVE,      0x0064 }, // Wizard / datasheet recommendation for Fenwal thermistor
    {MAX17205_AD_NTGAIN,       0xF49A }, // ditto
    {MAX17205_AD_NTOFF,        0x16A1 }, // ditto
    {MAX17205_AD_NDESIGNCAP,   0x1450 }, // Wizard: 2600 mAh (x2 conversion factor)
    {MAX17205_AD_NFULLCAPREP,  0x1450 }, // ditto
    {MAX17205_AD_NFULLCAPNOM,  0x1A22 }, // Wizard: 0x1794; full learning cycle found 0x1A22 on average
                                         //  of 2 packs so use that instead

    // Missing from in flight fw, but present in Wizard output with m5 EZ battery model:
    {MAX17205_AD_NQRTABLE00,   0x2280 }, // nQRTable00-30 contain battery characteristic data
    {MAX17205_AD_NQRTABLE10,   0x1000 }, // no further information provided in datasheet about
    {MAX17205_AD_NQRTABLE20,   0x0681 }, // actual meaning of each value
    {MAX17205_AD_NQRTABLE30,   0x0682 },
    {MAX17205_AD_NIAVGEMPTY,   0xEBB0 }, // Calling this I (current) seems wrong -- 0xEBB0 is actually -5200
                                         //  (2s complement) x 156.25uA = 0.8125A. Alternate value based on
                                         //  nVCfg2.enIAvg = 0 is -1 x nFullCapNom; for that EBB0 is right,
                                         //  but isn't actually right since nVCfg2.enIAvg = 1.
    {MAX17205_AD_NCONFIG,      0x0211 }, // Temperature channel enable, temperature alert enable
    {MAX17205_AD_NMISCCFG,     0x3870 }, // FUS (Full Update Slope) = 6% (rate of adj of FullCapRep near end
                                         //  of charge cycle); MR (Servo Mixing Rate) = 18.75uV. However,
                                         //  bit 11 must = 1 according to datasheet (Wizard said: 0x3070),
                                         //  so set that bit
    {MAX17205_AD_NCONVGCFG,    0x2241 }, // recommended factory default; RepLow = 4%; VoltLowOff = 40mV;
                                         //  MinSlopeX = 0.25; RepL per Stage = 1%
    {MAX17205_AD_NFULLSOCTHR,  0x5005 }, // recommended factory default for EZ Performance = 80% (units of 1/256%)
    {MAX17205_AD_NRIPPLECFGCFG,0x0204 }, // recommended factory default; kDV = 64 (amount of capacity to
                                         //  compensate proportional to ripple?); NR = filter mag for ripple
                                         //  observation = 22.4 seconds
    {MAX17205_AD_NRCOMP0,      0x006F }  // characteristic info for computing open-circuit voltage of cell under
                                         //  loaded conditions; undefined encoding
};

#if ENABLE_HEATERS
static battery_heating_state_machine_state_t current_battery_state_machine_state = BATTERY_STATE_MACHINE_STATE_NOT_HEATING;
#endif

// All packs defined here, including non-zero initial data.
static pack_t packs[NPACKS] = {
    {
     .heater_on = LINE_HEATER_ON_1,
     .line_dchg_dis = LINE_DCHG_DIS_PK1,
     .line_chg_dis = LINE_CHG_DIS_PK1,
     .line_dchg_stat = LINE_DCHG_STAT_PK1,
     .line_chg_stat = LINE_CHG_STAT_PK1,
     .pack_number = 1,
     .name = "Pack 1"
    },
    {
     .heater_on = LINE_HEATER_ON_2,
     .line_dchg_dis = LINE_DCHG_DIS_PK2,
     .line_chg_dis = LINE_CHG_DIS_PK2,
     .line_dchg_stat = LINE_DCHG_STAT_PK2,
     .line_chg_stat = LINE_CHG_STAT_PK2,
     .pack_number = 2,
     .name = "Pack 2"
    }
};

pack_t *get_pack(unsigned int pack)
{
    if (pack < NPACKS) {
        return &packs[pack];
    } else {
        return NULL;
    }
}

static void palSetLine(int line)
{
    // TODO: replace with GPIO stuff
}

static void palClearLine(int line)
{
    // TODO: replace with GPIO stuff
}

static void palToggleLine(int line)
{
    // TODO: replace with GPIO stuff
}

static void heaters_on(bool on)
{
    if (on) {
        palSetLine(LINE_MOARPWR);
        palSetLine(LINE_HEATER_ON_1);
        palSetLine(LINE_HEATER_ON_2);
        LOG_DBG("Heaters ON\n\n");
    } else {
        palClearLine(LINE_HEATER_ON_1);
        palClearLine(LINE_HEATER_ON_2);
        palClearLine(LINE_MOARPWR);
        LOG_DBG("Heaters OFF\n\n");
    }
}

/**
 * @brief Runs the battery state machine, responsible for turning on/off heaters, charging, discharging etc.
 */
#if ENABLE_HEATERS
static void run_battery_heating_state_machine(void)
{
    unsigned int i;

    for (i = 0; i < NPACKS; i++) {
        if (!packs[i].data.is_data_valid) {
            LOG_DBG("FAILSAFE: ");
            heaters_on(false);
            return;
        }
    }

    switch (current_battery_state_machine_state) {
        case BATTERY_STATE_MACHINE_STATE_HEATING:
            heaters_on(true);

            //Once they’re greater than 5 °C or the combined pack capacity is < 25%
            bool warm_enough = true;
            uint16_t total_state_of_charge = 0;
            for (i = 0; i < NPACKS; i++) {
                if( packs[i].data.avg_temp_1_C <= 5 ) {
                    warm_enough = false;
                }
                total_state_of_charge += packs[i].data.present_state_of_charge;
            }
            total_state_of_charge /= NPACKS;
            if( warm_enough || (total_state_of_charge < 25) ) {
                current_battery_state_machine_state = BATTERY_STATE_MACHINE_STATE_NOT_HEATING;
                LOG_DBG("Turning heaters OFF");
            }
            break;
        case BATTERY_STATE_MACHINE_STATE_NOT_HEATING:
            heaters_on(false);

            //Once they’re less than -5 °C and the combined pack capacity is > 25%
            bool too_cold = false;
            bool full_enough = false;
            for (i = 0; i < NPACKS; i++) {
                if( packs[i].data.avg_temp_1_C < -5 ) {
                    too_cold = true;
                }
                if( packs[i].data.present_state_of_charge > 25 ) {
                    full_enough = true;
                }
            }
            if( too_cold && full_enough ) {
                current_battery_state_machine_state = BATTERY_STATE_MACHINE_STATE_HEATING;
                LOG_DBG("Turning heaters ON");
            }
            break;
        default:
            current_battery_state_machine_state = BATTERY_STATE_MACHINE_STATE_NOT_HEATING;
            LOG_DBG("Unknown state: turning heaters OFF");
            break;
    }
}
#endif // ENABLE_HEATERS


/**
 * @brief Query the MAX17 chip for a given pack and populate *pk_data with the current status/state represented in the MAX17
 *
 * @param[in] *pack Destination into which to store MAX17205
 *       pack data.
 */
static void update_battery_charging_state(const pack_t *pack)
{
    LOG_DBG("LINE_DCHG_STAT_PK%d = %u", pack->pack_number, palReadLine(pack->line_dchg_stat));
    LOG_DBG("LINE_CHG_STAT_PK%d  = %u", pack->pack_number, palReadLine(pack->line_chg_stat));

#if !ENABLE_CHARGING_CONTROL
    (void)pack;
#else
    const batt_pack_data_t * const pk_data = &pack->data;

    if (!pk_data->is_data_valid) {
        //fail safe mode
        palSetLine(pack->line_dchg_dis);
        palSetLine(pack->line_chg_dis);
        LOG_DBG("ERROR: %s data is invalid; disabling charging and discharging", pack->name);
        return;
    }
    if( pk_data->v_cell_mV < 3000 || pk_data->present_state_of_charge < 20 ) {
        //Disable discharge on both packs
        LOG_DBG("Disabling discharge on pack %u", pack->pack_number);
        palSetLine(pack->line_dchg_dis);
    } else {
        LOG_DBG("Enabling discharge on pack %u", pack->pack_number);
        //Allow discharge on both packs
        palClearLine(pack->line_dchg_dis);
    }


    if( pk_data->v_cell_mV > 4100 ) {
        LOG_DBG("Disabling charging on pack %u", pack->pack_number);
        palSetLine(pack->line_chg_dis);
    } else {
        LOG_DBG("Enabling charging on pack %u", pack->pack_number);
        palClearLine(pack->line_chg_dis);
    }
#endif // ENABLE_CHARGING_CONTROL
}

/**
 * @param dev[in] The MAX17205 driver object to use to query
 *           pack data from
 * @param *dest[out] Destination into which to store pack data
 *        currently tracked in the MAX17205
 *
 * @return true on success, false otherwise
 */
static bool populate_pack_data(const struct device *dev, batt_pack_data_t *dest)
{
    msg_t r = 0;
    memset(dest, 0, sizeof(*dest));

    dest->is_data_valid = true;


    if( (r = max17205_read_average_temperature(dev, MAX17205_CHAN_TEMP_1, &dest->temp_1_C)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205_read_average_temperature(dev, MAX17205_CHAN_TEMP_2, &dest->temp_2_C)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205_read_average_temperature(dev, SENSOR_CHAN_GAUGE_TEMP, &dest->int_temp_C)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205_read_average_temperature(dev, MAX17205_CHAN_AVG_TEMP_1, &dest->avg_temp_1_C)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205_read_average_temperature(dev, MAX17205_CHAN_AVG_TEMP_2, &dest->avg_temp_2_C)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205_read_average_temperature(dev, MAX17205_CHAN_AVG_INT_TEMP, &dest->avg_int_temp_C)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205_read_max_temperature(dev, &dest->temp_max_C)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205_read_min_temperature(dev, &dest->temp_min_C)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    /* Record pack and cell voltages to object dictionary */
    if( (r = max17205_read_voltage(dev, MAX17205_CHAN_V_CELL_1, &dest->v_cell_1_mV)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    if( (r = max17205_read_voltage(dev, MAX17205_CHAN_V_CELL_AVG, &dest->v_cell_avg_mV)) != MSG_OK ) {
    dest->is_data_valid = false;
   }
    if( (r = max17205_read_voltage(dev, MAX17205_CHAN_V_CELL, &dest->v_cell_mV)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205_read_batt(dev, &dest->batt_mV)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    if( dest->is_data_valid ) {
        dest->v_cell_2_mV = dest->batt_mV - dest->v_cell_1_mV;
    }

    if( (r = max17205_read_max_voltage(dev, &dest->v_cell_max_volt_mV)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205_read_min_voltage(dev, &dest->v_cell_min_volt_mV)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    if( (r = max17205_read_current(dev, SENSOR_CHAN_CURRENT, &dest->current_mA)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205_read_current(dev, SENSOR_CHAN_GAUGE_AVG_CURRENT, &dest->avg_current_mA)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    if( (r = max17205_read_max_current(dev, &dest->max_current_mA)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205_read_min_current(dev, &dest->min_current_mA)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    /* capacity */
    if( (r = max17205_read_capacity(dev, SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY, &dest->full_capacity_mAh)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205_read_capacity(dev, SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY, &dest->available_capacity_mAh)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205_read_capacity(dev, MAX17205_CHAN_MIX_CAPACITY, &dest->mix_capacity_mAh)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205_read_capacity(dev, SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY, &dest->reported_capacity_mAh)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    /* state of charge */
    if( (r = max17205_read_time(dev, SENSOR_CHAN_GAUGE_TIME_TO_EMPTY, &dest->time_to_empty_seconds)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205_read_time(dev, SENSOR_CHAN_GAUGE_TIME_TO_FULL, &dest->time_to_full_seconds)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    if( (r = max17205_read_percentage(dev, MAX17205_CHAN_AVAILABLE_SOC, &dest->available_state_of_charge)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205_read_percentage(dev, MAX17205_CHAN_PRESENT_SOC, &dest->present_state_of_charge)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205_read_percentage(dev, MAX17205_CHAN_REPORTED_SOC, &dest->reported_state_of_charge)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    /* other info */
    if( (r = max17205_read_cycles(dev, &dest->cycles)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    LOG_DBG("");

    LOG_DBG("Temperature (C): Th1: avg = %d, cur = %d, Th2: avg = %d, cur = %d, Int: avg = %d, cur = %d, max = %d, min = %d",
              dest->avg_temp_1_C, dest->temp_1_C, dest->avg_temp_2_C, dest->temp_2_C, dest->avg_int_temp_C, dest->int_temp_C, dest->temp_min_C, dest->temp_max_C);

    LOG_DBG("Voltage (mV):    cell1 = %u, cell2 = %u, vcell = %u, max = %d, min %d, batt = %u",
              dest->v_cell_1_mV, dest->v_cell_2_mV, dest->v_cell_mV, dest->v_cell_max_volt_mV, dest->v_cell_min_volt_mV, dest->batt_mV);

    LOG_DBG("Current (mA):    cur = %d, max = %d, min = %d, avg = %d",
              dest->current_mA, dest->max_current_mA, dest->min_current_mA, dest->avg_current_mA);

    LOG_DBG("Capacity (mAh):  full = %u, available = %u, mix = %u, reported = %u",
              dest->full_capacity_mAh, dest->available_capacity_mAh, dest->mix_capacity_mAh, dest->reported_capacity_mAh);

    LOG_DBG("Time (seconds):  to_empty =%u, to_full = %u",
              dest->time_to_empty_seconds, dest->time_to_full_seconds);

    LOG_DBG("SOC (%%):         reported = %u%%",
              dest->reported_state_of_charge);

    LOG_DBG("cycles = %u", dest->cycles);

    LOG_DBG("");

    return dest->is_data_valid;
}

/**
 * @brief Populates CANOpen data structure values with values from the current battery pack data.
 *
 * @param *pack_data[in] Source of data for populating/publishing pack data.
 */
static void populate_od_pack_data(pack_t *pack)
{
    batt_pack_data_t *pack_data = &pack->data;
    uint8_t state_bitmask = 0;

    if( palReadLine(pack->heater_on) ) {
        state_bitmask |= (1 << STATUS_BIT_HEATER);
    }
    if( palReadLine(pack->line_dchg_dis) ) {
        state_bitmask |= (1 << STATUS_BIT_DCHG_DIS);
    }
    if (palReadLine(pack->line_chg_dis) ) {
        state_bitmask |= (1 << STATUS_BIT_CHG_DIS);
    }
    if( palReadLine(pack->line_dchg_stat) ) {
        state_bitmask |= (1 << STATUS_BIT_DCHG_STAT);
    }
    if( palReadLine(pack->line_chg_stat) ) {
        state_bitmask |= (1 << STATUS_BIT_CHG_STAT);
    }

    if (pack->pack_number == 1) {
        OD_RAM.x4000_pack_1.status = state_bitmask;
        OD_RAM.x4000_pack_1.vbatt = MIN(pack_data->batt_mV, UINT16_MAX);
        OD_RAM.x4000_pack_1.vcell_max = pack_data->v_cell_max_volt_mV;
        OD_RAM.x4000_pack_1.vcell_min = pack_data->v_cell_min_volt_mV;
        OD_RAM.x4000_pack_1.vcell = pack_data->v_cell_mV;
        OD_RAM.x4000_pack_1.vcell_1 = pack_data->v_cell_1_mV;
        OD_RAM.x4000_pack_1.vcell_2 = pack_data->v_cell_2_mV;
        OD_RAM.x4000_pack_1.vcell_avg = pack_data->v_cell_avg_mV;
        OD_RAM.x4000_pack_1.current = CLAMP(pack_data->current_mA, INT16_MIN, INT16_MAX);
        OD_RAM.x4000_pack_1.current_avg = CLAMP(pack_data->avg_current_mA, INT16_MIN, INT16_MAX);
        OD_RAM.x4000_pack_1.current_max = CLAMP(pack_data->max_current_mA, INT16_MIN, INT16_MAX);
        OD_RAM.x4000_pack_1.current_min = CLAMP(pack_data->min_current_mA, INT16_MIN, INT16_MAX);
        OD_RAM.x4000_pack_1.full_capacity = MIN(pack_data->full_capacity_mAh, UINT16_MAX);
        OD_RAM.x4000_pack_1.reported_capacity = MIN(pack_data->reported_capacity_mAh, UINT16_MAX);
        OD_RAM.x4000_pack_1.time_to_empty = MIN(pack_data->time_to_empty_seconds, UINT16_MAX);
        OD_RAM.x4000_pack_1.time_to_full = MIN(pack_data->time_to_full_seconds, UINT16_MAX);
        OD_RAM.x4000_pack_1.cycles = pack_data->cycles;
        OD_RAM.x4000_pack_1.reported_state_of_charge = pack_data->reported_state_of_charge;
        OD_RAM.x4000_pack_1.temperature = CLAMP(pack_data->temp_1_C, INT8_MIN, INT8_MAX);
        OD_RAM.x4000_pack_1.temperature_avg = CLAMP(pack_data->avg_temp_1_C, INT8_MIN, INT8_MAX);
        OD_RAM.x4000_pack_1.temperature_max = pack_data->temp_max_C;
        OD_RAM.x4000_pack_1.temperature_min = pack_data->temp_min_C;
    } else if (pack->pack_number == 2) {
        OD_RAM.x4001_pack_2.status = state_bitmask;
        OD_RAM.x4001_pack_2.vbatt = MIN(pack_data->batt_mV, UINT16_MAX);
        OD_RAM.x4001_pack_2.vcell_max = pack_data->v_cell_max_volt_mV;
        OD_RAM.x4001_pack_2.vcell_min = pack_data->v_cell_min_volt_mV;
        OD_RAM.x4001_pack_2.vcell = pack_data->v_cell_mV;
        OD_RAM.x4001_pack_2.vcell_1 = pack_data->v_cell_1_mV;
        OD_RAM.x4001_pack_2.vcell_2 = pack_data->v_cell_2_mV;
        OD_RAM.x4001_pack_2.vcell_avg = pack_data->v_cell_avg_mV;
        OD_RAM.x4001_pack_2.current = CLAMP(pack_data->current_mA, INT16_MIN, INT16_MAX);
        OD_RAM.x4001_pack_2.current_avg = CLAMP(pack_data->avg_current_mA, INT16_MIN, INT16_MAX);
        OD_RAM.x4001_pack_2.current_max = CLAMP(pack_data->max_current_mA, INT16_MIN, INT16_MAX);
        OD_RAM.x4001_pack_2.current_min = CLAMP(pack_data->min_current_mA, INT16_MIN, INT16_MAX);
        OD_RAM.x4001_pack_2.full_capacity = MIN(pack_data->full_capacity_mAh, UINT16_MAX);
        OD_RAM.x4001_pack_2.reported_capacity = MIN(pack_data->reported_capacity_mAh, UINT16_MAX);
        OD_RAM.x4001_pack_2.time_to_empty = MIN(pack_data->time_to_empty_seconds, UINT16_MAX);
        OD_RAM.x4001_pack_2.time_to_full = MIN(pack_data->time_to_full_seconds, UINT16_MAX);
        OD_RAM.x4001_pack_2.cycles = pack_data->cycles;
        OD_RAM.x4001_pack_2.reported_state_of_charge = pack_data->reported_state_of_charge;
        OD_RAM.x4001_pack_2.temperature = CLAMP(pack_data->temp_1_C, INT8_MIN, INT8_MAX);
        OD_RAM.x4001_pack_2.temperature_avg = CLAMP(pack_data->avg_temp_1_C, INT8_MIN, INT8_MAX);
        OD_RAM.x4001_pack_2.temperature_max = pack_data->temp_max_C;
        OD_RAM.x4001_pack_2.temperature_min = pack_data->temp_min_C;
    } else {
        LOG_DBG("ERROR: pack number not expected: %d", pack->pack_number);
    }
}

static bool are_batteries_critically_low(void)
{
    unsigned int i;

    for (i = 0; i < NPACKS; i++) {
        if ((packs[i].data.v_cell_1_mV < SHUTDOWN_MV) || (packs[i].data.v_cell_2_mV < SHUTDOWN_MV)) {
            LOG_DBG("Batteries are critically low!");
            return true;
        }
    }

    LOG_DBG("Batteries are not critically low");
    return false;
}

static bool check_for_critically_low_batteries(void)
{
    int rc;
    unsigned int i;

    LOG_DBG("Check for critically low batteries");
    for (i = 0; i < NPACKS; i++) {
        if ((rc = max17205_read_voltage(packs[i].dev, MAX17205_CHAN_V_CELL_1, &packs[i].data.v_cell_1_mV)) != 0) {
            packs[i].data.v_cell_1_mV = 0;
        }
        if ((rc = max17205_read_voltage(packs[i].dev, MAX17205_CHAN_V_CELL_2, &packs[i].data.v_cell_2_mV)) != 0) {
            packs[i].data.v_cell_2_mV = 0;
        }
    }
    return are_batteries_critically_low();
}

/* Battery monitoring thread */
void batt_thread_handler(void *p1, void *p2, void *p3)
{
    (void)p1;
    (void)p2;
    (void)p3;
    unsigned int i;
	packs[0].dev = DEVICE_DT_GET(DT_ALIAS(pack1));
	packs[1].dev = DEVICE_DT_GET(DT_ALIAS(pack2));

	if (!device_is_ready(packs[0].dev)) {
		printk("sensor: device pack1 not ready.\n");
		return;
	}

	if (!device_is_ready(packs[1].dev)) {
		printk("sensor: device pack2 not ready.\n");
		return;
	}

    console_init();

#if VERBOSE_DEBUG
    print_batt_hist();
#endif
    find_last_batt_hist();

    heaters_on(false);

    check_for_critically_low_batteries();

    for (i = 0; i < NPACKS; i++) {
        packs[i].updated = nv_ram_write(packs[i].dev, packs[i].name);

        max17205_print_volatile_memory(packs[i].dev);
        max17205_print_nonvolatile_memory(packs[i].dev);
#if VERBOSE_DEBUG
        max17205_read_history(packs[i].dev);
#endif
    }

    // Let MAX17205s startup and run for a bit. Overwriting the MIXCAP and REPCAP values
    // too quickly leads to bad measurements otherwise.
    k_msleep(500);
    for (i = 0; i < NPACKS; i++) {
        load_latest_batt_hist(&packs[i]);
    }

    uint32_t loop = 0;
    int64_t ms = 0;
    int64_t next_hist_update_ms = RUNTIME_BATT_STORE_INTERVAL_MS + k_uptime_get();
    int64_t next_led_update_ms = LED_TOGGLE_INTERVAL_MS + k_uptime_get();
    int64_t next_calib_update_ms = CALIB_INTERVAL_MS + k_uptime_get();

    for (;;) {
        ms = k_uptime_get();

        if (ms >= next_calib_update_ms) {
            next_calib_update_ms = ms + CALIB_INTERVAL_MS;
            manage_calibration();
        }

        if (ms >= next_hist_update_ms) {
            next_hist_update_ms = ms + RUNTIME_BATT_STORE_INTERVAL_MS;
            store_current_batt_hist();
        }

        if (ms >= next_led_update_ms) {
            next_hist_update_ms = ms + LED_TOGGLE_INTERVAL_MS;
            palToggleLine(LINE_LED);
            loop++;
            if (loop % 2 == 0) {
                continue; // we want light to blink at 2Hz, but code to run at 1Hz
            }
            // else falls through
        } else {
            k_msleep(BATT_TASK_SLEEP_INTERVAL_MS);
            continue;
        }

#if DEBUG_PRINT
        LOG_DBG("================================= loop %u, %u.%03u s", loop, ms / 1000, ms % 1000);
#endif

        for (i = 0; i < NPACKS; i++) {
            LOG_DBG("Populating %s Data", packs[i].name);

            if (populate_pack_data(packs[i].dev, &packs[i].data) ) {
                populate_od_pack_data(&packs[i]);
            } else {
                // TODO: do we need to keep this around? Adding the loop results in the wrong error code on pack 2.
            }
        }

#if ENABLE_HEATERS
        if (!are_batteries_critically_low()) {
            run_battery_heating_state_machine();
        }
#endif

        for (i = 0; i < NPACKS; i++) {
            update_battery_charging_state(&packs[i]);
        }
    }
}

void batt_close(void)
{
    LOG_DBG("Terminating battery thread...");

    for (int i = 0; i < NPACKS; i++) {
        // TODO: is there a way to stop a driver?
        // max17205Stop(packs[i].drv);
    }

    palClearLine(LINE_LED);
}
