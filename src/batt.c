#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/console/console.h>
#include <canopennode.h>
#include <OD.h>
#include <oresat.h>

#include "batt.h"
#include "max17205.h"
#include "CANopen.h"
#include "chtime.h"
#include "oresat_f0.h"
#include "flash_f0.h"
#include "crc.h"
#include <sys/param.h>
#include <string.h>

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

// Full battery detection thresholds
#define BATT_FULL_THRESHOLD_MV 8000
#define EOC_THRESHOLD_MA 50
#define CELL_CAPACITY_MAH 2600
#define CELL_CAPACITY_MAH_RAW 0x1450

#define MAX_HIST_STORE_RETRIES 4
#define RUNTIME_BATT_STORE_INTERVAL_MS 300000U // 5 minutes

#define ARRAY_LEN(x) (sizeof(x)/sizeof(x[0]))
#define CLAMP(val, low, high) MIN(MAX(val, low), high)

#define NCELLS          2U          /* Number of cells per pack */
#define NPACKS          2U          /* Number of packs */

// Some of the code and data below requires 2 packs of 2 cells. Any other configuration may require changes.
STATIC_ASSERT(NCELLS == 2);
STATIC_ASSERT(NPACKS == 2);

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

static const max17205_regval_t batt_cfg[] = {
    {MAX17205_AD_PACKCFG, PACKCFG},
    {MAX17205_AD_NRSENSE, MAX17205_RSENSE2REG(10000U)},
    {MAX17205_AD_CONFIG, MAX17205_CONFIG_TEN | MAX17205_CONFIG_ETHRM}
};

typedef struct {
    bool is_data_valid;

    uint32_t batt_mV;
    uint16_t v_cell_max_volt_mV;
    uint16_t v_cell_min_volt_mV;
    uint16_t v_cell_mV;
    uint16_t v_cell_1_mV;
    uint16_t v_cell_2_mV;
    uint16_t v_cell_avg_mV;

    int32_t current_mA;
    int32_t avg_current_mA;
    int32_t max_current_mA;
    int32_t min_current_mA;

    uint32_t full_capacity_mAh;
    uint32_t reported_capacity_mAh;

    // the next 2 are not reported over CAN
    uint32_t available_capacity_mAh;
    uint32_t mix_capacity_mAh;

    uint32_t time_to_empty_seconds;
    uint32_t time_to_full_seconds;

    uint16_t cycles; // count

    uint8_t reported_state_of_charge; //Percent

    // the next 2 are not reported over CAN
    uint8_t available_state_of_charge; //Percent
    uint8_t present_state_of_charge; //Percent

    int16_t int_temp_C;
    int16_t avg_int_temp_C;
    int8_t temp_max_C;
    int8_t temp_min_C;

    // the next 4 are not reported over CAN
    int16_t temp_1_C;
    int16_t temp_2_C;
    int16_t avg_temp_1_C;
    int16_t avg_temp_2_C;
} batt_pack_data_t;

#if ENABLE_HEATERS
static battery_heating_state_machine_state_t current_battery_state_machine_state = BATTERY_STATE_MACHINE_STATE_NOT_HEATING;
#endif

// All state for a pack is contained in this structure.
typedef struct pack {
    bool init;
    bool updated;
    MAX17205Driver drvr;
    MAX17205Config conf;
    batt_pack_data_t data;
    ioline_t heater_on;
    ioline_t line_dchg_dis;
    ioline_t line_chg_dis;
    ioline_t line_dchg_stat;
    ioline_t line_chg_stat;
    uint8_t pack_number;
    char *name;
} pack_t;

// All packs defined here, including non-zero initial data.
static pack_t packs[NPACKS] = {
    {.conf.i2cp = &I2CD1,
     .conf.regcfg = batt_cfg,
     .conf.rsense_uOhm = 10000,
     .heater_on = LINE_HEATER_ON_1,
     .line_dchg_dis = LINE_DCHG_DIS_PK1,
     .line_chg_dis = LINE_CHG_DIS_PK1,
     .line_dchg_stat = LINE_DCHG_STAT_PK1,
     .line_chg_stat = LINE_CHG_STAT_PK1,
     .pack_number = 1,
     .name = "Pack 1"},
    {.conf.i2cp = &I2CD2,
     .conf.regcfg = batt_cfg,
     .conf.rsense_uOhm = 10000,
     .heater_on = LINE_HEATER_ON_2,
     .line_dchg_dis = LINE_DCHG_DIS_PK2,
     .line_chg_dis = LINE_CHG_DIS_PK2,
     .line_dchg_stat = LINE_DCHG_STAT_PK2,
     .line_chg_stat = LINE_CHG_STAT_PK2,
     .pack_number = 2,
     .name = "Pack 2"}
};

// raw register values to store at runtime per pack;
// restoring after a reset should result in accurate
// fuel gauge estimates
typedef struct __attribute__((packed)) runtime_pack_data {
    uint16_t mixcap; // MAX17205_AD_MIXCAP
    uint16_t repcap; // MAX17205_AD_REPCAP
} runtime_pack_data_t;

// entry to append to already-written flash at next update interval
typedef struct __attribute__((packed)) runtime_battery_data {
    runtime_pack_data_t packs[NPACKS];
    uint16_t rst_cycle;         // number of reset cycles so far
    uint16_t minute : 12;       // number of minutes so far during a cycle
    uint16_t unused : 3;        // must be 0
    uint16_t estimated : 1;     // set to 1 if previous value was invalid or none found in flash
    uint16_t crc;               // crc calculated over all fields prior to this field
} runtime_battery_data_t;

#define NUM_BATT_HIST_ENTRIES ((STM32F093_FLASH_PAGE_SIZE) / sizeof(runtime_battery_data_t))

// the flash3 section is defined in app_battery's linker script
extern const void __flash3_size__;
#define FLASH3_SIZE ((const uint32_t)(&__flash3_size__))
runtime_battery_data_t *battery_history = (runtime_battery_data_t *) __flash3_base__;
static runtime_battery_data_t *last_valid_history_entry; // pointer to final entry with good CRC
static runtime_battery_data_t *last_empty_history_entry; // pointer to first empty entry

static uint16_t reset_cycle_count;

static void print_runtime_entry(runtime_battery_data_t *data, unsigned int start, unsigned int count, const char *prefix)
{
#if DEBUG_PRINT
    (void)data;
#endif
    LOG_DBG("%srst_cycle=%d, minute=%d, unused=%d, estimated=%d, crc=0x%08X",
            prefix ? prefix : "",
            data->rst_cycle, data->minute, data->unused, data->estimated, data->crc);
    for (unsigned int j = start; j < start + count; j++) {
        LOG_DBG("   pack %u: mixcap:0x%04X, repcap:0x%04X", j + 1, data->packs[j].mixcap, data->packs[j].repcap);
    }
}

#if VERBOSE_DEBUG
static void print_batt_hist(void)
{
    runtime_battery_data_t *data = battery_history;
    char prefix[10];

    LOG_DBG("Runtime battery history:\r\nentry size=%lu, entry count=%u",
              sizeof(runtime_battery_data_t), NUM_BATT_HIST_ENTRIES);

    for (unsigned int i = 0; i < NUM_BATT_HIST_ENTRIES; i++) {
        if (!flashIsErasedF091((flashaddr_t)data, sizeof(runtime_battery_data_t))) {
            snprintk(prefix, sizeof(prefix), "%d. ", i);
            print_runtime_entry(data, 0, NPACKS, prefix);
        } else {
            LOG_DBG("%u. Found erased entry. Done.", i);
            break;
        }
        data++;
    }
}
#endif // VERBOSE_DEBUG

static void find_last_batt_hist(void)
{
    runtime_battery_data_t *data = battery_history;
    last_valid_history_entry = NULL;
    last_empty_history_entry = NULL;

    // Search forward through stored history for the most recent entry that has
    // a correct CRC.
    for (unsigned int i = 0; i < NUM_BATT_HIST_ENTRIES; i++) {
        if (flashIsErasedF091((flashaddr_t)data, sizeof(runtime_battery_data_t))) {
            last_empty_history_entry = data;
            break;
        }
        uint32_t check_crc = crc32((uint8_t *)data, sizeof(*data) - sizeof(data->crc), 0);

        if (check_crc == data->crc) {
            last_valid_history_entry = data;
        } else {
            LOG_DBG("CRC failure on entry %u", i);
        }
        data++;
    }

    // we found a good entry, so save a copy in RAM
    if (last_valid_history_entry != NULL) {
        LOG_DBG("Selected last valid runtime entry:");
        print_runtime_entry(last_valid_history_entry, 0, NPACKS, NULL);

        reset_cycle_count = last_valid_history_entry->rst_cycle;

        // We have started a new cycle, so advance this counter once
        reset_cycle_count++;
    }
}

static void load_latest_batt_hist(pack_t *pack)
{
    if (last_valid_history_entry == NULL) {
        LOG_DBG("No latest battery history to load");
        return;
    }

    msg_t r;
    uint16_t tmp;

    LOG_DBG("Loading entry to pack %u:", pack->pack_number);
    print_runtime_entry(last_valid_history_entry, pack->pack_number - 1, 1, NULL);
    tmp = MIN(last_valid_history_entry->packs[pack->pack_number - 1].mixcap, CELL_CAPACITY_MAH_RAW);
    r = max17205Write(&pack->drvr, MAX17205_AD_MIXCAP, tmp);
    if (r != MSG_OK) {
        LOG_DBG("Error writing AD_MIXCAP");
    }
    tmp = MIN(last_valid_history_entry->packs[pack->pack_number - 1].repcap, CELL_CAPACITY_MAH_RAW);
    r = max17205Write(&pack->drvr, MAX17205_AD_REPCAP, tmp);
    if (r != MSG_OK) {
        LOG_DBG("Error writing AD_REPCAP");
    }
}

static bool add_next_batt_hist(runtime_battery_data_t *new_data)
{
    if (last_empty_history_entry < &battery_history[NUM_BATT_HIST_ENTRIES]) {
        if (flashWriteF091((flashaddr_t)last_empty_history_entry, (uint8_t *)new_data, sizeof(runtime_battery_data_t)) == FLASH_RETURN_SUCCESS) {
            last_valid_history_entry = last_empty_history_entry++;
            return true;
        } else {
            LOG_DBG("Error writing to flash");
        }
    }
    // history storage is full or error writing flash, so let caller recover
    last_valid_history_entry = NULL;
    last_empty_history_entry = battery_history;
    return false;
}

static bool store_current_batt_hist(void)
{
    runtime_battery_data_t new_data;
    unsigned int i;
    msg_t r;
    uint16_t tmp;

#if HIST_STORE_PROMPT

    LOG_DBG("********** Store batt_hist e(rase), y(es), n(o)? ");
    uint8_t ch = console_getchar(); // TODO: Zephyr console does not have a read timeout; switch to shell

    LOG_DBG("");
    if (ch == 'e') {
        LOG_DBG("Erasing *************");
        flashEraseF091((flashaddr_t)battery_history, STM32F093_FLASH_PAGE_SIZE);
        return true;
    } else if (ch != 'y') {
        LOG_DBG("Not storing ************");
        return true;
    }
#endif // HIST_STORE_PROMPT
    for (i = 0; i < NPACKS; i++) {
        r = max17205Read(&packs[i].drvr, MAX17205_AD_MIXCAP, &tmp);
        if (r != MSG_OK) {
            new_data.packs[i].mixcap = 0;
        } else {
            // clamp to design limit to handle case where full pack is in storage,
            // self discharges, then is charged later -- MAX17205 will then make max cap
            // higher than it should be
            new_data.packs[i].mixcap = MIN(tmp, CELL_CAPACITY_MAH_RAW);
        }
        r = max17205Read(&packs[i].drvr, MAX17205_AD_REPCAP, &tmp);
        if (r != MSG_OK) {
            new_data.packs[i].repcap = 0;
        } else {
            new_data.packs[i].repcap = MIN(tmp, CELL_CAPACITY_MAH_RAW);
        }
    }
    new_data.rst_cycle = reset_cycle_count;
    new_data.minute = TIME_I2S(chVTGetSystemTime()) / 60;
    new_data.unused = 0;
    new_data.estimated = false;
    new_data.crc = crc32((uint8_t *)&new_data, sizeof(new_data) - sizeof(new_data.crc), 0);
    LOG_DBG("Storing new runtime battery entry:");
    print_runtime_entry(&new_data, 0, NPACKS, NULL);
    for (i = 0; i < MAX_HIST_STORE_RETRIES; i++) {
        if (add_next_batt_hist(&new_data)) {
            LOG_DBG("Done.");
            return true;
        } else {
            LOG_DBG("Retry %d. Erasing page.", i + 1);
            flashEraseF091((flashaddr_t)battery_history, STM32F093_FLASH_PAGE_SIZE);
        }
    }
    LOG_DBG("ERROR: Unable to store current battery history.");
    return false;
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
static void run_battery_heating_state_machine(void) {
    unsigned int i;

    for (i = 0; i < NPACKS; i++) {
        if (!packs[i].data.is_data_valid) {
            LOG_DBG("FAILSAFE: ");
            heaters_on(false);
            //CO_errorReport(CO->em, CO_EM_GENERIC_ERROR, CO_EMC_HARDWARE, BATTERY_OD_ERROR_INFO_CODE_PACK_FAIL_SAFE_HEATING);
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
static void update_battery_charging_state(const pack_t *pack) {
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
        //CO_errorReport(CO->em, CO_EM_GENERIC_ERROR, CO_EMC_HARDWARE, BATTERY_OD_ERROR_INFO_CODE_PACK_FAIL_SAFE_CHARGING);
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
 * @param *driver[in] The MAX17 driver object to use to query pack data from
 * @param *dest[out] Destination into which to store pack data currently tracked in the MAX17
 *
 * @return true on success, false otherwise
 */
static bool populate_pack_data(MAX17205Driver *driver, batt_pack_data_t *dest) {
    msg_t r = 0;
    memset(dest, 0, sizeof(*dest));

    if( driver->state != MAX17205_READY ) {
        return false;
    }

    dest->is_data_valid = true;


    if( (r = max17205ReadAverageTemperature(driver, MAX17205_AD_TEMP1, &dest->temp_1_C)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205ReadAverageTemperature(driver, MAX17205_AD_TEMP2, &dest->temp_2_C)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205ReadAverageTemperature(driver, MAX17205_AD_INTTEMP, &dest->int_temp_C)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205ReadAverageTemperature(driver, MAX17205_AD_AVGTEMP1, &dest->avg_temp_1_C)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205ReadAverageTemperature(driver, MAX17205_AD_AVGTEMP2, &dest->avg_temp_2_C)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205ReadAverageTemperature(driver, MAX17205_AD_AVGINTTEMP, &dest->avg_int_temp_C)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205ReadMaxMinTemperature(driver, &dest->temp_max_C, &dest->temp_min_C)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    /* Record pack and cell voltages to object dictionary */
    if( (r = max17205ReadVoltage(driver, MAX17205_AD_AVGCELL1, &dest->v_cell_1_mV)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    if( (r = max17205ReadVoltage(driver, MAX17205_AD_AVGVCELL, &dest->v_cell_avg_mV)) != MSG_OK ) {
    dest->is_data_valid = false;
   }
    if( (r = max17205ReadVoltage(driver, MAX17205_AD_VCELL, &dest->v_cell_mV)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205ReadBatt(driver, &dest->batt_mV)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    if( dest->is_data_valid ) {
        dest->v_cell_2_mV = dest->batt_mV - dest->v_cell_1_mV;
    }

    if( (r = max17205ReadMaxMinVoltage(driver, &dest->v_cell_max_volt_mV, &dest->v_cell_min_volt_mV)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    if( (r = max17205ReadCurrent(driver, MAX17205_AD_CURRENT, &dest->current_mA)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205ReadCurrent(driver, MAX17205_AD_AVGCURRENT, &dest->avg_current_mA)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    if( (r = max17205ReadMaxMinCurrent(driver, &dest->max_current_mA, &dest->min_current_mA)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    /* capacity */
    if( (r = max17205ReadCapacity(driver, MAX17205_AD_FULLCAPREP, &dest->full_capacity_mAh)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205ReadCapacity(driver, MAX17205_AD_AVCAP, &dest->available_capacity_mAh)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205ReadCapacity(driver, MAX17205_AD_MIXCAP, &dest->mix_capacity_mAh)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205ReadCapacity(driver, MAX17205_AD_REPCAP, &dest->reported_capacity_mAh)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    /* state of charge */
    if( (r = max17205ReadTime(driver, MAX17205_AD_TTE, &dest->time_to_empty_seconds)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205ReadTime(driver, MAX17205_AD_TTF, &dest->time_to_full_seconds)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    if( (r = max17205ReadPercentage(driver, MAX17205_AD_AVSOC, &dest->available_state_of_charge)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205ReadPercentage(driver, MAX17205_AD_VFSOC, &dest->present_state_of_charge)) != MSG_OK ) {
        dest->is_data_valid = false;
    }
    if( (r = max17205ReadPercentage(driver, MAX17205_AD_REPSOC, &dest->reported_state_of_charge)) != MSG_OK ) {
        dest->is_data_valid = false;
    }

    /* other info */
    if( (r = max17205ReadCycles(driver, &dest->cycles)) != MSG_OK ) {
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
 * Helper function to trigger write of volatile memory on MAX71205 chip.
 * Returns true if NV RAM was written, false otherwise.
 */
static bool nv_ram_write(MAX17205Driver *devp, const char *pack_str) {
#if DEBUG_PRINT
    (void)pack_str;
#endif
    LOG_DBG("\r\nEnsure NV RAM settings are correct for %s", pack_str);

    bool all_elements_match = false;
    msg_t r = max17205ValidateRegisters(devp, batt_nv_programing_cfg, ARRAY_LEN(batt_nv_programing_cfg), &all_elements_match);
    if (r != MSG_OK) {
        return false;
    }

    if (all_elements_match) {
        LOG_DBG("All NV RAM elements already match expected values...");
        return false;
    } else {
        LOG_DBG("One or more NV RAM elements don't match expected values...");
    }

    r = max17205WriteRegisters(devp, batt_nv_programing_cfg, ARRAY_LEN(batt_nv_programing_cfg));
    if (r != MSG_OK) {
        LOG_DBG("Failed to write new NV RAM reg values\n");
        return false;
    }
    LOG_DBG("Successfully wrote new NV RAM reg values");

    all_elements_match = false;
    r = max17205ValidateRegisters(devp, batt_nv_programing_cfg, ARRAY_LEN(batt_nv_programing_cfg), &all_elements_match);
    if (r != MSG_OK) {
        return false;
    }
    if (!all_elements_match) {
        LOG_DBG("NV RAM elements failed to update after write.");
        return false;
    }

    LOG_DBG("All NV RAM elements now match expected values.");

    //Now make the chip use the changes written to the shadow registers.
    max17205FirmwareReset(devp);
    return true;
}

/**
 * Helper function to trigger write of volatile memory on MAX71205 chip.
 * Returns true if NV was written, false otherwise.
 */
#if ENABLE_NV_WRITE_PROMPT
static bool prompt_nv_write(MAX17205Driver *devp, const char *pack_str) {
    LOG_DBG("\r\nWrite NV RAM to NV%s", pack_str);

    uint16_t masking_register = 0;
    uint8_t num_writes_left = 0;
    if (max17205ReadNVWriteCountMaskingRegister(devp, &masking_register, &num_writes_left) == MSG_OK) {
        LOG_DBG("Memory Update Masking of register is 0x%X, num_writes_left = %u",
            masking_register, num_writes_left);
    }

    if (num_writes_left > 0) {
        // Answer n to just use the changes in the volatile registers
        LOG_DBG("Write NV memory on MAX17205 for %s ? y/n? ", pack_str);
        uint8_t ch = console_getchar(); // TODO: Zephyr console does not have a read timeout; switch to shell

        LOG_DBG("");

        if (ch == 'y') {
            LOG_DBG("Attempting to write non volatile memory on MAX17205...");
            chThdSleepMilliseconds(50);

            if (max17205NonvolatileBlockProgram(devp) == MSG_OK ) {
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
static bool update_learning_complete(MAX17205Driver *devp, pack_t *pack) {
    batt_pack_data_t *pack_data = &pack->data;
    bool ret = false;

    if ((pack_data->batt_mV > BATT_FULL_THRESHOLD_MV) &&
        (pack_data->avg_current_mA < EOC_THRESHOLD_MA) &&
        (pack_data->avg_current_mA >= 0) &&
        (pack_data->full_capacity_mAh >= CELL_CAPACITY_MAH) ) {

        LOG_DBG("Pack %d seems full", pack->pack_number);
        uint8_t state;
        msg_t r = max17205ReadLearnState(devp, &state);
        if (r != MSG_OK) {
            LOG_DBG("Error reading learn state");
            return ret;
        }
        LOG_DBG("Learning state = %u", state);
        if (state == MAX17205_LEARN_COMPLETE) {
            LOG_DBG("Learning is already complete.");
        } else {
            r = max17205WriteLearnState(devp, MAX17205_LEARN_COMPLETE);
            if (r != MSG_OK) {
                LOG_DBG("Error writing learn state");
                return ret;
            }
            r = max17205ReadLearnState(devp, &state);
            if (r != MSG_OK) {
                LOG_DBG("Error checking learn state");
                return ret;
            }
            if (state != 7) {
                LOG_DBG("Error setting state = %u; is %u", MAX17205_LEARN_COMPLETE, state);
                return ret;
            }
            LOG_DBG("Learning state set = %u", state);
            ret = true;
        }
        pack_data->mix_capacity_mAh = pack_data->reported_capacity_mAh = pack_data->full_capacity_mAh;
        if ( (r = max17205WriteCapacity(devp, MAX17205_AD_MIXCAP, pack_data->mix_capacity_mAh)) != MSG_OK ) {
            LOG_DBG("Failed to write MIXCAP");
        } else if ( (r = max17205WriteCapacity(devp, MAX17205_AD_REPCAP, pack_data->reported_capacity_mAh)) != MSG_OK ) {
            LOG_DBG("Failed to write REPCAP");
        } else {
            LOG_DBG("Mixcap and repcap set to %u", pack_data->full_capacity_mAh);
        }
        return ret;
    }
    return false;
}
#endif // ENABLE_LEARN_COMPLETE


/**
 * @brief Populates CANOpen data structure values with values from the current battery pack data.
 *
 * @param *pack_data[in] Source of data for populating/publishing pack data.
 */
static void populate_od_pack_data(pack_t *pack) {
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
    msg_t r;
    unsigned int i;

    LOG_DBG("Check for critically low batteries");
    for (i = 0; i < NPACKS; i++) {
        if ((r = max17205ReadVoltage(&packs[i].drvr, MAX17205_AD_AVGCELL1, &packs[i].data.v_cell_1_mV)) != MSG_OK) {
            packs[i].data.v_cell_1_mV = 0;
        }
        if ((r = max17205ReadBatt(&packs[i].drvr, &packs[i].data.batt_mV)) != MSG_OK) {
            packs[i].data.batt_mV = 0;
        }
        packs[i].data.v_cell_2_mV = packs[i].data.batt_mV - packs[i].data.v_cell_1_mV;
    }
    return are_batteries_critically_low();
}

static void manage_calibration(void)
{
#if DEBUG_PRINT
    unsigned int i;
#if ENABLE_NV_WRITE_PROMPT
    bool nv_written = false;
#endif

    for (i = 0; i < NPACKS; i++) {
        LOG_DBG("%s:", packs[i].name);
        max17205PrintVolatileMemory(&packs[i].drvr);

#if ENABLE_LEARN_COMPLETE
        // If ENABLE_LEARN_COMPLETE=1, ENABLE_NV_MEMORY_UPDATE_CODE=1 and DEBUG_PRINT are all enabled, we will only prompt to update
        // NV when learning is complete. If ENABLE_LEARN_COMPLETE is not 1 but the others are, then we will only prompt to update
        // NV if there is a change to NV RAM required (done prior to the main loop).
        packs[i].updated = update_learning_complete(&packs[i].drvr, &packs[i]);
#endif
#if ENABLE_NV_WRITE_PROMPT
        if (packs[i].init && packs[i].updated) {
            nv_written |= prompt_nv_write(&packs[i].drvr, packs[i].name);
            packs[i].updated = false;
        }
#endif
    }

#if ENABLE_NV_WRITE_PROMPT
    if (nv_written) {
        LOG_DBG("Done with NV RAM update code, set ENABLE_NV_MEMORY_UPDATE_CODE=0 and re-write firmware.");
        for (;;) {
            LOG_DBG(".");
            chThdSleepMilliseconds(1000);
        }
    }
#endif
#endif
}

/* Battery monitoring thread */
void batt_thread_handler(void *p1, void *p2, void *p3)
{
    (void)p1;
    (void)p2;
    (void)p3;
    unsigned int i;

#if 0
    LOG_DBG("Starting main loop");
    while (canopennode_is_running()) {
        timepoint = sys_timepoint_calc(K_MSEC(1000));
        board_sensors_fill_od();
        k_sleep(sys_timepoint_timeout(timepoint));
    }
#endif

    console_init();

#if VERBOSE_DEBUG
    print_batt_hist();
#endif
    find_last_batt_hist();

    for (i = 0; i < NPACKS; i++) {
        max17205ObjectInit(&packs[i].drvr);
        packs[i].init = max17205Start(&packs[i].drvr, &packs[i].conf);
        LOG_DBG("max17205Start(%s) = %u", packs[i].name, packs[i].init);
    }

    heaters_on(false);

    check_for_critically_low_batteries();

    for (i = 0; i < NPACKS; i++) {
        packs[i].updated = nv_ram_write(&packs[i].drvr, packs[i].name);

        max17205PrintVolatileMemory(&packs[i].drvr);
        max17205PrintNonvolatileMemory(&packs[i].drvr);
#if VERBOSE_DEBUG
        max17205ReadHistory(&packs[i].drvr);
#endif
    }

    // Let MAX17205s startup and run for a bit. Overwriting the MIXCAP and REPCAP values
    // too quickly leads to bad measurements otherwise.
    chThdSleepMilliseconds(500);
    for (i = 0; i < NPACKS; i++) {
        load_latest_batt_hist(&packs[i]);
    }

    uint32_t loop = 0;
    uint32_t ms = 0;
    uint32_t next_update_ms = RUNTIME_BATT_STORE_INTERVAL_MS;

    while (!chThdShouldTerminateX()) {
        loop++;
        chThdSleepMilliseconds(500);
        palToggleLine(LINE_LED);
        if (loop % 2 == 0) {
            continue; // we want light to blink at 2Hz, but code to run at 1Hz
        }

        ms = TIME_I2MS(chVTGetSystemTime());

        if (ms >= next_update_ms) {
            next_update_ms = ms + RUNTIME_BATT_STORE_INTERVAL_MS;
            store_current_batt_hist();
        }

#if DEBUG_PRINT
        LOG_DBG("================================= loop %u, %u.%03u s", loop, ms / 1000, ms % 1000);
#endif

        for (i = 0; i < NPACKS; i++) {
            LOG_DBG("Populating %s Data", packs[i].name);

            if (populate_pack_data(&packs[i].drvr, &packs[i].data) ) {
                populate_od_pack_data(&packs[i]);
            } else {
                // TODO: do we need to keep this around? Adding the loop results in the wrong error code on pack 2.
                // CO_errorReport(CO->em, CO_EM_GENERIC_ERROR, CO_EMC_COMMUNICATION, BATTERY_OD_ERROR_INFO_CODE_PACK_1_COMM_ERROR);
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

        if ((loop % 240) == 1) {
            manage_calibration();
        }
    }

    LOG_DBG("Terminating battery thread...");

    for (i = 0; i < NPACKS; i++) {
        max17205Stop(&packs[i].drvr);
    }

    palClearLine(LINE_LED);
    chThdExit(MSG_OK);
}
