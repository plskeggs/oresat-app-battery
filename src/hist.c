#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>

#include "hist.h"
#include "batt.h"
#include "calib.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(max17205, CONFIG_SENSOR_LOG_LEVEL);

TODO:
- port flash access to one of the Zephyr flash subsystems
- use Kconfigs in place of various configuration macros (DEBUG_PRINT, VERBOSE_DEBUG, HIST_STORE_PROMPT, etc.)

// raw register values to store at runtime per pack;
// restoring after a reset should result in accurate
// fuel gauge estimates
typedef struct __attribute__((packed)) runtime_pack_data {
    uint16_t mixcap; // MAX17205_AD_MIXCAP
    uint16_t repcap; // MAX17205_AD_REPCAP
} runtime_pack_data_t;

// entry to append to already-written flash at next update interval
typedef struct __attribute__((packed)) runtime_battery_data {
    runtime_pack_data_t rt_packs[NPACKS];
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
        LOG_DBG("   pack %u: mixcap:0x%04X, repcap:0x%04X", j + 1, data->rt_packs[j].mixcap, data->rt_packs[j].repcap);
    }
}

#if VERBOSE_DEBUG
void print_batt_hist(void)
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

void find_last_batt_hist(void)
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

static void utilize_batt_hist(pack_t *pack, runtime_battery_data_t *dest)
{
    unsigned int i = pack->pack_number - 1;
    uint16_t tmp;
    int rc;

    tmp = MIN(dest->rt_packs[i].mixcap, CELL_CAPACITY_MAH_RAW);
    rc = max17205_reg_write(pack->dev, MAX17205_AD_MIXCAP, tmp);
    if (rc) {
        LOG_DBG("Error writing AD_MIXCAP");
    }
    tmp = MIN(dest->rt_packs[i].repcap, CELL_CAPACITY_MAH_RAW);
    rc = max17205_reg_write(pack->dev, MAX17205_AD_REPCAP, tmp);
    if (rc) {
        LOG_DBG("Error writing AD_REPCAP");
    }
}

void load_latest_batt_hist(pack_t *pack)
{
    if (last_valid_history_entry == NULL) {
        LOG_DBG("No latest battery history to load");
        return;
    }

    LOG_DBG("Loading entry to pack %u:", pack->pack_number);
    print_runtime_entry(dest, pack->pack_number - 1, 1, NULL);

    utilize_batt_hist(pack, last_valid_history_entry);
}

static void create_batt_hist(pack_t *pack, runtime_battery_data_t *dest)
{
    unsigned int i = pack->pack_number - 1;
    uint16_t tmp;
    int rc;

    rc = max17205_reg_read(pack->dev, MAX17205_AD_MIXCAP, &tmp);
    if (rc) {
        dest->rt_packs[i].mixcap = 0;
    } else {
        // clamp to design limit to handle case where full pack is in storage,
        // self discharges, then is charged later -- MAX17205 will then make max cap
        // higher than it should be
        dest->rt_packs[i].mixcap = MIN(tmp, CELL_CAPACITY_MAH_RAW);
    }
    rc = max17205_reg_read(pack->dev, MAX17205_AD_REPCAP, &tmp);
    if (rc) {
        dest->rt_packs[i].repcap = 0;
    } else {
        dest->rt_packs[i].repcap = MIN(tmp, CELL_CAPACITY_MAH_RAW);
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

bool store_current_batt_hist(void)
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
        create_batt_hist(get_pack(i), &new_data);
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

