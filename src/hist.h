#ifndef _HIST_H_
#define _HIST_H_

#ifdef __cplusplus
extern "C" {
#endif

void print_batt_hist(void);
void find_last_batt_hist(void);
void load_latest_batt_hist(pack_t *pack);
bool add_next_batt_hist(runtime_battery_data_t *new_data);
bool store_current_batt_hist(void);

#ifdef __cplusplus
}
#endif

#endif /* _HIST_H_ */


