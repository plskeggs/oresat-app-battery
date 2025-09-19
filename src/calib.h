#ifndef _CALIB_H_
#define _CALIB_H_

#ifdef __cplusplus
extern "C" {
#endif

int max17205_reg_read(const struct device *dev, uint16_t addr, int16_t *val);
int max17205_reg_write(const struct device *dev, uint16_t addr, int16_t val);

int max17205_print_volatile_memory(const struct device *dev);
int max17205_print_nonvolatile_memory(const struct device *dev);

int max17205_read_history(const struct device *dev);

bool nv_ram_write(MAX17205Driver *devp, const char *pack_str);
void manage_calibration(void);

const char* max17205_reg_to_str(const uint16_t reg);

int read_channel_int8_t(const struct device *dev, enum sensor_channel type, int8_t *dest);
int read_channel_int16_t(const struct device *dev, enum sensor_channel type, int16_t *dest);
int read_channel_int32_t(const struct device *dev, enum sensor_channel type, int32_t *dest);

static inline int max17205_read_average_temperature(const struct device *dev, enum sensor_channel type, int16_t *dest)
{
    return read_channel_int16_t(dev, type, dest);
}

static inline int max17205_read_max_temperature(const struct device *dev, int8_t *dest)
{
    return read_channel_int8_t(dev, MAX17205_CHAN_TEMP_MAX, dest);
}

static inline int max17205_read_min_temperature(const struct device *dev, int8_t *dest)
{
    return read_channel_int8_t(dev, MAX17205_CHAN_TEMP_MIN, dest);
}

static inline int max17205_read_voltage(const struct device *dev, enum sensor_channel type, uint16_t *dest)
{
    return read_channel_int16_t(dev, type, dest);
}

static inline int max17205_read_batt(const struct device *dev, uint32_t *dest)
{
    return read_channel_int32_t(dev, SENSOR_CHAN_GAUGE_VOLTAGE, dest);
}

static inline int max17205_read_max_voltage(const struct device *dev, uint16_t *dest)
{
    return read_channel_int16_t(dev, MAX17205_CHAN_V_CELL_MAX, dest);
}

static inline int max17205_read_min_voltage(const struct device *dev, uint16_t *dest)
{
    return read_channel_int16_t(dev, MAX17205_CHAN_V_CELL_MIN, dest);
}

static inline int max17205_read_current(const struct device *dev, int16_t *dest)
{
    return read_channel_int16_t(dev, SENSOR_CHAN_CURRENT, dest);
}

static inline int max17205_read_avg_current(const struct device *dev, int16_t *dest)
{
    return read_channel_int16_t(dev, SENSOR_CHAN_GAUGE_AVG_CURRENT, dest);
}

static inline int max17205_read_max_current(const struct device *dev, int16_t *dest)
{
    return read_channel_int16_t(dev, MAX17205_CHAN_CURRENT_MAX, dest);
}

static inline int max17205_read_min_current(const struct device *dev, int16_t *dest)
{
    return read_channel_int16_t(dev, MAX17205_CHAN_CURRENT_MIN, dest);
}

static inline int max17205_read_capacity(const struct device *dev, enum sensor_channel type, uint32_t *dest)
{
    return read_channel_int32_t(dev, type, dest);
}

static inline int max17205_read_time(const struct device *dev, enum sensor_channel type, uint32_t *dest)
{
    return read_channel_int32_t(dev, type, dest);
}

static inline int max17205_read_percentage(const struct device *dev, enum sensor_channel type, uint32_t *dest)
{
    return read_channel_int8_t(dev, type, dest);
}

static inline int max17205_read_cycles(const struct device *dev, uint16_t *dest)
{
    return read_channel_int16_t(dev, SENSOR_CHAN_GAUGE_CYCLE_COUNT, dest);
}

#ifdef __cplusplus
}
#endif

#endif /* _CALIB_H_ */

