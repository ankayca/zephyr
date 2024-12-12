#define DT_DRV_COMPAT ams_tcs3472

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
LOG_MODULE_REGISTER(tcs3472, CONFIG_SENSOR_LOG_LEVEL);

/* Register addresses - Command bit (0x80) is included */
#define TCS3472_COMMAND_BIT     0x80
#define TCS3472_REG_ENABLE      (TCS3472_COMMAND_BIT | 0x00)
#define TCS3472_REG_ATIME       (TCS3472_COMMAND_BIT | 0x01)
#define TCS3472_REG_CONTROL     (TCS3472_COMMAND_BIT | 0x0F)
#define TCS3472_REG_ID          (TCS3472_COMMAND_BIT | 0x12)
#define TCS3472_REG_STATUS      (TCS3472_COMMAND_BIT | 0x13)
#define TCS3472_REG_CDATAL      (TCS3472_COMMAND_BIT | 0x14)

/* Enable register bits */
#define TCS3472_ENABLE_PON      0x01    /* Power ON */
#define TCS3472_ENABLE_AEN      0x02    /* RGBC Enable */

/* Status register bits */
#define TCS3472_STATUS_AVALID   0x01    /* Data valid */

struct tcs3472_config {
    struct i2c_dt_spec i2c;
};

struct tcs3472_data {
    uint16_t sample_crgb[4];  /* Clear, Red, Green, Blue */
};
/* Add attr_set and attr_get */
static int tcs3472_attr_set(const struct device *dev, enum sensor_channel chan,
                           enum sensor_attribute attr,
                           const struct sensor_value *val)
{
    LOG_DBG("Channel: %d, attribute: %d, val: %d.%d",
            chan, attr, val->val1, val->val2);
    return -ENOTSUP;
}

static int tcs3472_attr_get(const struct device *dev, enum sensor_channel chan,
                           enum sensor_attribute attr,
                           struct sensor_value *val)
{
    LOG_DBG("Channel: %d, attribute: %d", chan, attr);
    return -ENOTSUP;
}
/* Add trigger_set */
static int tcs3472_trigger_set(const struct device *dev,
                              const struct sensor_trigger *trig,
                              sensor_trigger_handler_t handler)
{
    LOG_DBG("Trigger channel: %d, type: %d", trig->chan, trig->type);
    return -ENOTSUP;
}
static int tcs3472_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    const struct tcs3472_config *cfg = dev->config;
    struct tcs3472_data *data = dev->data;
    uint8_t status;
    int ret;

    /* Check status */
    ret = i2c_reg_read_byte_dt(&cfg->i2c, TCS3472_REG_STATUS, &status);
    if (ret < 0) {
        LOG_ERR("Could not read status");
        return ret;
    }

    if (!(status & TCS3472_STATUS_AVALID)) {
        k_sleep(K_MSEC(50));  // Wait for data to be ready
    }

    /* Read all color channels */
    ret = i2c_burst_read_dt(&cfg->i2c, TCS3472_REG_CDATAL,
                           (uint8_t *)data->sample_crgb,
                           sizeof(data->sample_crgb));
    if (ret < 0) {
        LOG_ERR("Could not read sensor channels");
        return ret;
    }

    /* Convert from little-endian */
    for (int i = 0; i < 4; i++) {
        data->sample_crgb[i] = sys_le16_to_cpu(data->sample_crgb[i]);
    }

    LOG_DBG("Raw values - C: %d, R: %d, G: %d, B: %d", 
            data->sample_crgb[0], data->sample_crgb[1], 
            data->sample_crgb[2], data->sample_crgb[3]);

    return 0;
}

static int tcs3472_channel_get(const struct device *dev,
                              enum sensor_channel chan,
                              struct sensor_value *val)
{
    struct tcs3472_data *data = dev->data;
    uint16_t value;

    switch (chan) {
    case SENSOR_CHAN_RED:
        value = data->sample_crgb[1];
        val->val1 = value;
        val->val2 = 0;
        LOG_DBG("Red: %d", value);
        break;

    case SENSOR_CHAN_GREEN:
        value = data->sample_crgb[2];
        val->val1 = value;
        val->val2 = 0;
        LOG_DBG("Green: %d", value);
        break;

    case SENSOR_CHAN_BLUE:
        value = data->sample_crgb[3];
        val->val1 = value;
        val->val2 = 0;
        LOG_DBG("Blue: %d", value);
        break;

    case SENSOR_CHAN_LIGHT:
        value = data->sample_crgb[0];
        val->val1 = value;
        val->val2 = 0;
        LOG_DBG("Clear: %d", value);
        break;

    default:
        return -ENOTSUP;
    }

    return 0;
}

static int tcs3472_init(const struct device *dev)
{
    const struct tcs3472_config *cfg = dev->config;
    uint8_t id;
    int ret;
    
    LOG_INF("TCS3472: Init start");
    
    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("TCS3472: I2C bus not ready");
        return -ENODEV;
    }
    
    /* Read ID register */
    ret = i2c_reg_read_byte_dt(&cfg->i2c, TCS3472_REG_ID, &id);
    if (ret < 0) {
        LOG_ERR("TCS3472: Could not read ID: %d", ret);
        return ret;
    }
    LOG_INF("TCS3472: ID = 0x%02x", id);

    /* Power ON */
    ret = i2c_reg_write_byte_dt(&cfg->i2c, TCS3472_REG_ENABLE, TCS3472_ENABLE_PON);
    if (ret < 0) {
        LOG_ERR("Failed to power on device");
        return ret;
    }

    /* Wait for 2.4ms after PON */
    k_sleep(K_MSEC(3));

    /* Enable RGBC and Power */
    ret = i2c_reg_write_byte_dt(&cfg->i2c, TCS3472_REG_ENABLE, 
                               TCS3472_ENABLE_PON | TCS3472_ENABLE_AEN);
    if (ret < 0) {
        LOG_ERR("Failed to enable RGBC");
        return ret;
    }

    /* Set integration time to 24ms */
    ret = i2c_reg_write_byte_dt(&cfg->i2c, TCS3472_REG_ATIME, 0xF6);
    if (ret < 0) {
        LOG_ERR("Failed to set integration time");
        return ret;
    }

    /* Set gain to 4x */
    ret = i2c_reg_write_byte_dt(&cfg->i2c, TCS3472_REG_CONTROL, 0x01);
    if (ret < 0) {
        LOG_ERR("Failed to set gain");
        return ret;
    }

    return 0;
}

/* Update the API structure */
static const struct sensor_driver_api tcs3472_api = {
    .attr_set = tcs3472_attr_set,
    .attr_get = tcs3472_attr_get,
    .sample_fetch = tcs3472_sample_fetch,
    .channel_get = tcs3472_channel_get,
    .trigger_set = tcs3472_trigger_set,
};
static int cmd_tcs3472_get(const struct shell *sh, size_t argc, char **argv)
{
    const struct device *dev = DEVICE_DT_GET_ANY(ams_tcs3472);
    struct sensor_value red, green, blue, clear;
    int ret;

    if (!device_is_ready(dev)) {
        shell_error(sh, "Device %s not ready.", dev->name);
        return -ENODEV;
    }

    ret = sensor_sample_fetch(dev);
    if (ret < 0) {
        shell_error(sh, "Failed to fetch samples: %d", ret);
        return ret;
    }

    sensor_channel_get(dev, SENSOR_CHAN_RED, &red);
    sensor_channel_get(dev, SENSOR_CHAN_GREEN, &green);
    sensor_channel_get(dev, SENSOR_CHAN_BLUE, &blue);
    sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &clear);

    shell_print(sh, "R:%d G:%d B:%d C:%d", 
                red.val1, green.val1, blue.val1, clear.val1);

    return 0;
}

/* Add shell commands registration */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_tcs3472,
    SHELL_CMD(get, NULL, "Get sensor data", cmd_tcs3472_get),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(tcs3472, &sub_tcs3472, "TCS3472 sensor", NULL);
/* Update the device definition macro */
#define TCS3472_INIT(inst)                                          \
    static struct tcs3472_data tcs3472_data_##inst;                \
    static const struct tcs3472_config tcs3472_config_##inst = {    \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                         \
    };                                                             \
    DEVICE_DT_INST_DEFINE(inst,                                    \
                         tcs3472_init,                             \
                         NULL,                                      \
                         &tcs3472_data_##inst,                     \
                         &tcs3472_config_##inst,                   \
                         POST_KERNEL,                              \
                         CONFIG_SENSOR_INIT_PRIORITY,              \
                         &tcs3472_api);

DT_INST_FOREACH_STATUS_OKAY(TCS3472_INIT)