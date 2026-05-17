#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(display, LOG_LEVEL_INF);

#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>
#include <string.h>
#include <stdarg.h>

#include "display.h"

static const struct device *disp = DEVICE_DT_GET(DT_NODELABEL(ssd1309));
static char display_buff[64] = {0};

int display_init(void)
{
    int ret;

    if (!device_is_ready(disp)) {
        LOG_ERR("Display not ready");
        return -ENODEV;
    }

    if (display_set_pixel_format(disp, PIXEL_FORMAT_MONO10) != 0) {
        if (display_set_pixel_format(disp, PIXEL_FORMAT_MONO01) != 0) {
            LOG_ERR("Failed to set Display Format");
            return -EINVAL;
        }
        LOG_DBG("Set Display format to MONO01");
    } else {
        LOG_DBG("Set Display format to MONO10");
    }

    struct display_capabilities caps;
    display_get_capabilities(disp, &caps);
    LOG_DBG("Display caps: %dx%d, format=%d",
        caps.x_resolution, caps.y_resolution, caps.current_pixel_format);

    ret = cfb_framebuffer_init(disp);
    if (ret < 0) {
        LOG_ERR("Failed to initialize framebuffer: %d", ret);
        return ret;
    }

    ret = cfb_framebuffer_clear(disp, true);
    if (ret < 0) {
        LOG_ERR("Failed to clear framebuffer: %d", ret);
        return ret;
    }

    ret = display_blanking_off(disp);
    if (ret < 0) {
        LOG_ERR("Failed to turn off display blanking: %d", ret);
        return ret;
    }

    LOG_INF("Display initialized successfully");
    return 0;
}

int display_string(const char *fmt, ...)
{
    char buf[64];
    int ret;

    va_list args;
    va_start(args, fmt);
    ret = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    cfb_framebuffer_clear(disp, false);
    ret = cfb_print(disp, buf, 0, 0);
    if (ret < 0) {
        LOG_ERR("Failed to print string: %d", ret);
        return ret;
    }

    cfb_framebuffer_finalize(disp);
    return 0;
}

int display_update_row(uint8_t row, const char *fmt, ...)
{
    if (row >= 8) {
        LOG_ERR("Row %d out of bounds", row);
        return -EINVAL;
    }

    char buf[16];
    int ret;

    va_list args;
    va_start(args, fmt);
    ret = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (ret >= 16) {
        LOG_WRN("Formatted string truncated to fit row: %s", buf);
    }

    memcpy((void *)(display_buff + row * 16), buf, sizeof(buf));

    ret = cfb_print(disp, buf, 0, row * 8);
    if (ret < 0) {
        LOG_ERR("Failed to print string: %d", ret);
        return ret;
    }

    cfb_framebuffer_finalize(disp);
    return 0;
}

int display_clear_text(void)
{
    cfb_framebuffer_clear(disp, true);
    memset((void *)display_buff, 0, sizeof(display_buff));
    cfb_framebuffer_finalize(disp);
    return 0;
}

