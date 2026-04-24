#include <zephyr/device.h>

typedef int (*servo_set_position_t)(const struct device *dev,
                                    int32_t angle_mdeg);
typedef int (*servo_get_position_t)(const struct device *dev,
                                    int32_t *angle_mdeg);

__subsystem struct servo_driver_api {
  servo_set_position_t set_position;
  servo_get_position_t get_position;
};

static inline int servo_set_position(const struct device *dev,
                                     int32_t angle_mdeg) {
  const struct servo_driver_api *api = dev->api;
  return api->set_position(dev, angle_mdeg);
}

static inline int servo_get_position(const struct device *dev,
                                     int32_t *angle_mdeg) {
  const struct servo_driver_api *api = dev->api;
  return api->get_position(dev, angle_mdeg);
}
