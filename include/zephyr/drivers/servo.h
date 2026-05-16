#include <zephyr/device.h>

typedef int (*servo_set_position_t)(const struct device *dev,
                                    int32_t angle_mdeg);
typedef int (*servo_get_position_t)(const struct device *dev,
                                    int32_t *angle_mdeg);
typedef int (*servo_set_speed_t)(const struct device *dev,
                                 uint16_t speed);
typedef int (*servo_set_time_t)(const struct device *dev,
                                uint16_t time_ms);
typedef int (*servo_ping_t)(const struct device *dev);
typedef int (*servo_enable_torque_t)(const struct device *dev);
typedef int (*servo_disable_torque_t)(const struct device *dev);

__subsystem struct servo_driver_api {
  servo_set_position_t set_position;
  servo_get_position_t get_position;
  servo_set_speed_t set_speed;
  servo_set_time_t set_time;
  servo_ping_t ping;
  servo_enable_torque_t enable_torque;
  servo_disable_torque_t disable_torque;
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

static inline int servo_set_speed(const struct device *dev,
                                  uint16_t speed) {
  const struct servo_driver_api *api = dev->api;
  if (api->set_speed == NULL) {
    return -ENOSYS;
  }
  return api->set_speed(dev, speed);
}

static inline int servo_set_time(const struct device *dev,
                                 uint16_t time_ms) {
  const struct servo_driver_api *api = dev->api;
  if (api->set_time == NULL) {
    return -ENOSYS;
  }
  return api->set_time(dev, time_ms);
}

static inline int servo_ping(const struct device *dev) {
  const struct servo_driver_api *api = dev->api;
  if (api->ping == NULL) {
    return -ENOSYS;
  }
  return api->ping(dev);
}

static inline int servo_enable_torque(const struct device *dev) {
  const struct servo_driver_api *api = dev->api;
  if (api->enable_torque == NULL) {
    return -ENOSYS;
  }
  return api->enable_torque(dev);
}

static inline int servo_disable_torque(const struct device *dev) {
  const struct servo_driver_api *api = dev->api;
  if (api->disable_torque == NULL) {
    return -ENOSYS;
  }
  return api->disable_torque(dev);
}
