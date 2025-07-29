#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

extern "C" {
#include "BareDriver/platform/inc/vl53lx_platform.h"
#include "BareDriver/platform/inc/vl53lx_platform_init.h"
#include "BareDriver/core/inc/vl53lx_api.h"
}

namespace esphome {
namespace vl53l3cx {

class VL53L3CXComponent;

class VL53L3CXDistanceSensor : public sensor::Sensor {
 public:
  explicit VL53L3CXDistanceSensor(VL53L3CXComponent *parent) : parent_(parent) {}
  VL53L3CXComponent *parent_;
};

class VL53L3CXPresenceBinarySensor : public binary_sensor::BinarySensor {
 public:
  explicit VL53L3CXPresenceBinarySensor(VL53L3CXComponent *parent) : parent_(parent) {}
  VL53L3CXComponent *parent_;
};

class VL53L3CXComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_distance_sensor(VL53L3CXDistanceSensor *sensor) { distance_sensor_ = sensor; }
  void set_presence_sensor(VL53L3CXPresenceBinarySensor *sensor) { presence_sensor_ = sensor; }

  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  VL53LX_Dev_t dev_{};

 protected:
  VL53L3CXDistanceSensor *distance_sensor_{nullptr};
  VL53L3CXPresenceBinarySensor *presence_sensor_{nullptr};
};

}  // namespace vl53l3cx
}  // namespace esphome

