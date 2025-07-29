#include "vl53l3cx.h"
#include "esphome/core/log.h"

namespace esphome {
namespace vl53l3cx {

static const char *const TAG = "vl53l3cx";
static VL53L3CXComponent *global_vl53l3cx = nullptr;

void VL53L3CXComponent::setup() {
  global_vl53l3cx = this;
  // convert 7-bit address to 8-bit as used by ST driver
  this->dev_.i2c_slave_address = static_cast<uint8_t>(this->address_ << 1);
  this->dev_.comms_type = 1;  // I2C
  this->dev_.comms_speed_khz = 400;  // assume fast mode
  VL53LX_Error status = VL53LX_platform_init(&this->dev_, this->dev_.i2c_slave_address,
                                             this->dev_.comms_type, this->dev_.comms_speed_khz);
  if (status != VL53LX_ERROR_NONE) {
    ESP_LOGE(TAG, "platform init failed: %d", status);
    this->mark_failed();
    return;
  }
  status = VL53LX_WaitDeviceBooted(&this->dev_);
  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_DataInit(&this->dev_);
  if (status == VL53LX_ERROR_NONE)
    status = VL53LX_StartMeasurement(&this->dev_);
  if (status != VL53LX_ERROR_NONE) {
    ESP_LOGE(TAG, "sensor init failed: %d", status);
    this->mark_failed();
  }
}

void VL53L3CXComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "VL53L3CX:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Component failed to initialize");
  }
}

void VL53L3CXComponent::update() {
  if (this->is_failed())
    return;

  uint8_t ready = 0;
  VL53LX_Error status = VL53LX_GetMeasurementDataReady(&this->dev_, &ready);
  if (status != VL53LX_ERROR_NONE || !ready) {
    return;
  }

  VL53LX_MultiRangingData_t data;
  status = VL53LX_GetMultiRangingData(&this->dev_, &data);
  if (status != VL53LX_ERROR_NONE) {
    ESP_LOGW(TAG, "read error: %d", status);
    return;
  }

  if (this->distance_sensor_ != nullptr) {
    if (data.NumberOfObjectsFound > 0)
      this->distance_sensor_->publish_state(data.RangeData[0].RangeMilliMeter / 1000.0f);
    else
      this->distance_sensor_->publish_state(NAN);
  }

  if (this->presence_sensor_ != nullptr) {
    this->presence_sensor_->publish_state(data.NumberOfObjectsFound > 0);
  }

  VL53LX_ClearInterruptAndStartMeasurement(&this->dev_);
}

}  // namespace vl53l3cx
}  // namespace esphome

