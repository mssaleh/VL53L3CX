#include "vl53l3cx.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace vl53l3cx {

extern VL53L3CXComponent *global_vl53l3cx;  // defined in vl53l3cx.cpp

}  // namespace vl53l3cx
}  // namespace esphome

using namespace esphome;
using namespace esphome::vl53l3cx;

extern "C" {

VL53LX_Error VL53LX_CommsInitialise(VL53LX_Dev_t *pdev, uint8_t comms_type, uint16_t comms_speed_khz) {
  (void) pdev;
  (void) comms_type;
  (void) comms_speed_khz;
  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_CommsClose(VL53LX_Dev_t *pdev) {
  (void) pdev;
  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WriteMulti(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count) {
  if (global_vl53l3cx == nullptr)
    return VL53LX_ERROR_CONTROL_INTERFACE;
  if (global_vl53l3cx->write_register16(index, pdata, count) != i2c::ERROR_OK)
    return VL53LX_ERROR_CONTROL_INTERFACE;
  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_ReadMulti(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count) {
  if (global_vl53l3cx == nullptr)
    return VL53LX_ERROR_CONTROL_INTERFACE;
  if (global_vl53l3cx->read_register16(index, pdata, count) != i2c::ERROR_OK)
    return VL53LX_ERROR_CONTROL_INTERFACE;
  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WrByte(VL53LX_Dev_t *pdev, uint16_t index, uint8_t data) {
  return VL53LX_WriteMulti(pdev, index, &data, 1);
}

VL53LX_Error VL53LX_WrWord(VL53LX_Dev_t *pdev, uint16_t index, uint16_t data) {
  uint8_t buffer[2];
  buffer[0] = data >> 8;
  buffer[1] = data & 0xFF;
  return VL53LX_WriteMulti(pdev, index, buffer, 2);
}

VL53LX_Error VL53LX_WrDWord(VL53LX_Dev_t *pdev, uint16_t index, uint32_t data) {
  uint8_t buffer[4];
  buffer[0] = data >> 24;
  buffer[1] = (data >> 16) & 0xFF;
  buffer[2] = (data >> 8) & 0xFF;
  buffer[3] = data & 0xFF;
  return VL53LX_WriteMulti(pdev, index, buffer, 4);
}

VL53LX_Error VL53LX_RdByte(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata) {
  return VL53LX_ReadMulti(pdev, index, pdata, 1);
}

VL53LX_Error VL53LX_RdWord(VL53LX_Dev_t *pdev, uint16_t index, uint16_t *pdata) {
  uint8_t buffer[2];
  VL53LX_Error res = VL53LX_ReadMulti(pdev, index, buffer, 2);
  if (res == VL53LX_ERROR_NONE)
    *pdata = (uint16_t(buffer[0]) << 8) | buffer[1];
  return res;
}

VL53LX_Error VL53LX_RdDWord(VL53LX_Dev_t *pdev, uint16_t index, uint32_t *pdata) {
  uint8_t buffer[4];
  VL53LX_Error res = VL53LX_ReadMulti(pdev, index, buffer, 4);
  if (res == VL53LX_ERROR_NONE)
    *pdata = (uint32_t(buffer[0]) << 24) | (uint32_t(buffer[1]) << 16) |
             (uint32_t(buffer[2]) << 8) | buffer[3];
  return res;
}

VL53LX_Error VL53LX_WaitUs(VL53LX_Dev_t *pdev, int32_t wait_us) {
  (void) pdev;
  delayMicroseconds(wait_us);
  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitMs(VL53LX_Dev_t *pdev, int32_t wait_ms) {
  (void) pdev;
  delay(wait_ms);
  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz) {
  *ptimer_freq_hz = 1000;
  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTimerValue(int32_t *ptimer_count) {
  *ptimer_count = millis();
  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioSetMode(uint8_t pin, uint8_t mode) { return VL53LX_ERROR_NONE; }
VL53LX_Error VL53LX_GpioSetValue(uint8_t pin, uint8_t value) { return VL53LX_ERROR_NONE; }
VL53LX_Error VL53LX_GpioGetValue(uint8_t pin, uint8_t *value) { return VL53LX_ERROR_NONE; }
VL53LX_Error VL53LX_GpioXshutdown(uint8_t value) { return VL53LX_ERROR_NONE; }
VL53LX_Error VL53LX_GpioCommsSelect(uint8_t value) { return VL53LX_ERROR_NONE; }
VL53LX_Error VL53LX_GpioPowerEnable(uint8_t value) { return VL53LX_ERROR_NONE; }
VL53LX_Error VL53LX_GpioInterruptEnable(void (*function)(void), uint8_t edge) { return VL53LX_ERROR_NONE; }
VL53LX_Error VL53LX_GpioInterruptDisable(void) { return VL53LX_ERROR_NONE; }

}

