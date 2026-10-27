#include "yunmai_scale.h"
#include "esphome/core/log.h"

#ifdef ARDUINO_ARCH_ESP32

namespace esphome {
namespace yunmai_scale {

static const char *const TAG = "yunmai_scale";

void YunmaiScale::dump_config() {
  ESP_LOGCONFIG(TAG, "Yunmai Scale");
  LOG_SENSOR("  ", "Weight", this->weight_);
  LOG_SENSOR("  ", "Impedance", this->impedance_);
}

bool YunmaiScale::parse_device(const esp32_ble_tracker::ESPBTDevice &device) {
  if (device.address_uint64() != this->address_) {
    //ESP_LOGVV(TAG, "parse_device(): unknown MAC address.");
    return false;
  }
  ESP_LOGV(TAG, "parse_device(): MAC address %s found.", device.address_str().c_str());

  bool success = false;
  for (auto &service_data : device.get_manufacturer_datas()) {//此处，改为manufacturer
    auto res = parse_header(service_data);
    if (!res.has_value()) {
      continue;
    }
    if (!(parse_message(service_data.data, *res))) {
      continue;
    }
    if (!(report_results(res, device.address_str()))) {
      continue;
    }
    if (res->weight.has_value() && this->weight_ != nullptr)
      this->weight_->publish_state(*res->weight);
    if (res->impedance.has_value() && this->impedance_ != nullptr)
      this->impedance_->publish_state(*res->impedance);
    success = true;
  }

  return success;
}

optional<ParseResult> YunmaiScale::parse_header(const esp32_ble_tracker::ServiceData &service_data) {
  ParseResult result;
  if (!service_data.uuid.contains(0xZZ, 0xYY)) { //此处，UUID为MAC地址后两个字节，可改为自动截取，逆序
    ESP_LOGD(TAG, "parse_header(): no service data UUID magic bytes.");
    return {};
  }

  return result;
}

bool YunmaiScale::parse_message(const std::vector<uint8_t> &message, ParseResult &result) {

  const uint8_t *data = message.data();
  const int data_length = 14; //此处，云麦的消息长度是14

  if (message.size() != data_length) {
    ESP_LOGD(TAG, "parse_message(): payload has wrong size (%d)!", message.size());
    return false;
  }

  bool is_Stabilized = (data[8] == 3) ? true : false;//此处，可信度标识
  bool loadRemoved = (data[8] == 0) ? true : false;//此处，城中结束

  // weight, 2 bytes, 16-bit  unsigned integer, 1 kg
  const int16_t weight = uint16_t(data[10]) | (uint16_t(data[9]) << 8);//位置
    result.weight = weight * 0.01f ;  // unit 'kg'//单位本就是公斤

  // impedance, 2 bytes, 16-bit
  const int16_t impedance = uint16_t(data[12]) | (uint16_t(data[11]) << 8);//位置
  result.impedance = impedance;

  if (!is_Stabilized || loadRemoved || impedance == 0 || impedance >= 3000) {
    return false;
  }

  return true;
}

bool YunmaiScale::report_results(const optional<ParseResult> &result, const std::string &address) {
  if (!result.has_value()) {
    ESP_LOGD(TAG, "report_results(): no results available.");
    return false;
  }

  ESP_LOGD(TAG, "Got Yunmai Scale (%s):", address.c_str());

  if (result->weight.has_value()) {
    ESP_LOGD(TAG, "  Weight: %.2fkg", *result->weight);
  }
  if (result->impedance.has_value()) {
    ESP_LOGD(TAG, "  Impedance: %.0fohm", *result->impedance);
  }

  return true;
}

}  // namespace yunmai_scale
}  // namespace esphome

#endif
