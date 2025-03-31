#include "mqtt_room.h"

#ifdef USE_ESP32

namespace esphome {
namespace mqtt_room {
static const char *const TAG = "mqtt_room";

void MqttRoomTracker::set_rssi_sensor(sensor::Sensor *rssi_sensor) {
  this->rssi_sensor_ = rssi_sensor;
  this->rssi_sensor_->add_on_state_callback([this](float rssi) { this->update_distance_sensor(rssi); });
}

void MqttRoomTracker::set_distance_sensor(sensor::Sensor *distance_sensor) {
  this->distance_sensor_ = distance_sensor;
  distance_sensor->add_on_state_callback(
      [this](float distance) { this->mqtt_room_->send_tracker_update(this->device_id_, this->name_, distance); });
}

void MqttRoomTracker::dump_config() {
  ESP_LOGCONFIG(TAG, "MQTT Room Tracker:");
  ESP_LOGCONFIG(TAG, "  Device ID: '%s'", this->device_id_.c_str());
  if (!this->name_.empty())
    ESP_LOGCONFIG(TAG, "  Name: '%s'", this->name_.c_str());
  LOG_SENSOR("  ", "RSSI", this->rssi_sensor_);
  LOG_SENSOR("  ", "Distance", this->distance_sensor_);
}

void MqttRoomTracker::update_rssi_sensor(int rssi, int signal_power) {
  if (signal_power != 0) {
    this->signal_power_ = signal_power;
  }

  if (this->rssi_sensor_ == nullptr) {
    this->update_distance_sensor(rssi);
  } else {
    this->rssi_sensor_->publish_state(rssi);
  }
}

void MqttRoomTracker::update_distance_sensor(int rssi) {
  float ratio = (this->signal_power_ - rssi) / (50.0f);
  float distance = pow(10, ratio);

  if (this->distance_sensor_ == nullptr) {
    this->mqtt_room_->send_tracker_update(this->device_id_, this->name_, distance);
  } else {
    this->distance_sensor_->publish_state(distance);
  }
}

void MqttRoom::dump_config() {
  ESP_LOGCONFIG(TAG, "MQTT Room:");
  ESP_LOGCONFIG(TAG, "  MQTT topic: '%s'", this->mqtt_topic_.c_str());
}

void MqttRoom::send_tracker_update(std::string &id, std::string &name, float distance) {
  ESP_LOGD(TAG, "'%s': Sending state %f m with 2 decimals of accuracy", id.c_str(), distance);
  distance = std::round(distance * 100) / 100;

  mqtt::global_mqtt_client->publish_json(this->mqtt_topic_, [=](ArduinoJson::JsonObject root) -> void {
    root["distance"] = distance;
    root["id"] = id;
    root["name"] = (name.empty()) ? id : name;
  });
}

bool MqttRoom::parse_device(const esp32_ble_tracker::ESPBTDevice &device) {
  if (device.get_name().empty()) {
    return false;
  }
  std::string id = std::string("name:") + MqttRoom::format_device_name(device.get_name());
  int signal_power = (!device.get_tx_powers().empty()) ? -65 + device.get_tx_powers().at(0) : 0;
  int rssi = device.get_rssi();
  ESP_LOGD(TAG, "BLE Device found with id: '%s', signal_power: '%d', rssi: '%d'", id.c_str(), signal_power, rssi);

  for (MqttRoomTracker *tracker : this->trackers_) {
    if (tracker->get_device_id() == id) {
      tracker->update_rssi_sensor(rssi, signal_power);
      return true;
    }
  }

  return false;
}

std::string MqttRoom::format_device_name(const std::string &device_name) {
  auto str = device_name;
  bool prev_is_space = false;

  // Remove all double spaces from the string
  str.erase(std::unique(str.begin(), str.end(), [](char lhs, char rhs) { return (lhs == ' ') && (rhs == ' '); }), str.end());
  std::transform(str.begin(), str.end(), str.begin(), [](char c) {
    if (c == ' ')
      return '-';
    return (char) std::tolower(c);
  });

  return str.substr(str.find_first_not_of('-'), str.find_last_not_of('-') + 1);
}

std::string MqttRoom::format_device_address(const uint8_t *device_address) {
  char mac[13];
  snprintf(mac, sizeof(mac), "%02x%02x%02x%02x%02x%02x", device_address[0], device_address[1], device_address[2],
           device_address[3], device_address[4], device_address[5]);
  return mac;
}

}  // namespace mqtt_room
}  // namespace esphome

#endif
