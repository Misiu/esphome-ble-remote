#include <map>
#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#ifdef USE_API
#include "esphome/components/api/custom_api_device.h"
#endif
#include "hid_parser.h"

#ifdef USE_ESP32

namespace esphome {
namespace ble_client_hid {

namespace espbt=esphome::esp32_ble_tracker;

enum class HIDState {
  // Initial state
  INIT,
  SETUP,

  HID_SERVICE_FOUND,

  NO_HID_SERVICE,
  // Client is coonnected
  BLE_CONNECTED,
  // Start reading relevant client characteristics
  READING_CHARS,
  // Finished reaading client characteristics
  READ_CHARS,
  // Configure ble client with read chars e. g. register fr notify
  CONFIGURING,

  NOTIFICATIONS_REGISTERING,

  NOTIFICATIONS_REGISTERED,

  CONN_PARAMS_UPDATING,

  CONFIGURED,
  
};

class GATTReadData {
  public:
    GATTReadData(uint16_t handle, uint8_t *value, uint16_t value_len){
      this->handle_ = handle;
      this->value_len_ = value_len;
      this->value_ = new uint8_t[value_len];
      memcpy(this->value_, value, sizeof(uint8_t) * value_len);
    }
    ~GATTReadData(){
      delete value_;
    }
  public:
    uint8_t *value_;
    uint16_t value_len_;
    uint16_t handle_;
};

#ifdef USE_API
class BLEClientHID : public Component, public api::CustomAPIDevice, public ble_client::BLEClientNode {
#else
class BLEClientHID : public Component, public ble_client::BLEClientNode {
#endif
 public:
  void loop() override;
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

  void dump_config() override;
  void schedule_read_char(ble_client::BLECharacteristic *characteristic, const char *name = "unknown");
  void on_gatt_read_finished(GATTReadData *data);
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;
  void read_client_characteristics();
  float get_setup_priority() const override { return setup_priority::AFTER_BLUETOOTH; }
  void register_last_event_usage_text_sensor(text_sensor::TextSensor *last_event_usage_text_sensor);
  void register_last_event_code_text_sensor(text_sensor::TextSensor *last_event_code_text_sensor);
  void register_last_event_value_sensor(sensor::Sensor *last_event_value_sensor);
  void configure_hid_client();
  
 protected:
  void send_input_report_event(esp_ble_gattc_cb_param_t *p_data);
  uint8_t *parse_characteristic_data(ble_client::BLEService *service, uint16_t uuid);
  HIDReportMap* hid_report_map = nullptr;
  std::vector<ble_client::BLECharacteristic *> characteristics;
  std::vector<uint16_t> handles_registered_for_notify;
  std::map<uint16_t, GATTReadData *> handles_to_read;
  std::map<uint16_t, uint8_t> handle_report_id;
  text_sensor::TextSensor *last_event_usage_text_sensor = nullptr;
  text_sensor::TextSensor *last_event_code_text_sensor = nullptr;
  sensor::Sensor *last_event_value_sensor = nullptr;
  HIDState hid_state = HIDState::INIT;
  uint8_t handles_waiting_for_notify_registration = 0;
  esp_ble_conn_update_params_t preferred_conn_params = {0};
  // Keepalive: every KEEPALIVE_INTERVAL_MS, write "Exit Suspend" (0x01) then
  // immediately "Suspend" (0x00) to the HID Control Point characteristic.
  //
  // Exit Suspend resets the remote's application-level idle timer (preventing
  // the 5-minute HCI reason-0x13 self-disconnect).  Immediately writing
  // Suspend afterwards returns the device to suspend mode so that the next
  // button press triggers its LED feedback (the remote flashes the LED when
  // it wakes from suspend to deliver a HID report to the host).
  //
  // Both writes use Write Without Response (ESP_GATT_WRITE_TYPE_NO_RSP).
  // This is critical: GATT reads (used previously) require the remote to send
  // an ATT Read Response.  If the BLE connection drops while a read is in
  // flight, the remote is left with a queued but undelivered response.  When
  // a new connection arrives the remote's GATT stack is in an inconsistent
  // state and crashes (requiring a battery pull to recover).  Write Without
  // Response has no such pending-state problem.
  static constexpr uint32_t KEEPALIVE_INTERVAL_MS = 90000;  // 90 seconds
  uint16_t hid_control_point_handle_ = 0;  // HID CP for keepalive writes
  uint32_t last_keepalive_ms_ = 0;
};

}  // namespace ble_client_hid
}  // namespace esphome
#endif
