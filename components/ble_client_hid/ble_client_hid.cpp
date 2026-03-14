#include "ble_client_hid.h"

#include "usages.h"

#ifdef USE_ESP32

namespace esphome {
namespace ble_client_hid {

static const char *const TAG = "ble_client_hid";

static const std::string EMPTY = "";

static TickType_t last_run = 0;

void BLEClientHID::loop() {
  switch (this->hid_state) {
    case HIDState::BLE_CONNECTED:
      this->read_client_characteristics();  // not instant, finished when
                                            // hid_state = HIDState::READ_CHARS
      this->hid_state = HIDState::READING_CHARS;
      break;
    case HIDState::READ_CHARS:
      this->configure_hid_client();
      this->hid_state = HIDState::NOTIFICATIONS_REGISTERING;
      break;
    case HIDState::NOTIFICATIONS_REGISTERED:
      // Update connection parameters per device's preference.  This is
      // fire-and-forget — we do NOT block on the GAP event because that
      // event may never fire (remote rejects update, event is for another
      // device, etc.).  Move to CONFIGURED immediately so the keepalive
      // loop runs regardless.
      if (this->preferred_conn_params.max_int > 0) {
        esp_ble_gap_update_conn_params(&this->preferred_conn_params);
      }
      this->hid_state = HIDState::CONFIGURED;
      this->node_state = espbt::ClientState::ESTABLISHED;
      this->last_keepalive_ms_ = millis();
      break;
    case HIDState::CONFIGURED: {
      // Periodic keepalive: read a characteristic to prevent the remote from
      // self-disconnecting after an idle timeout (reason 0x13).
      uint32_t now = millis();
      if (this->keepalive_char_handle_ != 0 &&
          (now - this->last_keepalive_ms_) >= KEEPALIVE_INTERVAL_MS) {
        this->last_keepalive_ms_ = now;
        ESP_LOGD(TAG, "Keepalive: reading char handle=0x%04X",
                 this->keepalive_char_handle_);
        esp_ble_gattc_read_char(this->parent()->get_gattc_if(),
                                this->parent()->get_conn_id(),
                                this->keepalive_char_handle_,
                                ESP_GATT_AUTH_REQ_NO_MITM);
      }
      break;
    }
    default:
      break;
  }
}

void BLEClientHID::dump_config() {
  ESP_LOGCONFIG(TAG, "BLE Client HID:");
  ESP_LOGCONFIG(TAG, "  MAC address        : %s",
                this->parent()->address_str());
}

void BLEClientHID::gap_event_handler(esp_gap_ble_cb_event_t event,
  esp_ble_gap_cb_param_t *param) {
   switch (event)
   {
   case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
    // Validate BDA so we don't react to updates for other connected devices.
    if (memcmp(param->update_conn_params.bda, this->parent()->get_remote_bda(), 6) != 0)
      break;
    ESP_LOGI(TAG, "Updated conn params to interval=%.2f ms, latency=%u, timeout=%.1f ms", param->update_conn_params.conn_int * 1.25f, param->update_conn_params.latency, param->update_conn_params.timeout * 10.f);
     break;
   default:
     break;
   }
  }

void BLEClientHID::read_client_characteristics() {
  ESP_LOGD(TAG, "Reading client characteristics");
  using namespace ble_client;
  BLEService *battery_service =
      this->parent()->get_service(ESP_GATT_UUID_BATTERY_SERVICE_SVC);
  BLEService *device_info_service =
      this->parent()->get_service(ESP_GATT_UUID_DEVICE_INFO_SVC);

  BLEService *hid_service = this->parent()->get_service(ESP_GATT_UUID_HID_SVC);
  BLEService *generic_access_service = this->parent()->get_service(0x1800);

  // === GATT Service Discovery Dump ===
  ESP_LOGD(TAG, "=== GATT Services ===");
  struct { BLEService *svc; const char *name; } services[] = {
    {generic_access_service, "GenericAccess(0x1800)"},
    {battery_service,        "Battery(0x180F)"},
    {device_info_service,    "DeviceInfo(0x180A)"},
    {hid_service,            "HID(0x1812)"},
  };
  for (auto &entry : services) {
    if (entry.svc == nullptr) {
      ESP_LOGD(TAG, "  Service %s: not present", entry.name);
      continue;
    }
    ESP_LOGD(TAG, "  Service %s: %d characteristics", entry.name,
             entry.svc->characteristics.size());
    for (auto *chr : entry.svc->characteristics) {
      ESP_LOGD(TAG, "    Char UUID=0x%04X handle=0x%04X props=0x%02X",
               chr->uuid.get_uuid().uuid.uuid16, chr->handle, chr->properties);
      BLEDescriptor *rpt_ref = chr->get_descriptor(ESP_GATT_UUID_RPT_REF_DESCR);
      if (rpt_ref != nullptr) {
        ESP_LOGD(TAG, "      RPT_REF_DESCR handle=0x%04X", rpt_ref->handle);
      }
    }
  }

  if (generic_access_service != nullptr) {
    BLECharacteristic *device_name_char =
        generic_access_service->get_characteristic(
            ESP_GATT_UUID_GAP_DEVICE_NAME);
    BLECharacteristic *pref_conn_params_char = generic_access_service->get_characteristic(ESP_GATT_UUID_GAP_PREF_CONN_PARAM);
    this->schedule_read_char(pref_conn_params_char, "GAP_PREF_CONN_PARAM");
    this->schedule_read_char(device_name_char, "GAP_DEVICE_NAME");
    // Save device name handle as keepalive fallback (always readable).
    if (device_name_char != nullptr &&
        (device_name_char->properties & ESP_GATT_CHAR_PROP_BIT_READ) != 0) {
      this->keepalive_char_handle_ = device_name_char->handle;
    }
  }
  if (device_info_service != nullptr) {
    BLECharacteristic *pnp_id_char =
        device_info_service->get_characteristic(ESP_GATT_UUID_PNP_ID);
    this->schedule_read_char(pnp_id_char, "PNP_ID");
    BLECharacteristic *manufacturer_char =
        device_info_service->get_characteristic(ESP_GATT_UUID_MANU_NAME);
    this->schedule_read_char(manufacturer_char, "MANU_NAME");
    BLECharacteristic *serial_number_char =
        device_info_service->get_characteristic(
            ESP_GATT_UUID_SERIAL_NUMBER_STR);
    this->schedule_read_char(serial_number_char, "SERIAL_NUMBER");
  }
  if (hid_service != nullptr) {
    BLECharacteristic *hid_report_map_char =
        hid_service->get_characteristic(ESP_GATT_UUID_HID_REPORT_MAP);
    this->schedule_read_char(hid_report_map_char, "HID_REPORT_MAP");
    ESP_LOGD(TAG, "Found %d characteristics",
             hid_service->characteristics.size());
    for (auto *chr : hid_service->characteristics) {
      if (chr->uuid.get_uuid().uuid.uuid16 != ESP_GATT_UUID_HID_REPORT) {
        continue;
      }

      bool has_notify = (chr->properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) != 0;
      BLEDescriptor *rpt_ref_desc =
          chr->get_descriptor(ESP_GATT_UUID_RPT_REF_DESCR);
      ESP_LOGD(TAG, "  HID_REPORT char handle=0x%04X props=0x%02X notify=%s rpt_ref=%s",
               chr->handle, chr->properties,
               has_notify ? "yes" : "no",
               rpt_ref_desc != nullptr ? "yes" : "no");

      if (rpt_ref_desc != nullptr) {
        if (esp_ble_gattc_read_char_descr(
                this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                rpt_ref_desc->handle, ESP_GATT_AUTH_REQ_NO_MITM) != ESP_OK) {
          ESP_LOGW(TAG, "scheduling reading of RPT_REF_DESCR failed for handle 0x%04X.",
                   chr->handle);
        }
        this->handles_to_read.insert(
            std::make_pair(rpt_ref_desc->handle, nullptr));
      }
    }
  }
}
void BLEClientHID::on_gatt_read_finished(GATTReadData *data) {
  std::map<uint16_t, GATTReadData *>::iterator itr;
  itr = this->handles_to_read.find(data->handle_);
  // Guard: if this handle is not in the tracked map (e.g. a keepalive read
  // that fired while fully configured), discard the data and return without
  // touching the state machine.
  if (itr == this->handles_to_read.end()) {
    delete data;
    return;
  }
  itr->second = data;
  // check if all handles have been read:
  for (auto const &element : this->handles_to_read) {
    if (element.second == nullptr) {
      return;
    }
  }
  this->hid_state = HIDState::READ_CHARS;
}

void BLEClientHID::gattc_event_handler(esp_gattc_cb_event_t event,
                                       esp_gatt_if_t gattc_if,
                                       esp_ble_gattc_cb_param_t *param) {
  esp_ble_gattc_cb_param_t *p_data = param;
  switch (event) {
    case ESP_GATTC_CONNECT_EVT: {
      auto ret = esp_ble_set_encryption(param->connect.remote_bda,
                                        ESP_BLE_SEC_ENCRYPT);
      if (ret) {
        ESP_LOGE(TAG, "[%d] [%s] esp_ble_set_encryption error, status=%d",
                 this->parent()->get_connection_index(),
                 this->parent()->address_str(), ret);
      }
      esp_gap_conn_params_t params;
      ret = esp_ble_get_current_conn_params(
          this->parent()->get_remote_bda(), &params);
      if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get conn params");
      }
      ESP_LOGI(TAG, "conn params: interval=%u, latency=%u, timeout=%u",
               params.interval, params.latency, params.timeout);
      break;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "[%s] Disconnected!",
               this->parent()->address_str());
      this->status_set_warning("Disconnected");
      // Reset all HID state so reconnection starts cleanly.
      this->hid_state = HIDState::INIT;
      this->handles_waiting_for_notify_registration = 0;
      this->keepalive_char_handle_ = 0;
      this->last_keepalive_ms_ = 0;
      for (auto &kv : this->handles_to_read) {
        delete kv.second;
      }
      this->handles_to_read.clear();
      this->handle_report_id.clear();
      this->handles_registered_for_notify.clear();
      delete this->hid_report_map;
      this->hid_report_map = nullptr;
      break;
    }
    case ESP_GATTC_SEARCH_RES_EVT: {
      if (p_data->search_res.srvc_id.uuid.uuid.uuid16 ==
          ESP_GATT_UUID_HID_SVC) {
        this->hid_state = HIDState::HID_SERVICE_FOUND;
        ESP_LOGD(TAG, "GATT HID service found on device %s",
                 this->parent()->address_str());
      }
      break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      if (this->hid_state != HIDState::HID_SERVICE_FOUND) {
        // service not found
        ESP_LOGW(TAG, "No GATT HID service found on device %s",
                 this->parent()->address_str());
        this->hid_state = HIDState::NO_HID_SERVICE;
        this->status_set_warning("Invalid device config");
        break;
      }
      ESP_LOGD(TAG, "GATTC search finished with status code %d",
               p_data->search_cmpl.status);
      this->hid_state = HIDState::BLE_CONNECTED;
      esp_gap_conn_params_t params;
      esp_err_t ret = esp_ble_get_current_conn_params(
          this->parent()->get_remote_bda(), &params);
      if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get conn params");
      }
      ESP_LOGI(TAG, "conn params: interval=%u, latency=%u, timeout=%u",
               params.interval, params.latency, params.timeout);
      break;
    }
    case ESP_GATTC_READ_CHAR_EVT:
    case ESP_GATTC_READ_DESCR_EVT: {
      if (param->read.conn_id != this->parent()->get_conn_id()) break;
      if (param->read.status != ESP_OK) {
        ESP_LOGW(TAG, "GATTC read failed with status code %d",
                 param->read.status);
        break;
      }
      // All code below runs only for successful reads.
      // Update battery sensor whenever a battery char read completes
      // (covers both keepalive reads and notify-driven initial reads).
      if (event == ESP_GATTC_READ_CHAR_EVT &&
          param->read.handle == this->battery_handle &&
          param->read.value_len > 0 &&
          this->battery_sensor != nullptr) {
        this->battery_sensor->publish_state(param->read.value[0]);
      }
      GATTReadData *data = new GATTReadData(
          param->read.handle, param->read.value, param->read.value_len);
      this->on_gatt_read_finished(data);
      break;
    }
    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.conn_id != this->parent()->get_conn_id()) break;
      if (p_data->notify.handle == this->battery_handle) {
        uint8_t battery_level = p_data->notify.value[0];
        if (this->battery_sensor == nullptr) {
          break;
        }
        this->battery_sensor->publish_state(battery_level);
      } else {
        // has to be hid report
        this->send_input_report_event(p_data);
      }
      break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      if (param->reg_for_notify.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Register for notify failed for handle 0x%04X with status=%d",
                 param->reg_for_notify.handle, param->reg_for_notify.status);
      }
      this->handles_waiting_for_notify_registration--;
      if (this->handles_waiting_for_notify_registration == 0) {
        this->hid_state = HIDState::NOTIFICATIONS_REGISTERED;
      }
      break;
    }
    default: {
      break;
    }
  }
}

void BLEClientHID::send_input_report_event(esp_ble_gattc_cb_param_t *p_data) {
  if (this->hid_report_map == nullptr) {
    ESP_LOGW(TAG, "HID report received but report map not yet initialized, dropping");
    return;
  }
  // Determine report_id from our mapping (avoid default-entry creation via [])
  uint8_t report_id = 0;
  if (this->handle_report_id.count(p_data->notify.handle) > 0) {
    report_id = this->handle_report_id.at(p_data->notify.handle);
  } else {
    ESP_LOGW(TAG, "No report_id mapping for notify handle 0x%04X, defaulting to 0",
             p_data->notify.handle);
  }

  // Log raw notify payload in hex for diagnostics
  if (p_data->notify.value_len > 0) {
    std::string hex;
    char byte_str[5];
    for (uint16_t i = 0; i < p_data->notify.value_len; i++) {
      snprintf(byte_str, sizeof(byte_str), "%02X ", p_data->notify.value[i]);
      hex += byte_str;
    }
    ESP_LOGD(TAG, "HID notify: handle=0x%04X report_id=%d len=%d data=[%s]",
             p_data->notify.handle, report_id, p_data->notify.value_len, hex.c_str());
  }

  uint8_t *data = new uint8_t[p_data->notify.value_len + 1];
  memcpy(data + 1, p_data->notify.value, p_data->notify.value_len);
  data[0] = report_id;
  std::vector<HIDReportItemValue> hid_report_values =
      this->hid_report_map->parse(data);
  if (hid_report_values.size() == 0) {
    delete[] data;
    return;
  }
  for (HIDReportItemValue value : hid_report_values) {
    std::string usage;
    if (USAGE_PAGES.count(value.usage.page) > 0 &&
        USAGE_PAGES.at(value.usage.page).usages_.count(value.usage.usage) > 0) {
      usage = USAGE_PAGES.at(value.usage.page).usages_.at(value.usage.usage);
      ESP_LOGD(TAG, "HID event: page=0x%02X usage=0x%04X value=%d name='%s'",
               value.usage.page, value.usage.usage, value.value, usage.c_str());
    } else {
      usage = std::to_string(value.usage.page) + "_" +
              std::to_string(value.usage.usage);
      ESP_LOGD(TAG, "HID event: page=0x%02X usage=0x%04X value=%d (unknown usage, fallback='%s')",
               value.usage.page, value.usage.usage, value.value, usage.c_str());
    }
    #ifdef USE_API_HOMEASSISTANT_SERVICES
    this->fire_homeassistant_event("esphome.hid_events", {{"usage", usage}, {"value", std::to_string(value.value)}});
    ESP_LOGD(TAG, "Send HID event to HomeAssistant: usage: %s, value: %d", usage.c_str(), value.value);
    #endif
    if(this->last_event_usage_text_sensor != nullptr){
      this->last_event_usage_text_sensor->publish_state(usage);
    }
    if(this->last_event_code_text_sensor != nullptr){
      std::string event_code = std::to_string(value.usage.page) + "_" +
                               std::to_string(value.usage.usage);
      this->last_event_code_text_sensor->publish_state(event_code);
    }
    if (this->last_event_value_sensor != nullptr) {
      this->last_event_value_sensor->publish_state(value.value);
    }
    ESP_LOGI(TAG, "Send HID event to HomeAssistant: usage: %s, value: %d",
             usage.c_str(), value.value);
  }

  delete[] data;
}

void BLEClientHID::register_last_event_value_sensor(
    sensor::Sensor *last_event_value_sensor) {
  this->last_event_value_sensor = last_event_value_sensor;
}

void BLEClientHID::register_battery_sensor(sensor::Sensor *battery_sensor) {
  this->battery_sensor = battery_sensor;
}

void BLEClientHID::register_last_event_usage_text_sensor(
    text_sensor::TextSensor *last_event_usage_text_sensor) {
  this->last_event_usage_text_sensor = last_event_usage_text_sensor;
}

void BLEClientHID::register_last_event_code_text_sensor(
    text_sensor::TextSensor *last_event_code_text_sensor) {
  this->last_event_code_text_sensor = last_event_code_text_sensor;
}

void BLEClientHID::schedule_read_char(
    ble_client::BLECharacteristic *characteristic, const char *name) {
  if (characteristic == nullptr) {
    ESP_LOGW(TAG, "Characteristic '%s': not found on device (nullptr)", name);
    return;
  }
  if ((characteristic->properties & ESP_GATT_CHAR_PROP_BIT_READ) == 0) {
    ESP_LOGW(TAG,
             "Characteristic '%s' (handle=0x%04X): exists but READ property "
             "not set (properties=0x%02X)",
             name, characteristic->handle, characteristic->properties);
    return;
  }
  if (esp_ble_gattc_read_char(
          this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
          characteristic->handle, ESP_GATT_AUTH_REQ_NO_MITM) != ESP_OK) {
    ESP_LOGW(TAG, "read_char failed for '%s' (handle=0x%04X)",
             name, characteristic->handle);
  }
  this->handles_to_read.insert(
      std::make_pair(characteristic->handle, nullptr));
}

uint8_t *BLEClientHID::parse_characteristic_data(
    ble_client::BLEService *service, uint16_t uuid) {
  using namespace ble_client;
  BLECharacteristic *characteristic = service->get_characteristic(uuid);
  if (characteristic == nullptr) {
    ESP_LOGD(TAG, "No characteristic with uuid %#X found on device", uuid);
    return nullptr;
  }
  if (handles_to_read.count(characteristic->handle) >= 1 &&
      handles_to_read.at(characteristic->handle) != nullptr) {
    ESP_LOGD(
        TAG,
        "Characteristic parsed for uuid %#X and handle %#X starts with %#X",
        uuid, characteristic->handle,
        *(handles_to_read.at(characteristic->handle)->value_));
    return handles_to_read.at(characteristic->handle)->value_;
  }
  ESP_LOGD(TAG,
           "Characteristic with uuid %#X and handle %#X not stored in "
           "handles_to_read",
           uuid, characteristic->handle);
  return nullptr;
}

void BLEClientHID::configure_hid_client() {
  using namespace ble_client;
  BLEService *battery_service =
      this->parent()->get_service(ESP_GATT_UUID_BATTERY_SERVICE_SVC);
  BLEService *device_info_service =
      this->parent()->get_service(ESP_GATT_UUID_DEVICE_INFO_SVC);
  BLEService *hid_service = this->parent()->get_service(ESP_GATT_UUID_HID_SVC);
  BLEService *generic_access_service = this->parent()->get_service(0x1800);

  if (generic_access_service != nullptr) {
    uint8_t *t_device_name = this->parse_characteristic_data(
        generic_access_service, ESP_GATT_UUID_GAP_DEVICE_NAME);
    if (t_device_name != nullptr) {
      this->device_name = (const char *)t_device_name;
    } else {
      this->device_name = "Generic";
    }
  }
  if (battery_service != nullptr) {
    BLECharacteristic *battery_level_char =
        battery_service->get_characteristic(ESP_GATT_UUID_BATTERY_LEVEL);
    if (battery_level_char != nullptr &&
        ((battery_level_char->properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) !=
         0)) {
      this->battery_handle = battery_level_char->handle;
      auto status = esp_ble_gattc_register_for_notify(
          this->parent()->get_gattc_if(), this->parent()->get_remote_bda(),
          battery_level_char->handle);
      
      if (status != ESP_OK) {
        ESP_LOGW(TAG, "Register for notify failed for handle %d with status=%d",
                 battery_level_char->handle, status);
      } else {
        this->handles_waiting_for_notify_registration++;
      }
    }
    // Prefer battery char for keepalive reads (also readable when no NOTIFY).
    if (battery_level_char != nullptr &&
        (battery_level_char->properties & ESP_GATT_CHAR_PROP_BIT_READ) != 0) {
      this->keepalive_char_handle_ = battery_level_char->handle;
    }
  }
  if (device_info_service != nullptr) {
    BLECharacteristic *pnp_id_char =
        device_info_service->get_characteristic(ESP_GATT_UUID_PNP_ID);
    if (pnp_id_char != nullptr &&
        this->handles_to_read.count(pnp_id_char->handle) > 0 &&
        this->handles_to_read.at(pnp_id_char->handle) != nullptr) {
      uint8_t *rdata = this->handles_to_read.at(pnp_id_char->handle)->value_;
      this->vendor_id = *((uint16_t *)&rdata[1]);
      this->product_id = *((uint16_t *)&rdata[3]);
      this->version = *((uint16_t *)&rdata[5]);
      ESP_LOGD(TAG, "PNP ID: vendor=0x%04X product=0x%04X version=0x%04X",
               this->vendor_id, this->product_id, this->version);
      delete this->handles_to_read.at(pnp_id_char->handle);
      this->handles_to_read.erase(pnp_id_char->handle);
    } else {
      ESP_LOGW(TAG, "PNP ID characteristic not available or not read");
    }

    uint8_t *t_manufacturer = this->parse_characteristic_data(
        device_info_service, ESP_GATT_UUID_MANU_NAME);
    if (t_manufacturer != nullptr) {
      this->manufacturer = (const char *)t_manufacturer;
    } else {
      this->manufacturer = "Generic";
    }

    uint8_t *t_serial = this->parse_characteristic_data(
        device_info_service, ESP_GATT_UUID_SERIAL_NUMBER_STR);
    if (t_serial != nullptr) {
      this->serial_number = (const char *)t_serial;
    } else {
      this->serial_number = "000000";
    }
  }
  if (hid_service != nullptr) {
    BLECharacteristic *hid_report_map_char =
        hid_service->get_characteristic(ESP_GATT_UUID_HID_REPORT_MAP);
    if (hid_report_map_char != nullptr &&
        this->handles_to_read.count(hid_report_map_char->handle) > 0 &&
        this->handles_to_read.at(hid_report_map_char->handle) != nullptr) {
      ESP_LOGD(TAG, "Parse HID Report Map");
      HIDReportMap::esp_logd_report_map(
          this->handles_to_read.at(hid_report_map_char->handle)->value_,
          this->handles_to_read.at(hid_report_map_char->handle)->value_len_);
      delete this->hid_report_map;
      this->hid_report_map = HIDReportMap::parse_report_map_data(
          this->handles_to_read.at(hid_report_map_char->handle)->value_,
          this->handles_to_read.at(hid_report_map_char->handle)->value_len_);
      ESP_LOGD(TAG, "Parse HID Report Map Done");
    } else {
      ESP_LOGW(TAG, "HID_REPORT_MAP characteristic not available or not read");
    }
    std::vector<BLECharacteristic *> chars = hid_service->characteristics;
    for (BLECharacteristic *hid_char : chars) {
      if (hid_char->uuid.get_uuid().uuid.uuid16 == ESP_GATT_UUID_HID_REPORT) {
        if (hid_char->properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
          ESP_LOGD(TAG, "HID_REPORT char handle=0x%04X: registering for notify",
                   hid_char->handle);
          auto status = esp_ble_gattc_register_for_notify(
              this->parent()->get_gattc_if(), this->parent()->get_remote_bda(),
              hid_char->handle);
          if (status != ESP_OK) {
            ESP_LOGW(TAG,
                     "Register for notify failed for handle 0x%04X with status=%d",
                     hid_char->handle, status);
          } else {
            this->handles_waiting_for_notify_registration++;
          }
        }
        BLEDescriptor *rpt_ref_desc =
            hid_char->get_descriptor(ESP_GATT_UUID_RPT_REF_DESCR);
        if (rpt_ref_desc != nullptr &&
            this->handles_to_read.count(rpt_ref_desc->handle) > 0 &&
            this->handles_to_read.at(rpt_ref_desc->handle) != nullptr) {
          uint8_t report_id = this->handles_to_read.at(rpt_ref_desc->handle)->value_[0];
          uint8_t report_type = this->handles_to_read.at(rpt_ref_desc->handle)->value_len_ > 1
                                    ? this->handles_to_read.at(rpt_ref_desc->handle)->value_[1]
                                    : 0;
          handle_report_id.insert(std::make_pair(hid_char->handle, report_id));
          ESP_LOGD(TAG, "HID_REPORT char handle=0x%04X: report_id=%d report_type=%d",
                   hid_char->handle, report_id, report_type);
        } else {
          ESP_LOGD(TAG,
                   "HID_REPORT char handle=0x%04X: no RPT_REF_DESCR data, "
                   "report_id will default to 0",
                   hid_char->handle);
        }
      }
    }
  }
  if(generic_access_service != nullptr){
    uint8_t *t_conn_params = this->parse_characteristic_data(generic_access_service, ESP_GATT_UUID_GAP_PREF_CONN_PARAM);
    if(t_conn_params != nullptr){
      this->preferred_conn_params.min_int = t_conn_params[0] | (t_conn_params[1] << 8);
      this->preferred_conn_params.max_int = t_conn_params[2] | (t_conn_params[3] << 8);
      this->preferred_conn_params.latency = t_conn_params[4] | (t_conn_params[5] << 8);
      this->preferred_conn_params.timeout = t_conn_params[6] | (t_conn_params[7] << 8);
      memcpy(this->preferred_conn_params.bda, this->parent()->get_remote_bda(), 6);
      ESP_LOGI(TAG, "Got preferred connection paramters: interval: %.2f - %.2f ms, latency: %u, timeout: %.1f ms", preferred_conn_params.min_int * 1.25f, preferred_conn_params.max_int * 1.25f, preferred_conn_params.latency, preferred_conn_params.timeout*10.f);
    }
  }
  // delete read data:
  for (auto &kv : this->handles_to_read) {
    delete kv.second;
  }
  this->handles_to_read.clear();
}

}  // namespace ble_client_hid
}  // namespace esphome
#endif