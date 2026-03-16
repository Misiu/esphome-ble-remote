import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_TYPE,
    UNIT_EMPTY,
    DEVICE_CLASS_EMPTY,
    STATE_CLASS_NONE,
)
from esphome.components import ble_client_hid


DEPENDENCIES = ['ble_client_hid']

TYPE_LAST_EVENT_VALUE = "last_event_value"

LastEventValueSensor = sensor.sensor_ns.class_(
    "Sensor"
)

CONFIG_SCHEMA = cv.All(
    cv.typed_schema(
        {
            TYPE_LAST_EVENT_VALUE: sensor.sensor_schema(
                LastEventValueSensor,
                unit_of_measurement=UNIT_EMPTY,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_EMPTY,
                state_class=STATE_CLASS_NONE,
            )
            .extend(ble_client_hid.BLE_CLIENT_HID_SCHEMA),
        },
    ),
)

async def last_event_value_sensor_to_code(config):
    var = await sensor.new_sensor(config)
    await ble_client_hid.register_last_event_value_sensor(var, config)

async def to_code(config):
    await last_event_value_sensor_to_code(config)
