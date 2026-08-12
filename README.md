# esphome-hcpbridge

[![GitHub](https://img.shields.io/github/license/14yannick/esphome-hcpbridge)](https://github.com/14yannick/esphome-hcpbridge/blob/main/LICENSE)
[![GitHub issues](https://img.shields.io/github/issues/14yannick/esphome-hcpbridge)](https://github.com/14yannick/esphome-hcpbridge)
[![GitHub Sponsors](https://img.shields.io/github/sponsors/mapero)](https://github.com/sponsors/mapero)
![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/14yannick/esphome-hcpbridge/build.yaml)


This is a esphome-based adaption of the HCPBridge. thanks to [mapero](https://github.com/14yannick/esphome-hcpbridge) for the initial esphome port. Credits for the initial development of the HCPBridge go to [Gifford47](https://github.com/Gifford47/HCPBridgeMqtt), [hkiam](https://github.com/hkiam/HCPBridge) and all the other guys contributed.

## Usage

### Example esphome configuration

### Breaking change for existing configurations

The component brings its own Modbus RTU responder and talks to the drive
through the ESP-IDF UART driver, so an existing configuration needs three
edits:

- `framework: type: arduino` becomes `framework: type: esp-idf`
- the `esphome: libraries:` entry for `emelianov/modbus-esp8266` goes away
- any `platformio_options` that were only there for that library go with it

Supported targets are the classic ESP32 and the ESP32-S3, the two variants
that have a third UART. The configuration is rejected with a readable message
elsewhere rather than failing inside the compiler.

```YAML
substitutions:
  name: "hcpbridge"
  friendly_name: "Garage Door"
esphome:
  name: "${name}"
  friendly_name: "${friendly_name}"

external_components:
    source: github://14yannick/esphome-hcpbridge
    refresh: 0s # Ensure you always get the latest version

esp32:
  board: adafruit_feather_esp32s3 # set your board
  cpu_frequency: 240MHZ
  framework:
    type: esp-idf

hcpbridge:
  id: hcpbridge_id
  rx_pin: 18 # optional; default 18 on ESP32-S3, 16 on plain ESP32
  tx_pin: 17 # optional, default 17
  #rts_pin: 1 # optional; drives an RS485 transceiver that has no automatic
  #           # direction control. Untested on hardware.
  #uart_num: 2            # optional, default 2
  #update_interval: 500ms # optional; how often entities are refreshed

cover:
  - platform: hcpbridge
    name: ${friendly_name}
    device_class: garage
    id: garagedoor_cover
```

### Home Assistant

![Home Assistant Device Overview](docs/device_overview.png)

### Cover

The component provides a cover component to control the garage door.

### Light

The component provides a Light component to turn the light off and on.
The Output is needed to control the light.
```YAML
output:
  - platform: hcpbridge
    id: output_light

light:
  - platform: hcpbridge
    id: gd_light
    output: output_light
    name: Garage Door Light
```
### Telling the drive about a restart

The bridge is a bus accessory, and a drive notices one that stops answering. On
an ordinary restart it says so itself and waits for the drive to confirm before
going away.

An update is different: writing the new image stops the serial driver from
running, so the bus is unanswered for the whole transfer, long before any
shutdown handler runs. To be told in time the drive has to hear it when the
transfer starts, which is a trigger on the `ota:` platform:

```YAML
ota:
  - platform: esphome
    on_begin:
      then:
        - lambda: 'id(hcpbridge_id).announce_pause();'
```

`id(...)` is the id of the `hcpbridge:` block. Without this the update still
works; the drive simply sees the accessory vanish and may drop it from its list,
which then needs a bus scan at the drive to undo.

While at it: `logger:` writes to the serial console by default, and this
component logs from its own task. On a busy or noisy bus that can hold up an
answer to the drive. `baud_rate: 0` turns the console off and keeps the network
log, which is what you want on a device that is not on a desk.

### Binary_Sensor

The component provides you three sensor.

- `is_connected`: Who indicated if there is a valid connection with the door.
  It goes off again after twenty seconds without a frame from the drive, so a
  bus that has gone quiet no longer looks like a working one.
- `relay_state`: Give the status of the option relay (Menu 30) of the HCP.
- `actuator_error`: The drive's own fault indication, taken from the state it
  broadcasts anyway. Nothing extra is sent to read it. Worth having on: it was
  seen standing at fault for an hour while the door still worked normally, and
  the drive then dropped the accessory off the bus and showed a communication
  error on its own display. It cleared when the drive was power cycled. One
  observation, so treat it as an early warning to watch rather than a verdict.
```YAML
binary_sensor:
  - platform: hcpbridge
    is_connected:
      name: "HCPBridge Connected"
      id: sensor_connected
    relay_state:
      name: "Garage Door Relay state"
      id: sensor_relay
      #on_state:
      #create your automation based on Garage Door Relay state
    actuator_error:
      name: "Garage Door Fault"
      id: sensor_actuator_error
```
### Text_sensor

This component provide you a detailed current state of the door. This text can be changed using the substitute functionality.
```YAML
text_sensor:
  - platform: hcpbridge
    id: sensor_templ_state
    name: "Garage Door State"
```

With `type` the same platform reports what the drive says about itself instead
of what the door is doing. Both values are asked for once after the drive has
started talking to us, so they cost one exchange per boot and nothing after
that. They stay empty until the drive has answered. Without `type` the sensor
reports the door state as before.

- `serial_number`: the drive's serial number.
- `firmware_version`: the drive's firmware version.
```YAML
text_sensor:
  - platform: hcpbridge
    type: serial_number
    id: sensor_drive_serial
    name: "Garage Door Serial Number"
    entity_category: diagnostic
  - platform: hcpbridge
    type: firmware_version
    id: sensor_drive_firmware
    name: "Garage Door Firmware Version"
    entity_category: diagnostic
```
### sensor

This component provide you the position of the door in %. Where 100% is fully open.
```YAML
sensor:
  - platform: hcpbridge
    id: sensor_position
    name: ${sen_pos}
```

With `type: target_position` the same platform reports where the door is
heading instead of where it is. The drive sends both in one register, so this
needs no extra traffic. Without `type` the sensor reports the current position
as before.
```YAML
sensor:
  - platform: hcpbridge
    type: target_position
    id: sensor_target_position
    name: "Garage Door Target Position"
```
### Button

This component allows you to add three buttons to sond commands to the door.
```YAML
button:
  - platform: hcpbridge
    vent_button:
      id: button_vent
      name: "Garage Door Vent"
    impulse_button:
      id: button_impulse
      name: "Garage Door Impulse"
    half_button:
      id: button_half
      name: "Half"
```

### Switch

This component allows you to add two switch to sond commands to the door.
```YAML
switch:
  - platform: hcpbridge
    vent_switch:
      id: switch_vent
      name: "Venting"
      restore_mode: disabled
    half_switch:
      id: half_switch
      name: "Open Half"
      restore_mode: disabled
```

### Services

Additionally, when using the cover component, you can expose the following services to the API:

- `esphome.hcpbridge_go_to_close`: To close the garage door
- `esphome.hcpbridge_go_to_half`: To move the garage door to half position
- `esphome.hcpbridge_go_to_vent`: To move the garage door to the vent position
- `esphome.hcpbridge_go_to_open`: To open the garage door
- `esphome.hcpbridge_toggle`: Send an Impulse command to the door

There are in the YAML and not directly in the Cover to remove the API dependency there. This give the possibility to use the Cover without the API Component for exemple only with the web_server or mqtt.
```YAML
api:
  encryption:
    key: !secret api_key
  services:
    - service: go_to_open
      then:
        - lambda: |-
            id(garagedoor_cover).on_go_to_open();
    - service: go_to_close
      then:
        - lambda: |-
            id(garagedoor_cover).on_go_to_close();
    - service: go_to_half
      then:
        - lambda: |-
            id(garagedoor_cover).on_go_to_half();
    - service: go_to_vent
      then:
        - lambda: |-
            id(garagedoor_cover).on_go_to_vent();
    - service: toggle
      then:
        - cover.toggle: garagedoor_cover
```

### Example YAML

Check out the [example_hcpbridge.yaml](./example_hcpbridge.yaml) for a complete yaml with all hcpbridge components.

# Project

- HCPBridge from `Tysonpower` on an `Hörmann Promatic 4`

You can find more information on the project here: [Hörmann garage door via MQTT](https://community.home-assistant.io/t/hormann-garage-door-via-mqtt/279938/340)
Known working hardware are the ESP32 and S3 dual core chip.

# ToDo

- [x] Initial working version
- [x] Map additional functions to esphome
- [x] Use callbacks instead of pollingComponent (Only hcpbridge is polling)
- [x] Expert options for the HCPBridge component (GPIOs ...)

# Contribute

I am open for contribution. Just get in contact with me.

# License

```
MIT License

Copyright (c) 2023 Jochen Scheib

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
