# esphome-hcpbridge

[![GitHub](https://img.shields.io/github/license/14yannick/esphome-hcpbridge)](https://github.com/14yannick/esphome-hcpbridge/blob/main/LICENSE)
[![GitHub issues](https://img.shields.io/github/issues/14yannick/esphome-hcpbridge)](https://github.com/14yannick/esphome-hcpbridge)
[![GitHub Sponsors](https://img.shields.io/github/sponsors/mapero)](https://github.com/sponsors/mapero)
![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/14yannick/esphome-hcpbridge/build.yaml)


This is a esphome-based adaption of the HCPBridge. thanks to [mapero](https://github.com/mapero/esphome-hcpbridge) for the initial esphome port. Credits for the initial development of the HCPBridge go to [Gifford47](https://github.com/Gifford47/HCPBridgeMqtt), [hkiam](https://github.com/hkiam/HCPBridge) and all the other guys contributed.

The single-core support and the observation that a low-power UART must not be
handed this bus come from [hoellen](https://github.com/14yannick/esphome-hcpbridge/pull/20).

## Usage

### Example esphome configuration

### Breaking change for existing configurations

The component brings its own Modbus RTU responder and talks to the drive
through the ESP-IDF UART driver, so an existing configuration needs three
edits:

- `framework: type: arduino` becomes `framework: type: esp-idf`
- the `esphome: libraries:` entry for `emelianov/modbus-esp8266` goes away
- any `platformio_options` that were only there for that library go with it
- `binary_sensor`, `button` and `switch` entities each get their own
  `- platform: hcpbridge` line with a `type:`, the way `sensor` and
  `text_sensor` already did

The last one is the same edit twice avoided. Two shapes existed: some domains
nested their entities under one platform line, the rest gave each entity its
own. Since the framework change already means editing every configuration
once, both now look the same:

```YAML
# before
binary_sensor:
  - platform: hcpbridge
    is_connected:
      name: "Connected"
    relay_state:
      name: "Relay"

# after
binary_sensor:
  - platform: hcpbridge
    type: is_connected
    name: "Connected"
  - platform: hcpbridge
    type: relay_state
    name: "Relay"
```

The types are `is_connected`, `relay_state` and `actuator_error` for
`binary_sensor`; `impulse`, `vent` and `half` for `button`; `vent` and `half`
for `switch`. Device classes and icons follow the type, so they no longer have
to be repeated in the configuration.

### Why the link is down, not just that it is

`is_connected` is a yes or no, and the two ways it can be no want opposite
things done about them: a bus nothing has ever arrived on wants its wiring
checked or the drive told to scan for accessories, while one that fell quiet
wants the drive power cycled. Guessing which costs an evening.

```YAML
text_sensor:
  - platform: hcpbridge
    type: link_state
    name: "Link"
    entity_category: diagnostic
```

| | |
| --- | --- |
| `Never seen` | not one frame, ever. Wiring, or the drive has not been told to look |
| `Enumerating` | the drive is talking, but has not accepted an answer yet |
| `Registered` | answering polls, which is the ordinary state |
| `Silent` | was registered, then nothing for twenty seconds |
| `Paused` | told the drive we are going quiet, and it agreed |

### Older drives

A SupraMatic E3 speaks a different bus. Say so, and the image is built for it:

```YAML
hcpbridge:
  protocol: hcp1   # default is hcp2, what a Series 4 and up speaks
  rx_pin: 16
  tx_pin: 17
```

It is a build-time choice, not a switch: the drive's generation is fixed when
the door is installed, so the other protocol has no reason to be on the device.

That bus carries less. There is no serial number, no firmware version, no
position beyond open and shut, and no way to ask for the half-open position -
the drive can do it, but neither description of the protocol says how to ask,
and the alternative to refusing is guessing with a door. Entities for any of
those are refused during validation rather than offered and left reading
nothing.

Also missing is any acknowledgement that an answer arrived. On the newer bus a
lost answer is visible and the command is sent again; here the two are
indistinguishable, so nothing is ever repeated - a second send would be a
second press.

**This has not been tried on a drive.** Both descriptions of the protocol agree,
the published example frames are reproduced byte for byte, and the exchange is
checked against a simulated drive - but nobody here owns an E3.

### Two doors on one board

A classic ESP32 has three usable UARTs and the console takes one, which leaves
room for a second drive. Give each its own block:

```YAML
hcpbridge:
  - id: left_door
    uart_num: 1
    rx_pin: 16
    tx_pin: 17
  - id: right_door
    uart_num: 2
    rx_pin: 25
    tx_pin: 26

cover:
  - platform: hcpbridge
    hcpbridge_id: left_door
    name: Left
    device_class: garage
```

A single block without a list stays valid and needs no `hcpbridge_id` on its
entities. Two buses sharing a port or a pin are refused rather than left to
read half of each other's frames.

### Supported chips

Any ESP32 with a spare hardware UART, which is every variant ESPHome supports.
What differs is how much has to be said in the configuration:

`uart_num` defaults to 2 where the chip has more than two full UARTs and to 1
where it has exactly two, so every configuration written before this stays on
the port it already used. The pins default only on the ESP32 and the ESP32-S3;
everywhere else `rx_pin` and `tx_pin` have to be named.

Pins are left implicit only where a real door has confirmed them. Elsewhere a
guessed pair would validate, build, and then quietly never hear the drive.

Low-power UARTs are not counted. Several variants report one alongside their
full ports, and its 16-byte buffer is a fraction of what a frame here can need,
so `uart_num` stops at the last full one. `uart_num: 0` is refused outright:
the ROM loader prints on that port at every boot and the drive would read it as
a broken frame.

If the logger or a `uart:` component already holds the port this component
would take, the configuration says so instead of the two fighting over it at
runtime. The same check refuses GPIO16/17 on a WROVER with `psram:` enabled,
where those two pins carry the memory bus.

Only the ESP32 and the ESP32-S3 have been run against an actual door. The
ESP32-S2, the C3, the C6 and the P4 are built on every change, which shows the
sources compile and link there and nothing more than that. The remaining
variants in the table above are accepted by validation and have never been
built, so treat them as untried rather than supported.

### Knowing what is on the device

The example sets `esphome:` `project:` so the running build shows up in Home
Assistant under the device. It is worth bumping the `build:` substitution on
every flash: a drive that starts refusing an accessory is hard enough to
diagnose without also having to guess which firmware is answering it.

What reaches a door has been through everything under "Checking a change"
below: a modelled drive played against it, a gate that says whether those
checks would have gone red at all, and a build for every chip. None of that
can say how a particular drive will take it. The drive counts unanswered
polls, so a device that is flashed and then left unreachable is worse than one
that was never flashed: keep the door in sight while it comes back up.

```YAML
substitutions:
  name: "hcpbridge"
  friendly_name: "Garage Door"
  build: "unversioned" # bump on every flash, see above
esphome:
  name: "${name}"
  friendly_name: "${friendly_name}"
  project:
    name: "hcpbridge.esphome"
    version: "${build}"

external_components:
  - source:
      type: local
      path: "./components/"

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

# Checking a change

Five things run without a drive, in rising order of what they cost:

```sh
sh test/unit/run.sh          # pieces that stand on their own, a few seconds
sh test/sim/run.sh           # a modelled drive played against the accessory
sh test/config/run.sh        # every configuration that should be refused, is
sh test/build/run.sh         # the real build matrix, every chip and both buses
sh test/mutation/run.sh      # breaks the codec on purpose, reports who noticed
```

The first two run against stub headers, which is why the build matrix exists:
only a real toolchain can say that what compiles on a desk also links for a C3,
or for the older bus that no drive in reach speaks. It fetches ESPHome and the
ESP-IDF toolchain on first use, so it is slow once and quick afterwards.

The last one asks what the others cannot ask about themselves. It breaks the
codec on purpose, one defect at a time, and reports which gates went red. A
suite that stays green on broken code is decoration, and the first run of this
found exactly that: every test here watched what went out on the wire and none
watched what came back, so halving the reported position passed all of them.

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
