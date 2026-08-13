// The build system generates this on the device; on the host it only has to
// name a target, so the variant-dependent branches pick one. The harness models
// a classic ESP32, so that is the one it claims to be.
#pragma once
#define CONFIG_IDF_TARGET_ESP32 1
