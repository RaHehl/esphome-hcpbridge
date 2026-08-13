#!/bin/sh
# The build matrix, on the machine you are sitting at.
#
# Everything else here runs against stubs or a modelled drive, and stubs are
# exactly what let four build errors sit in the tree at once: a constant only
# the newer bus declared, a class renamed out from under the button platform,
# a UART 2 the C3 does not have, and pin defaults that existed twice. None of
# them are visible without a real toolchain, and the leg that would have caught
# them ran only after a push - which is a long way to send a typo.
#
# Slow and large the first time: it fetches ESPHome and the ESP-IDF toolchain.
# Not part of the quick loop; run it before pushing.
set -e
cd "$(dirname "$0")/../.."

. test/esphome-venv.sh

# The same list the workflow builds. The full example is the only one with every
# entity on it, and the entity platforms are where the renamed class hid.
SET="\
.github/example_build_hcpbridge.yaml
.github/variants/esp32.yaml
.github/variants/esp32s2.yaml
.github/variants/esp32c3.yaml
.github/variants/esp32c6.yaml
.github/variants/esp32_hcp1.yaml
.github/variants/esp32p4.yaml"

# The configuration a user copies from. It pulled the component out of GitHub
# until now, so it validated against whatever was published rather than against
# this tree - which is to say it validated nothing here. The key is generated
# and thrown away; shipping a working one in secrets.yaml.example would put a
# real key in everybody's repository.
printf '  %-28s' "example (user-facing)"
if [ -f secrets.yaml ]; then
  echo "SKIPPED, a secrets.yaml of your own is in the way"
else
  sed 's|^api_key:.*|api_key: "'"$(python3 -c 'import base64,os;print(base64.b64encode(os.urandom(32)).decode())')"'"|' \
      secrets.yaml.example > secrets.yaml
  if "$ESPHOME" config example_hcpbridge.yaml >/tmp/hcpbridge-example.log 2>&1; then
    echo "ok"
  else
    echo "FAILED"
    tail -12 /tmp/hcpbridge-example.log
    rm -f secrets.yaml
    exit 1
  fi
  rm -f secrets.yaml
fi

failed=0
for yaml in $SET; do
  name=$(basename "$yaml" .yaml)
  printf '  %-28s' "$name"
  if "$ESPHOME" compile "$yaml" >"/tmp/hcpbridge-build-$name.log" 2>&1; then
    echo "ok"
  else
    echo "FAILED"
    grep -E "error:" "/tmp/hcpbridge-build-$name.log" | sed 's/.*error: /      /' | sort -u | head -6
    failed=$((failed + 1))
  fi
done

[ "$failed" = "0" ] || { echo "$failed of the builds did not link"; exit 1; }
echo "all builds link"
