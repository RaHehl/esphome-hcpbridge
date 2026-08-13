# Sourced, not run. Puts an ESPHome matching the workflow on $ESPHOME, in a
# venv shared by every gate that needs one, and leaves the caller in the
# repository root.
#
# The version is read from the workflow rather than repeated here, so the desk
# and CI cannot drift apart on which ESPHome this component claims to build
# against.
VERSION=$(sed -n 's/^ *ESPHOME_VERSION: *"\(.*\)"/\1/p' .github/workflows/build.yaml)
[ -n "$VERSION" ] || { echo "no ESPHOME_VERSION in .github/workflows/build.yaml"; exit 1; }

VENV=${HCPBRIDGE_BUILD_VENV:-.esphome-venv}
if [ ! -x "$VENV/bin/esphome" ] \
   || [ "$("$VENV/bin/esphome" version 2>/dev/null | tr -dc 0-9.)" != "$(echo "$VERSION" | tr -dc 0-9.)" ]; then
  echo "setting up ESPHome $VERSION in $VENV"
  rm -rf "$VENV"
  python3 -m venv "$VENV"
  "$VENV/bin/pip" -q install "esphome==$VERSION"
fi
ESPHOME="$VENV/bin/esphome"
