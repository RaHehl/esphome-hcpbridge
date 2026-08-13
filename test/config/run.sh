#!/bin/sh
# Every configuration this component refuses, checked that it actually refuses.
#
# The build matrix shows that valid configurations pass. Nothing showed that
# the invalid ones fail, and that gap is not hypothetical: the check that
# refuses GPIO16/17 on a WROVER with psram: enabled was dead for a while - it
# read pins out of the configuration while the defaults still lived in C++, so
# in the default case, which is exactly the case that hits the trap, it looked
# at nothing.
#
# Each file under rejected/ names the fragment it expects in the refusal. A
# case that fails for some other reason counts as a failure: it would otherwise
# pass while testing nothing.
set -e
cd "$(dirname "$0")/../.."
. test/esphome-venv.sh

failed=0
for yaml in test/config/rejected/*.yaml; do
  name=$(basename "$yaml" .yaml)
  expect=$(sed -n 's/^# expect: //p' "$yaml")
  [ -n "$expect" ] || { echo "  $name: no '# expect:' line"; failed=$((failed + 1)); continue; }
  printf '  %-44s' "$name"
  if out=$("$ESPHOME" config "$yaml" 2>&1); then
    echo "ACCEPTED"
    echo "      it should have been refused with: $expect"
    failed=$((failed + 1))
  elif echo "$out" | grep -qF "$expect"; then
    echo "refused"
  else
    echo "WRONG REASON"
    echo "      wanted: $expect"
    echo "$out" | grep -iE "error|invalid" | head -3 | sed 's/^/      got:    /'
    failed=$((failed + 1))
  fi
done

[ "$failed" = "0" ] || { echo "$failed of the refusals did not hold"; exit 1; }
echo "every configuration that should be refused is refused"
