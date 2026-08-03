#!/usr/bin/env bash
#
# Reports which quadrature encoder channels are physically alive.
#
# The counting happens on the MCU, in the 5 kHz control loop, not here. Sampling
# GPIO over SWD from the host tops out around thirty reads a second, while a
# hand-turned encoder emits hundreds of edges a second, so a perfectly healthy
# channel can read as dead. The firmware keeps per-pin edge counters instead and
# this just zeroes them, waits while you move the arm, and reads them back.
#
# Independent of whether a pin's EXTI is configured, or configured for the right
# port, so it can distinguish "not wired" from "wired but the decoder ignores it".
#
# usage: tools/encoder_probe.sh [seconds]

set -u
SECONDS_TO_RUN="${1:-20}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ELF="$SCRIPT_DIR/../build/firmware.elf"

if [[ ! -f "$ELF" ]]; then
  echo "cannot find $ELF - build the firmware first" >&2
  exit 1
fi

sym() { arm-none-eabi-nm "$ELF" | grep -iE " $1\$" | awk '{print $1}'; }

EDGES=$(sym encoder_pin_edges)
SAMPLES=$(sym encoder_debug_samples)
if [[ -z "$EDGES" || -z "$SAMPLES" ]]; then
  echo "firmware has no encoder debug counters; is it the current build?" >&2
  exit 1
fi

CMD_FILE="$(mktemp)"
trap 'rm -f "$CMD_FILE"' EXIT

# Zero the counters, wait while the operator moves the arm, then read them back.
{
  printf 'w4 0x%s' "$SAMPLES"; printf ' 0\n'
  for i in $(seq 0 79); do
    printf 'w4 0x%x 0\n' $(( 0x$EDGES + i * 4 ))
  done
  printf 'sleep %d\n' $(( SECONDS_TO_RUN * 1000 ))
  # J-Link reads this count as hex: 0x50 is the eighty pins of GPIOA..GPIOE.
  printf 'mem32 0x%s,50\n' "$EDGES"
  printf 'mem32 0x%s,1\n' "$SAMPLES"
  printf 'qc\n'
} > "$CMD_FILE"

echo "Counters zeroed. Move the joints you care about for the next ${SECONDS_TO_RUN}s."
echo

JLinkExe -nogui 1 -device STM32F407VE -if SWD -speed 4000 -autoconnect 1 \
  -CommanderScript "$CMD_FILE" 2>&1 \
  | grep -E '^[0-9A-F]{8} = ' \
  | python3 "$SCRIPT_DIR/encoder_probe.py" "$EDGES" "$SAMPLES"
