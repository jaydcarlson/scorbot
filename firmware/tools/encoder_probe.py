#!/usr/bin/env python3
"""Reports which GPIO pins saw edges, from counters dumped by encoder_probe.sh.

Every pin of GPIOA..GPIOE is reported, not just the ones the firmware believes
are encoders, so a signal bodged onto an unexpected pin shows up as an unclaimed
pin with activity rather than silently as a dead encoder.

Parses J-Link "ADDRESS = W0 W1 W2 W3" lines into an address-keyed map and looks
counters up by real address. Slicing positionally is a trap: J-Link reads the
count argument of `mem32 addr,N` as hex, so asking for 12 words returns 18.

usage: encoder_probe.py <edges-address-hex> <samples-address-hex>
"""

import sys

CONTROL_LOOP_HZ = 5000.0
PORTS = "ABCDE"

# What the current firmware believes each pin is, from Inc/main.h.
ENCODER_PINS = {
    "PA10": "M1 shoulder_pan A",
    "PB5": "M1 shoulder_pan B",
    "PA15": "M2 shoulder_lift A",
    "PC9": "M2 shoulder_lift B",
    "PD12": "M3 elbow A",
    "PC6": "M3 elbow B",
    "PB4": "M4 wrist A",
    "PA9": "M4 wrist B",
    "PD13": "M5 wrist A",
    "PC7": "M5 wrist B",
    "PC8": "M6 gripper A",
    "PB3": "M6 gripper B",
}

# Limit switches. Not encoders, but hugely useful here: a switch toggling is
# independent proof that the joint really moved, which is what turns "no encoder
# edges" from ambiguous into conclusive.
LIMIT_PINS = {
    "PD11": "MS1 shoulder_pan",
    "PD10": "MS2 shoulder_lift",
    "PE15": "MS3 elbow",
    "PB10": "MS4 wrist_roll",
    "PD14": "MS5 wrist_pitch",
    "PD15": "MS6 gripper",
    "PC15": "MS7 slide",
    "PE12": "MS8 spare",
}

# Expected to toggle for reasons that have nothing to do with joint motion.
# The RMII lines in particular are noisy enough to bury everything else: the
# reference clock alone runs at 50 MHz and aliases into tens of thousands of
# apparent edges at the 5 kHz sample rate.
IGNORED = {
    "PA13": "SWDIO (debugger)",
    "PA14": "SWCLK (debugger)",
    "PD5": "DEBUG_TX (printf)",
    "PA1": "ETH RMII_REF_CLK (50 MHz)",
    "PA2": "ETH MDIO",
    "PA7": "ETH RMII_CRS_DV",
    "PC1": "ETH MDC",
    "PC4": "ETH RMII_RXD0",
    "PC5": "ETH RMII_RXD1",
    "PB11": "ETH RMII_TX_EN",
    "PB12": "ETH RMII_TXD0",
    "PB13": "ETH RMII_TXD1",
    "PA8": "TIM1_CH1 (slide hardware encoder)",
}


def parse_words(stream):
    words = {}
    for line in stream:
        if "=" not in line:
            continue
        head, _, tail = line.partition("=")
        try:
            base = int(head.strip(), 16)
        except ValueError:
            continue
        for index, token in enumerate(tail.split()):
            try:
                words[base + 4 * index] = int(token, 16)
            except ValueError:
                break
    return words


def main():
    if len(sys.argv) != 3:
        print("usage: encoder_probe.py <edges-address-hex> <samples-address-hex>")
        return 2

    edges_addr = int(sys.argv[1], 16)
    samples_addr = int(sys.argv[2], 16)
    words = parse_words(sys.stdin)

    if samples_addr not in words:
        print("did not get the counters back from the target")
        return 1

    samples = words[samples_addr]
    print(f"control-loop samples: {samples} "
          f"({samples / CONTROL_LOOP_HZ:.1f}s at {CONTROL_LOOP_HZ:.0f} Hz)")
    if samples == 0:
        print("\nThe sampling loop never ran. Is the control-loop timer running?")
        return 1

    active = []
    for index in range(len(PORTS) * 16):
        count = words.get(edges_addr + 4 * index, 0)
        if count == 0:
            continue
        name = f"P{PORTS[index // 16]}{index % 16}"
        active.append((count, name))
    active.sort(reverse=True)

    print("\n--- encoder channels the firmware expects ---")
    print(f"{'pin':<7}{'role':<22}{'edges':>9}")
    counts = {name: c for c, name in active}
    for pin, role in ENCODER_PINS.items():
        print(f"{pin:<7}{role:<22}{counts.get(pin, 0):>9}")

    switches = [(c, n) for c, n in active if n in LIMIT_PINS]
    print("\n--- limit switches that toggled (proof the joint moved) ---")
    if switches:
        for count, name in switches:
            print(f"{name:<7}{LIMIT_PINS[name]:<22}{count:>9}")
    else:
        print("none")

    unclaimed = [(c, n) for c, n in active
                 if n not in ENCODER_PINS and n not in LIMIT_PINS and n not in IGNORED]
    print("\n--- unexplained pin activity ---")
    if unclaimed:
        print(f"{'pin':<7}{'edges':>9}")
        for count, name in unclaimed:
            print(f"{name:<7}{count:>9}")
        print("\nActivity on a pin that is not a declared encoder channel is where a\n"
              "bodged signal actually landed. That is the pin to remap to.")
    else:
        print("none - nothing is toggling on a pin we were not already accounting for,\n"
              "so a silent encoder is not merely wired somewhere unexpected.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
