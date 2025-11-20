#!/usr/bin/env python3
import ev3_dc as ev3
import time

# Connect to EV3 brick
my_ev3 = ev3.EV3(protocol=ev3.USB)
my_ev3.verbosity = 0  # set 1 for verbose USB packets

def play_tone(freq_hz, duration_ms, volume=100):
    """Play one tone on the EV3 speaker via direct command."""
    ops = b''.join((
        ev3.opSound,
        ev3.TONE,
        ev3.LCX(volume),
        ev3.LCX(freq_hz),
        ev3.LCX(duration_ms),
    ))
    my_ev3.send_direct_cmd(ops)
    time.sleep(duration_ms / 1000.0 * 1.05)  # small gap

# === Imperial March (simplified) ===
melody = [
    (440, 500), (440, 500), (440, 500),
    (349, 350), (523, 150),
    (440, 500), (349, 350), (523, 150),
    (440, 1000),
    (659, 500), (659, 500), (659, 500),
    (698, 350), (523, 150),
    (415, 500), (349, 350), (523, 150),
    (440, 1000),
]

print("Playing Imperial March...")
for freq, dur in melody:
    play_tone(freq, dur, volume=20)

print("Done!")
my_ev3.disconnect()
