#!/usr/bin/env python3
import ev3_dc as ev3
import time

# Connect to EV3 brick
my_ev3 = ev3.EV3(protocol=ev3.USB)
my_ev3.verbosity = 0  # set 1 for verbose USB packets

def play_sound(filename, volume=100):
    """Play a sound file from the EV3 brick."""
    # Make sure filename is a string of length <= 20
    filename_bytes = filename.encode('utf-8')
    filename_bytes += b'\x00' * (20 - len(filename_bytes))  # pad to 20 bytes

    ops = b''.join((
        ev3.opSound,
        ev3.PLAY,
        ev3.LCX(volume),
        filename_bytes
    ))
    my_ev3.send_direct_cmd(ops)

# Example: play a sound effect (must be on the brick)
play_sound('sci-fi_puzzle_stage_select_bpm105.mp3', volume=20)

# Wait for the sound to finish (adjust as needed)
time.sleep(3)  # duration in seconds

print("Done!")
my_ev3.disconnect()
