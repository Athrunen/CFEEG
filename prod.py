#!/usr/bin/python3
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.button import Button
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
from ev3dev2.console import Console

import time

import numpy as np
import serial
from numpy.fft import rfft, rfftfreq

# Variables

state = 'STANDBY'

speed = 20

frequency = 50
calibration_duration = 10

rmin = 10
rmax = 30

# Setup

stream = serial.Serial()
stream.baudrate = 9600
stream.port = "/dev/ttyACM0"
stream.timeout = 0.01
stream.open()

manual_override_r = TouchSensor(INPUT_4)
manual_override_l = TouchSensor(INPUT_3)
button = Button()

sound = Sound()

console = Console()
console.reset_console()

leds = Leds()
leds.set_color('LEFT', 'GREEN')
leds.set_color('RIGHT', 'GREEN')

drum = MediumMotor(OUTPUT_A)
tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)

def left(pressed):
    global state
    if pressed:
        return
    state = 'MEASURE'
    leds.set_color('LEFT', 'RED')
    leds.set_color('RIGHT', 'RED')

def right(pressed):
    global state
    if pressed:
        return
    state = 'OUTPUT'
    leds.set_color('LEFT', 'AMBER')
    leds.set_color('RIGHT', 'AMBER')

button.on_left = left
button.on_right = right

sound.beep()

# Functions

def collect_data(freq, dur):
    d = np.zeros(freq * dur)
    n = 0
    while n < (freq * dur):
        stream.reset_input_buffer()
        time.sleep(1 / freq)
        try:
            s = stream.readline().decode()
            if not s.strip():
                continue
            s = float(s)
            s = s / 1024.
            if (s > 0.15 and s < 0.95):
                d = np.roll(d, -1)
                d[-1] = s
                n += 1
                leds.set_color('LEFT', 'RED')
                leds.set_color('RIGHT', 'RED')
                continue
            leds.set_color('LEFT', 'AMBER')
            leds.set_color('RIGHT', 'AMBER')
        except Exception as e:
            continue
    return d

def calibrate(d, freq, dur):
    yf = rfft(d)
    np_a = np.abs(yf)
    xf = rfftfreq(freq * dur, 1 / freq)
    alpha = np_a[7*dur:13*dur]
    a_s = sum(alpha) / (6 * dur)
    beta = np_a[13*dur:25*dur]
    b_s = sum(beta) / (12 * dur)

    s = a_s + b_s
    
    if s != 0.0:
    
        a_s = a_s / s
        b_s = b_s / s

    variance = np.ptp(np_a[4:25])
    return(a_s, b_s, variance)

def map_to_rotations(rmin, rmax, cal=None):
    dist = rmax - rmax

    if (cal):
        half = (rmin + rmax) / 2
        dist = half - rmin
        return (half + dist * ((cal[0] * 10 - cal[1] * 10)))
    return rmin

# Main loop

cal = None
while True:
    if state == 'STANDBY':
        if manual_override_r.is_pressed or manual_override_l.is_pressed:
            drum.on(SpeedPercent(-speed if manual_override_r.is_pressed else speed))
        elif drum.is_running:
            drum.stop()
        button.process()
    elif state == 'MEASURE':
        print("Measuring")
        data = collect_data(frequency, calibration_duration)
        print("Collected!")
        cal = calibrate(data, frequency, calibration_duration)
        print(cal)
        console.text_at(cal, column=1, row=5, reset_console=True, inverse=True)
        sound.beep()
        state = 'STANDBY'
        leds.set_color('LEFT', 'GREEN')
        leds.set_color('RIGHT', 'GREEN')
    elif state == 'OUTPUT':
        rotations = map_to_rotations(rmin, rmax, cal)
        print("Rotating for: " + str(rotations))
        console.text_at(rotations, column=1, row=5, reset_console=True, inverse=True)
        drum.on_for_rotations(SpeedPercent(-speed), rotations)
        time.sleep(1)
        tank_pair.on_for_rotations(left_speed=speed, right_speed=speed, rotations=1)
        state =  'STANDBY'
        leds.set_color('LEFT', 'GREEN')
        leds.set_color('RIGHT', 'GREEN')
    time.sleep(0.01)
