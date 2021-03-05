#!/usr/bin/python3
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.button import Button
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
import time

# Variables

state = 'STANDBY'

speed = 20

frequency = 50
calibration_duration = 20
analyze_duration = 10

rmin = 20
rmax = 30

# Setup

manual_override_r = TouchSensor(INPUT_4)
manual_override_l = TouchSensor(INPUT_3)
button = Button()

sound = Sound()

leds = Leds()
leds.set_color('LEFT', 'GREEN')
leds.set_color('RIGHT', 'GREEN')

drum = MediumMotor(OUTPUT_A)
tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)

def left(pressed):
        global state
        if pressed:
                return
        state = 'CALIBRATE'
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
        return []

def calibrate(d, freq, dur, rmin, rmax):
        return rmax

# Main loop

rotations = rmin
while True:
        #print(state)
        if state == 'STANDBY':
                if manual_override_r.is_pressed or manual_override_l.is_pressed:
                        drum.on(SpeedPercent(-speed if manual_override_r.is_pressed else speed))
                elif drum.is_running:
                        drum.stop()
                button.process()
        elif state == 'CALIBRATE':
                print("Calibrating")
                data = collect_data(frequency, calibration_duration)
                rotations = calibrate(data, frequency, calibration_duration, rmin, rmax)
                time.sleep(2)
                state = 'STANDBY'
                leds.set_color('LEFT', 'GREEN')
                leds.set_color('RIGHT', 'GREEN')
        elif state == 'OUTPUT':
                print("Rotating for: " + str(rotations))
                drum.on_for_rotations(SpeedPercent(-speed), rotations)
                time.sleep(1)
                tank_pair.on_for_rotations(left_speed=100, right_speed=100, rotations=1)
                state =  'STANDBY'
                leds.set_color('LEFT', 'GREEN')
                leds.set_color('RIGHT', 'GREEN')
        time.sleep(0.01)
