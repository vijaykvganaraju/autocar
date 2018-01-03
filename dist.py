import RPi.GPIO as GPIO

import time

import urllib.request
import json

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

lw = 6
rw = 13
bw1 = 19
bw2 = 26
turningTime = 2
GPIO.setup(lw, GPIO.OUT)
GPIO.setup(rw, GPIO.OUT)
GPIO.setup(bw1, GPIO.OUT)
GPIO.setup(bw2, GPIO.OUT)


GPIO.output(bw2, GPIO.HIGH)

GPIO.output(bw1, GPIO.HIGH)
GPIO.output(lw, GPIO.HIGH)
GPIO.output(rw, GPIO.HIGH)
