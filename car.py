import RPi.GPIO as GPIO

import time

import urllib.request
import json

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

lw = 5
rw = 6
bw1 = 13
bw2 = 26

GPIO.setup(lw, GPIO.OUT)
GPIO.setup(rw, GPIO.OUT)
GPIO.setup(bw1, GPIO.OUT)
GPIO.setup(bw2, GPIO.OUT)

def getDirections():
    url = 'https://api.thingspeak.com/channels/330275/feeds.json?api_key=6KPE66DUXE2QJ3I0&results=2'
    req = urllib.request.Request(url)
    #parsing response
    r = urllib.request.urlopen(req).read()
    json_data = json.loads(r.decode('utf-8'))
    return json_data['feeds'][-1]['field1'] #Returning Direction as string

distance = 0
while (True):
    directions = getDirections()
    print(directions)
    for i in range(len(directions)):
        if directions[i] == 'f' or directions[i] == 'F':
            j = 0
            while directions[i + 1].isdigit():
                distance = distance * j + int(directions[i + 1])
                i, j = i + 1, j + 1
            distance *= 10 + int(directions[i])
            break
        print(distance)
    time.sleep(10000)
                