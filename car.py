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
turningTime = 10
velocity = 0.3 # meter(s) per second

key = '' #Removed for security reasons
trigL = 2
echoL = 17
trigR = 3
echoR = 27
trigF = 4
echoF = 22

GPIO.setup(lw, GPIO.OUT)
GPIO.setup(rw, GPIO.OUT)
GPIO.setup(bw1, GPIO.OUT)
GPIO.setup(bw2, GPIO.OUT)

GPIO.setup(trigL, GPIO.OUT)
GPIO.setup(trigR, GPIO.OUT)
GPIO.setup(trigF, GPIO.OUT)
GPIO.setup(echoL, GPIO.IN)
GPIO.setup(echoR, GPIO.IN)
GPIO.setup(echoF, GPIO.IN)

def getDirections():		#To get directions
    url = 'https://api.thingspeak.com/channels/330275/feeds.json?api_key=' + key + '&results=2'
    req = urllib.request.Request(url)
    #parsing response
    r = urllib.request.urlopen(req).read()
    json_data = json.loads(r.decode('utf-8'))
    return json_data['feeds'][-1]['field1'] #Returning Direction as string
	
def forward(distance):
    t = time.time();
    #while (distance / velocity) * 1000 <= time.time() - t:
	    #obstructionCheck()
    GPIO.output(bw2, GPIO.LOW)
    GPIO.output(bw1, GPIO.HIGH)
    time.sleep(5)
    GPIO.output(bw1, GPIO.LOW)
    print("Forward")

def lTurn():
    GPIO.output(rw, GPIO.LOW)
    GPIO.output(lw, GPIO.HIGH)
    time.sleep(turningTime)
    GPIO.output(lw, GPIO.LOW)
    print("Turned Left")

def rTurn():
    GPIO.output(lw, GPIO.LOW)
    GPIO.output(rw, GPIO.HIGH)
    time.sleep(turningTime)
    GPIO.output(rw, GPIO.LOW)
    print("Turned Right")

def ultrasonicData(trig, echo):
    GPIO.output(trig, False)                 #Set TRIG as LOW
    print("Waitng For Sensor To Settle")
    time.sleep(1)                            #Delay of 1 second

    GPIO.output(trig, True)                  #Set TRIG as HIGH
    time.sleep(0.00001)                      #Delay of 0.00001 seconds
    GPIO.output(trig, False)                 #Set TRIG as LOW

    while GPIO.input(echo)==0:               #Check whether the ECHO is LOW
        pulse_start = time.time()              #Saves the last known time of LOW pulse

    while GPIO.input(echo)==1:               #Check whether the ECHO is HIGH
        pulse_end = time.time()                #Saves the last known time of HIGH pulse 

    pulse_duration = pulse_end - pulse_start #Get pulse duration to a variable

    distance = pulse_duration * 17150        #Multiply pulse duration by 17150 to get distance
    distance = int(distance)                    #Round to two decimal points

    return distance
    
distance = 0
while (True):
    directions = getDirections()
    print(directions)
    i = 0
    while i < len(directions):
        if directions[i] == 'f' or directions[i] == 'F':
            distance = 0
            while i < len(directions) - 1 and directions[i + 1].isdigit():
                distance = distance * 10 + int(directions[i + 1])
                i += 1
            print(distance)
            time.sleep(2)
            forward(distance)
		
        elif directions[i] == 'r' or directions[i] == 'R':
            rTurn()
            
			
        elif directions[i] == 'l' or directions[i] == 'L':
            lTurn()
			
        i += 1
    print("End")
    time.sleep(10)
                
