#!/usr/bin/env python3

import serial
import sys
import json
import time
import re
import subprocess

try:
    ser = serial.Serial(port='/dev/arduino', baudrate=9600, write_timeout=3)
except:
    sys.exit(0)

time.sleep(3)
t0 = []
t1 = []
ser.write(' '.encode('ascii'))
response = ""
while True:
    line = ser.readline().decode('ascii')
    response = response + line
    line = line.strip()
#    print(line)
    if line == "}":
        break
    
ser.close()
#print(response)

temp2 = 0
temp3 = 0

try:
    f = open("/sys/class/thermal/thermal_zone0/temp", "r")
    lines = f.readlines()
    line = lines[0].strip()
    temp2 = int(line, 10)/1000.0
    f.close()
except:
    pass

try:
    out = subprocess.Popen(["/usr/bin/vcgencmd", "measure_temp"], stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    stdout, stderr = out.communicate()
    stdout = stdout.decode()
    lines = stdout.split('\n')
    temp = re.match("temp=(.*)'C", lines[0]).group(1)
    temp3 = float(temp)
except:
    pass

values = json.loads(response)
pi = {}
pi['cpu'] = temp2
pi['gpu'] = temp3
values['pi'] = pi 

print(json.dumps(values))
sys.exit(0)
