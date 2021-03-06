#!/usr/bin/python3

import os
import glob
from simple_pid import PID
import RPi.GPIO as GPIO  
import time              
 
# Read the 2 lines in w1_slave file 
def read_temp_raw():
    f = open(device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines

# Parse the 2 lines and convert to temperature
def read_temp():
    # Get full input
    lines = read_temp_raw()
    # Read file until valid CRC is detected
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_temp_raw()
    # Get temperature (mC)
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        # Convert string to temperature
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        temp_f = temp_c * 9.0 / 5.0 + 32.0
        return temp_c, temp_f

#======================
# 1-wire initialization
#======================
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')
base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

#===================
# PID initialization
#===================
Kp = 1
Ki = 1
Kd = 1
sample_time = 0.01

# desired value in deg C
desired = 25

pid = PID(Kp, Ki, Kd, setpoint = desired)

# update system every 'sample_time' seconds
pid.sample_time = sample_time

# output limited between 0% and 100%
pid.output_limits = (-50, 50)
#pid.proportional_on_measurement = True

#===================
# PWM initialization
#===================
pin = 12
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin, GPIO.OUT)
GPIO.output(pin, GPIO.LOW)
# set frequency to 1kHz
pwm = GPIO.PWM(pin, 1000)
# initialize PWM to 0% duty cycle
pwm.start(0)
control = 0

while True:
    # update PWM speed based on previous control value
    pwm.ChangeDutyCycle(control)
    
    # read new temperature
    tempC = read_temp()[0]

    # get new output from PID controller
    output = pid(tempC)

    control = -1 * output + 50

    print(tempC, control)
    


    
    


