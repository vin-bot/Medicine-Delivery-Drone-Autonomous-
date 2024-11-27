       # import required modules
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import math
import socket
import argparse
import geopy.distance
import serial
import numpy as np
import time
import firebase_admin
from firebase_admin import db,credentials
import random


import RPi.GPIO as GPIO  # Imports the standard Raspberry Pi GPIO library
from time import sleep

serial_port = "/dev/ttyAMA0"  # This might be different based on your Raspberry Pi model
baud_rate = 9600
# Initialize the serial connection
ser = serial.Serial(serial_port, baud_rate, timeout=1)

list = []

# authenticate to firebase
cred = credentials.Certificate("/home/dronepi/credentials.json")
firebase_admin.initialize_app(cred, {"databaseURL": "https://sample-a028e-default-rtdb.firebaseio.com/"})

ref = db.reference("/")
# retrieving data from root node
ref.get()
print(ref.get())


def connectMyCopter():
    parser =  argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect
    baud_rate = 57600
    print("\nConnecting to vehicle on: %s" % connection_string)
    vehicle = connect(connection_string ,baud=baud_rate ,wait_ready=True)
    return  vehicle

def get_dstance(cord1, cord2):
    # return distance n meter
    return (geopy.distance.geodesic(cord1, cord2).km ) *1000

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)


    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(2)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    time.sleep(3)



    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def goto_location(to_lat, to_long):

    print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    curr_lat = vehicle.location.global_relative_frame.lat
    curr_lon = vehicle.location.global_relative_frame.lon
    curr_alt = vehicle.location.global_relative_frame.alt

    # set to locaton (lat, lon, alt)
    to_lat = to_lat
    to_lon = to_long
    to_alt = curr_alt

    to_pont = LocationGlobalRelative(to_lat ,to_lon ,to_alt)
    vehicle.simple_goto(to_pont, groundspeed=4)

    to_cord = (to_lat, to_lon)
    while True:
        curr_lat = vehicle.location.global_relative_frame.lat
        curr_lon = vehicle.location.global_relative_frame.lon
        curr_cord = (curr_lat, curr_lon)
        print("curr location: {}".format(curr_cord))
        distance = get_dstance(curr_cord, to_cord)
        print("distance ramaining {}".format(distance))
        if distance <= 2:
            print("Reached within 2 meters of target location...")
            break
        time.sleep(1)
def servo():
    GPIO.setmode(GPIO.BOARD)

    # Set up pin 11 for PWM
    GPIO.setup(11, GPIO.OUT)
    p1 = GPIO.PWM(11, 50)  # Servo 1 on pin 11
    p1.start(0)

    # Set up pin 13 for PWM
    GPIO.setup(13, GPIO.OUT)
    p2 = GPIO.PWM(13, 50)  # Servo 2 on pin 13
    p2.start(0)

    try:
            # Move servo 1 back and forth
        p1.ChangeDutyCycle(3)
        time.sleep(5)
        p2.ChangeDutyCycle(3)
        time.sleep(20)

            # Move servo 2 back and forth
        p2.ChangeDutyCycle(12)
        time.sleep(3)
        p1.ChangeDutyCycle(12)
        time.sleep(1)

    except KeyboardInterrupt:
        pass

    finally:
        # At the end of the1 program, stop the PWM and clean up
        p1.stop()
        p2.stop()
        GPIO.cleanup()
    
def gsm():
    
    try:
        
    # Wait for the GSM module to initialize (you may need to adjust this delay)
        time.sleep(5)

    # AT command to set the message mode
        ser.write(b'AT+CMGF=1\r\n')
        time.sleep(1)
        print(ph_num)
    # Replace 'your_phone_number' with the actual phone number you want to send the message to
        phone_number = ph_num

    # Replace 'your_message' with the actual message you want to send
        message = 'Enter ' + str(otp) + ' in message'
    # AT command to set the recipient's phone number
        ser.write(f'AT+CMGS="{phone_number}"\r\n'.encode('utf-8'))
        time.sleep(1)

    # Send the message
        ser.write(f'{message}\r\n'.encode('utf-8'))

    # Send the CTRL+Z character to indicate the end of the message
        ser.write(bytes([26]))

    # Wait for the message to be sent (adjust the delay if needed)
        time.sleep(5)
        db.reference("/" + username + "/" + 'otp').set(message)

    except Exception as e:
        
        print(f"Error: {e}")

    finally:
    # Close the serial connection
        ser.close()  

def send_command(command):
    ser.write(command.encode() + b'\r\n')
    time.sleep(1)
    response = ser.read_all().decode()
    return response

def setup_sim800l():
    send_command('AT')
    send_command('AT+CMGF=1')
    send_command('AT+CNMI=1,2,0,0,0')

def read_serial():
    while ser.in_waiting:
        line = ser.readline().decode().strip()
        list.append(line)
            
        # You can add your SMS handling logic here

def receive_sms():
    setup_sim800l()
    z = 0
    while z == 0:
        read_serial()
        print(list)
        if len(list) >= 3:
            z = 1
            break

data = db.reference("/").get()
for key, value in data.items():
    if ((data[key]['dronestatus']) == "none") and (data[key]['route']['route'] == 'pres'):
        route = 'pres'
        ph_num = (data[key]['userDetails']['phone'])
        username = (data[key]['userDetails']['username'])
        user_lat = (data[key]['userDetails']['userLat'])
        user_long = (data[key]['userDetails']['userLong'])
        shop_lat =23.249900;         
        shop_long = 77.523503;
        break
    elif (((data[key]['route']['route'] == 'shop') and (data[key]['dronestatus']) == "none")):
        route = 'shop'
        ph_num = (data[key]['userDetails']['phone'])
        username = (data[key]['userDetails']['username'])
        shop_lat = (data[key]['shopDetails']['shopLat'])
        shop_long = (data[key]['shopDetails']['shopLong'])
        user_lat = (data[key]['userDetails']['userLat'])
        user_long = (data[key]['userDetails']['userLong'])
        # db.reference("/" + username + "/" + 'dronestatus').set("none")
        break
print(username)
print(user_lat)
print(user_long)
print(shop_lat)
print(shop_long)
otp = random.randint(0000,9999)

if route == 'pres':
    vehicle = connectMyCopter()
    home_lat = vehicle.location.global_relative_frame.lat
    home_long = vehicle.location.global_relative_frame.lon
    time.sleep(5)
    ht = 10
    arm_and_takeoff(ht)
    time.sleep(1)
    goto_location(shop_lat ,shop_long)
    time.sleep(2)
    vehicle.mode = VehicleMode("LAND")
    time.sleep(30)
    servo()
    time.sleep(10)
    arm_and_takeoff(ht)
    time.sleep(1)
    goto_location(user_lat ,user_long)
    time.sleep(1)
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    gsm()
    ser.close()
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    receive_sms()
    ser.close()
    if list[-1] == str(otp):
        vehicle.mode = VehicleMode("LAND")
        time.sleep(20)
        servo()
        time.sleep(5)
        arm_and_takeoff(ht)
        goto_location(home_lat ,home_long)
        time.sleep(2)
        vehicle.mode = VehicleMode("LAND")
    else:
        arm_and_takeoff(ht)
        goto_location(home_lat ,home_long)
        time.sleep(2)
        vehicle.mode = VehicleMode("LAND")
    
elif route == 'shop':
    vehicle = connectMyCopter()
    
    home_lat = vehicle.location.global_relative_frame.lat
    home_long = vehicle.location.global_relative_frame.lon
    
    time.sleep(5)
    ht = 10
    arm_and_takeoff(ht)
    time.sleep(1)
    goto_location(shop_lat ,shop_long)
    time.sleep(2)
    vehicle.mode = VehicleMode("LAND")
    time.sleep(30)
    servo()
    time.sleep(10)
    arm_and_takeoff(ht)
    time.sleep(1)
    goto_location(user_lat ,user_long)
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    gsm()
    ser.close()
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    receive_sms()
    ser.close()
    if list[-1] == str(otp):
        vehicle.mode = VehicleMode("LAND")
        time.sleep(20)
        servo()
        time.sleep(5)
        arm_and_takeoff(ht)
        goto_location(home_lat ,home_long)
        time.sleep(2)
        vehicle.mode = VehicleMode("LAND")
    else:
        arm_and_takeoff(ht)
        goto_location(home_lat ,home_long)
        time.sleep(2)
        vehicle.mode = VehicleMode("LAND")


db.reference("/" + username + "/" + 'dronestatus').set("done")
#goes to the shopkeeper
#vehicle = connectMyCopter()
#time.sleep(1)
#ht = 10
#arm_and_takeoff(ht)
#time.sleep(1)
#goto_location(shop_lat ,shop_long)
#time.sleep(2)
#vehicle.mode = VehicleMode("LAND")

#goes to the customer
    
# print(username)
# print(user_lat)
# print(user_long)
# print(shop_lat)
# print(shop_long)
