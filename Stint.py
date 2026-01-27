# Stint Telemetry Data Logger
# simula un data logger para emitir telemetria en tiempo real simulando sensores; gps, aerodinamica, entre otros.
import ac
import acsys
import sys
import os.path
import platform

if platform.architecture()[0] == "64bit":
    sysdir = os.path.join(os.path.dirname(__file__), 'stdlib64')
    sys.path.insert(0, sysdir)
    os.environ['PATH'] = os.environ['PATH'] + ";."

import socket
import json
import configparser
import math

try:
    from features.radar import RadarSystem
except Exception as e:
    ac.log("Stint ERROR: 'radar.py' not found: " + str(e))

SERVER_IP = "127.0.0.1"
SERVER_PORT = 9996
UPDATE_FREQ = 20 # 20Hz
SLOW_FREQ = 2.0 # 2s
PACKET_TYPE = 2
PACKET_TYPE_S = 3

sock = None
radar_sys = None
lbl_status = 0

timer_fast = 0
timer_slow = 0
period_fast = 1.0 / float(UPDATE_FREQ)
period_slow = SLOW_FREQ

DRIVER_NAME = "Driver"
CAR_MODEL = "UNKNOWN"
TEAM_ID = "DMG"

def load_config():

    global SERVER_IP, SERVER_PORT, UPDATE_FREQ, TEAM_ID, period_fast, period_slow

    try:
        config = configparser.ConfigParser()
        config_path = os.path.join(os.path.dirname(__file__), 'config.ini')
        config.read(config_path)

        if config.has_section("SETTINGS"):
            SERVER_IP = config.get("SETTINGS", "SERVER_IP", fallback="127.0.0.1")
            SERVER_PORT = config.getint("SETTINGS", "SERVER_PORT", fallback=9996)
            UPDATE_FREQ = config.getint("SETTINGS", "UPDATE_FREQ", fallback=20)
        if config.has_section("DRIVER"):
            TEAM_ID = config.get("DRIVER", "TEAM_ID", fallback="DMG")
        
        if UPDATE_FREQ <= 0: UPDATE_FREQ = 20
        period_fast = 1.0 / float(UPDATE_FREQ)

    except Exception as e:
        ac.log("Stint ERROR: config.ini: " + str(e))


def get_nose_direction():
    try:
        # obtiene vector de direccion local [x, y, z]        
        vel = ac.getCarState(0, acsys.CS.Velocity) # devuelve vector [x, y, z]
        speed = ac.getCarState(0, acsys.CS.SpeedKMH)
        
        if speed > 5:
            # math.atan2(x, z) da el angulo en radianes
            return round(math.atan2(vel[0], vel[2]),2)
        
        else:
            return 0.0 # si estamos quietos = 0 norte
    except:
        return 0.0

def send_telemetry():
    
    global sock, radar_sys, SERVER_IP, SERVER_PORT
    
    try:
        
        position = ac.getCarRealTimeLeaderboardPosition(0) + 1

        rpm = int(ac.getCarState(0, acsys.CS.RPM))
        speed = round(ac.getCarState(0, acsys.CS.SpeedKMH),1)
        gear = ac.getCarState(0, acsys.CS.Gear) - 1
        throttle = round(ac.getCarState(0, acsys.CS.Gas),2)
        brake = round(ac.getCarState(0, acsys.CS.Brake),2)
        steer = round(ac.getCarState(0, acsys.CS.Steer),2)

        drag = round(ac.ext_getDrag(),2)
        downforce = round(ac.ext_getDownforce(2),2) # total (0 = front, 1 = rear, pero da 0)
        cl_front = round(ac.getCarState(0, acsys.AERO.CL_Front),4)
        cl_rear = round(ac.getCarState(0, acsys.AERO.CL_Rear),4)
        cd_aero = round(ac.getCarState(0, acsys.AERO.CD),4)
        nose_dir = get_nose_direction()

        pos = ac.getCarState(0, acsys.CS.WorldPosition)

        x = round(pos[0],2)
        z = round(pos[2],2)

        nearby_cars = []

        if radar_sys:
            nearby_cars = radar_sys.get_nearby_cars(x, z)

        tl_payload = {
            "type": PACKET_TYPE,
            "car": {
                "position": position,
                "rpm": rpm,
                "speed": speed,
                "gear": gear,
                "throttle": throttle,
                "brake": brake,
                "steer": steer,
                "aero": {
                    "drag": drag,
                    "downForce": downforce,
                    "clFront": cl_front,
                    "clRear": cl_rear,
                    "cdAero": cd_aero
                },
                "gps": {
                    "noseDir": nose_dir,
                    "x": x,
                    "z": z
                },
                "radar": nearby_cars
            }
        }
        # ac.log("STINT: " + json.dumps(tl_payload, indent=2))

        if sock:
            msg = json.dumps(tl_payload).encode('utf-8')
            sock.sendto(msg, (SERVER_IP, SERVER_PORT))

    except Exception as e:
        ac.log("STINT ERROR: TL " + str(e))

def send_handshake():
    
    global sock, SERVER_IP, SERVER_PORT, DRIVER_NAME, CAR_MODEL
    
    try:
        s_payload = {
            "type": PACKET_TYPE_S,
            "driver": DRIVER_NAME,
            "teamId": TEAM_ID,
            "car": CAR_MODEL
        }
        
        if sock:
            msg = json.dumps(s_payload).encode('utf-8')
            sock.sendto(msg, (SERVER_IP, SERVER_PORT))
            
    except Exception as e:
        ac.log("Stint ERROR: HS " + str(e))

def acMain(ac_version):

    global sock, lbl_status, radar_sys, DRIVER_NAME, CAR_MODEL
    
    load_config()

    try:
        radar_sys = RadarSystem(radar_range=40.0)
        radar_sys.scan_grid()
        
    except:
        ac.log("Stint ERROR: Initializing Radar system")

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(0)
    except:
        ac.log("Stint ERROR: Socket")

    appWindow = ac.newApp("Stint Logger")
    ac.setSize(appWindow, 200, 50)
    ac.setPosition(lbl_status, 10, 25)
    
    DRIVER_NAME = ac.getDriverName(0)
    CAR_MODEL = ac.getCarName(0) or "UNKNOWN"

    ac.log("Stint: READING data")

    return "Stint"

def acUpdate(deltaT):

    global timer_fast, timer_slow, period_fast, period_slow

    timer_fast += deltaT
    timer_slow += deltaT
    
    if timer_fast > period_fast:
        send_telemetry()
        timer_fast = 0
        

    if timer_slow > period_slow:
        send_handshake()
        timer_slow = 0