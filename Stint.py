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
import struct
import time
from math import atan2

try:
    from modules.sim_info import SimInfo
except Exception as e:
    ac.log("Stint ERROR: 'sim_info.py' not found: " + str(e))

# valores DEFAULT
SERVER_IP = "127.0.0.1"
SERVER_PORT = 9996
UPDATE_FREQ = 20 # 20Hz
UPDATE_SLOW_FREQ = 5 # 5s

sock = None
sim_info = None

timer_fast = 0
timer_slow = 0
tick = 0
period_fast = 1.0 / float(UPDATE_FREQ)
period_slow = UPDATE_SLOW_FREQ

HEADER_STRUCT = struct.Struct('<BBQ')
INFO_STRUCT = struct.Struct('<4s32s20s?6f??')
INPUT_STRUCT = struct.Struct('<I10f')
IMU_STRUCT = struct.Struct('<8f')
SUSP_STRUCT = struct.Struct('<18f')
TIMING_STRUCT = struct.Struct('<BIfBIIIH?B')
TYRE_STRUCT = struct.Struct('<10s20f')
AERO_STRUCT = struct.Struct('<5f')
GPS_STRUCT = struct.Struct('<3f')

PKT_INFO = 1
PKT_INPUT = 2
PKT_IMU = 3
PKT_SUSP = 4
PKT_LIVE_TIMING = 5
PKT_GPS = 6
PKT_TYRE = 7
PKT_AERO = 8

DEVICE_ID = 1
DRIVER_NAME = "Driver"
CAR_MODEL = "UNKNOWN"
CAR_NUMBER = "0"
TEAM_ID = "DMG"

def load_config():

    global SERVER_IP, SERVER_PORT, UPDATE_FREQ, UPDATE_SLOW_FREQ, DEVICE_ID, TEAM_ID, DIV_MID, DIV_SLOW, period_fast, period_slow

    try:
        config = configparser.ConfigParser()
        config_path = os.path.join(os.path.dirname(__file__), 'config.ini')
        config.read(config_path)

        if config.has_section("SETTINGS"):
            val = config.get("SETTINGS", "SERVER_IP", fallback="").strip()
            if val:
                SERVER_IP = val
            
            val = config.get("SETTINGS", "SERVER_PORT", fallback="").strip()
            if val:
                SERVER_PORT = int(val)
            
            val = config.get("SETTINGS", "UPDATE_FREQ", fallback="").strip()
            if val:
                UPDATE_FREQ = int(val)

            val = config.get("SETTINGS", "UPDATE_SLOW_FREQ", fallback="").strip()
            if val:
                UPDATE_SLOW_FREQ = float(val)
        
        if config.has_section("DRIVER"):
            val = config.get("DRIVER", "DEVICE_ID", fallback="").strip()
            if val:
                DEVICE_ID = int(val)

            val = config.get("DRIVER", "TEAM_ID", fallback="").strip()
            if val:
                TEAM_ID = val
        
        UPDATE_FREQ = int(max(1, min(UPDATE_FREQ, 60)))
        UPDATE_SLOW_FREQ = max(0.5, min(UPDATE_SLOW_FREQ, 60.0))
        period_fast = 1.0 / float(UPDATE_FREQ)
        period_slow = UPDATE_SLOW_FREQ

        # logica de sincronizacion
        if UPDATE_FREQ >= 60:
            DIV_MID = 4   # 60/4 = 15Hz
            DIV_SLOW = 12 # 60/12 = 5Hz
        elif UPDATE_FREQ >= 30:
            DIV_MID = 2   # 30/2 = 15Hz
            DIV_SLOW = 6  # 30/6 = 5Hz
        elif UPDATE_FREQ >= 20:
            DIV_MID = 2   # 20/2 = 10Hz
            DIV_SLOW = 4  # 20/4 = 5Hz
        elif UPDATE_FREQ >= 10:
            DIV_MID = 1 # 10/1 = 10Hz
            DIV_SLOW = 2 # 10/2 = 5Hz
        else:
            DIV_MID = 1
            DIV_SLOW = 1

    except Exception as e:
        ac.log("Stint ERROR: config.ini: " + str(e))

def get_number_from_livery(car_model, skin_name):
    try:
        file_path = "content/cars/{}/skins/{}/ui_skin.json".format(car_model, skin_name)
        
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                data = json.load(f)
                return str(data.get("number", ""))

    except Exception:
        return None
    return None

def send_udp_pkt(pkt_id, binary_body):

    global sock, SERVER_IP, SERVER_PORT, DEVICE_ID

    if sock:
        try:
            timestamp = int(time.time() * 1000000)

            head = HEADER_STRUCT.pack(pkt_id, DEVICE_ID, timestamp)
            msg = head + binary_body
            sock.sendto(msg, (SERVER_IP, SERVER_PORT))
        except:
            pass


def send_input_data():

    global sim_info

    try:
        if sim_info is None:
            return
        
        rpm = int(ac.getCarState(0, acsys.CS.RPM))
        turbo = ac.getCarState(0, acsys.CS.TurboBoost) #kpa
        speed = ac.getCarState(0, acsys.CS.SpeedKMH)
        gear = int(ac.getCarState(0, acsys.CS.Gear) - 1)
        throttle = ac.getCarState(0, acsys.CS.Gas)
        brake = ac.getCarState(0, acsys.CS.Brake)
        clutch = ac.getCarState(0, acsys.CS.Clutch)
        steer = ac.getCarState(0, acsys.CS.Steer)
        fuel = sim_info.physics.fuel
        kers_charge = sim_info.physics.kersCharge
        kers_input = sim_info.physics.kersInput

        packet_body = INPUT_STRUCT.pack(
            rpm,
            turbo,
            speed,
            gear,
            throttle,
            brake,
            clutch,
            steer,
            fuel,
            kers_charge,
            kers_input
        )

        send_udp_pkt(PKT_INPUT, packet_body)
        
    except:
        pass

def send_imu_data():

    global sim_info

    try:
        if sim_info is None:
            return
        
        ag = sim_info.physics.accG
        r = sim_info.physics.roll
        p = sim_info.physics.pitch
        agv = ac.getCarState(0, acsys.CS.LocalAngularVelocity)
        yawr = agv[1] # yaw_rate: rad/s
        vel = ac.getCarState(0, acsys.CS.LocalVelocity) # [x,y,z]
        sl = atan2(vel[0], vel[2]) # side_slip: rad  - derivado, de uso referencial
        cgh = sim_info.physics.cgHeight

        packet_body = IMU_STRUCT.pack(
            ag[0], ag[1], ag[2], # X, Y, Z
            r,
            p,
            yawr,
            sl,
            cgh
        )

        send_udp_pkt(PKT_IMU, packet_body)

    except:
        pass

def send_suspension_data():

    global sim_info

    try:
        if sim_info is None:
            return

        st = sim_info.physics.suspensionTravel
        cmb = sim_info.physics.camberRAD
        w_l = sim_info.physics.wheelLoad
        w_asp = sim_info.physics.wheelAngularSpeed
        rh = sim_info.physics.rideHeight

        # FL, FR, RL, RR, rh: FRONT, REAR
        packet_body = SUSP_STRUCT.pack(
            st[0], st[1], st[2], st[3],
            cmb[0], cmb[1], cmb[2], cmb[3],
            w_l[0], w_l[1], w_l[2], w_l[3],
            w_asp[0], w_asp[1], w_asp[2], w_asp[3],
            rh[0], rh[1]
        )

        send_udp_pkt(PKT_SUSP, packet_body)
        
    except:
        pass

def send_live_timing_data():
    
    global sim_info

    try:
        if sim_info is None:
            return

        pos = ac.getCarRealTimeLeaderboardPosition(0) + 1
        current_lap_ms = int(ac.getCarState(0, acsys.CS.LapTime))
        delta = ac.getCarState(0, acsys.CS.PerformanceMeter)
        sector_idx = sim_info.graphics.currentSectorIndex + 1
        sector_time = int(sim_info.graphics.lastSectorTime)
        last_lap_ms = int(ac.getCarState(0, acsys.CS.LastLap))
        best_lap_ms = int(ac.getCarState(0, acsys.CS.BestLap))
        lap_num = int(ac.getCarState(0, acsys.CS.LapCount) + 1)
        in_pit_lane = bool(ac.isCarInPitline(0))
        flag = sim_info.graphics.flag

        packet_body = TIMING_STRUCT.pack(
            pos,
            current_lap_ms,
            delta,
            sector_idx,
            sector_time,
            last_lap_ms,
            best_lap_ms,
            lap_num,
            in_pit_lane,
            flag
        )

        send_udp_pkt(PKT_LIVE_TIMING, packet_body)

    except:
        pass

def send_tyre_data():

    global sim_info

    try:
        if sim_info is None:
            return
        
        compound = ac.getCarTyreCompound(0) or "NC"
        tyre_compound = compound.encode('utf-8')[:10]

        temps = ac.getCarState(0, acsys.CS.CurrentTyresCoreTemp)
        press = ac.getCarState(0, acsys.CS.DynamicPressure)
        dirt = ac.getCarState(0, acsys.CS.TyreDirtyLevel)
        wear = sim_info.physics.tyreWear
        slip = sim_info.physics.wheelSlip # derivado, pero util como referencia desde el motor de ac

        packet_body = TYRE_STRUCT.pack(
            tyre_compound,
            temps[0], temps[1], temps[2], temps[3],
            press[0], press[1], press[2], press[3],
            dirt[0], dirt[1], dirt[2], dirt[3],
            wear[0], wear[1], wear[2], wear[3],
            slip[0], slip[1], slip[2], slip[3]
        )

        send_udp_pkt(PKT_TYRE, packet_body)
        
    except:
        pass

def send_aero_data():

    try:
        drag = ac.ext_getDrag()
        downforce = ac.ext_getDownforce(2) # total (0 = front, 1 = rear, pero da 0)
        cl_front = ac.getCarState(0, acsys.AERO.CL_Front)
        cl_rear = ac.getCarState(0, acsys.AERO.CL_Rear)
        cd_aero = ac.getCarState(0, acsys.AERO.CD)

        packet_body = AERO_STRUCT.pack(
            drag,
            downforce,
            cl_front,
            cl_rear,
            cd_aero
        )

        send_udp_pkt(PKT_AERO, packet_body)
        
    except:
        pass

def send_gps_data():
    
    global sim_info

    try:
        if sim_info is None:
            return

        nose_dir = sim_info.physics.heading
        pos = ac.getCarState(0, acsys.CS.WorldPosition)

        x = round(pos[0],2)
        z = round(pos[2],2)

        packet_body = GPS_STRUCT.pack(
            nose_dir,
            x,
            z
        )

        send_udp_pkt(PKT_GPS, packet_body)

    except:
        pass

def send_info():
    
    global sim_info, DRIVER_NAME, CAR_NUMBER, TEAM_ID
    
    try:
        if sim_info is None:
            return
        
        num = (str(CAR_NUMBER) or "").encode('utf-8')[:4]
        driver = (str(DRIVER_NAME) or "Driver").encode('utf-8')[:32]
        team_id = (str(TEAM_ID) or "DMG").encode('utf-8')[:20]
        in_pit_box = bool(ac.isCarInPit(0))
        dist = sim_info.graphics.distanceTraveled
        c_dmg = sim_info.physics.carDamage
        tc_on = bool(sim_info.physics.tc > 0.0)
        abs_on = bool(sim_info.physics.abs > 0.0)

        packet_body = INFO_STRUCT.pack(
            num,
            driver,
            team_id,
            in_pit_box,
            dist,
            c_dmg[0], c_dmg[1], c_dmg[2], c_dmg[3], c_dmg[4],
            tc_on,
            abs_on
        )

        send_udp_pkt(PKT_INFO, packet_body)

    except:
        pass

def acMain(ac_version):

    global sock, lbl_status, DRIVER_NAME, CAR_MODEL, CAR_NUMBER
    global sim_info
    
    load_config()

    try:
        sim_info = SimInfo()
    except:
        ac.log("Stint ERROR: Reading sim info")

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(0)
    except:
        ac.log("Stint ERROR: Socket")

    appWindow = ac.newApp("Stint")
    ac.setSize(appWindow, 200, 50)
    
    livery = ac.getCarSkin(0)
    DRIVER_NAME = ac.getDriverName(0)
    CAR_MODEL = ac.getCarName(0) or "UNKNOWN"
    CAR_NUMBER = get_number_from_livery(CAR_MODEL, livery)

    ac.log("Stint sending telemetry data")

    return "Stint"

def acUpdate(deltaT):

    global timer_fast, timer_slow, period_fast, period_slow, tick, DIV_MID, DIV_SLOW

    timer_fast += deltaT
    timer_slow += deltaT
    
    if timer_fast > period_fast:
        send_input_data()
        send_suspension_data()
        send_imu_data()

        if tick % DIV_MID == 0:
            send_live_timing_data()
        elif tick % DIV_MID == 1:
            send_gps_data()

        slow_cycle = tick % DIV_SLOW
        
        if slow_cycle == 2:
            send_tyre_data()
        elif slow_cycle == 3:
            send_aero_data()
        elif slow_cycle == 4:
            pass

        tick += 1
        timer_fast = 0
        if tick > 1000: tick = 0 #---- no es mandatorio, pero reinicio preventivo

    if timer_slow > period_slow:
        send_info()
        timer_slow = 0