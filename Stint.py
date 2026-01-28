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

try:
    from features.sim_info import SimInfo
except Exception as e:
    ac.log("Stint ERROR: 'sim_info.py' not found: " + str(e))

try:
    from features.radar import RadarSystem
except Exception as e:
    ac.log("Stint ERROR: 'radar.py' not found: " + str(e))

# valores DEFAULT
SERVER_IP = "127.0.0.1"
SERVER_PORT = 9996
UPDATE_FREQ = 20 # 20Hz
UPDATE_SLOW_FREQ = 5 # 5s

sock = None
sim_info = None
radar_sys = None

timer_fast = 0
timer_slow = 0
tick = 0
period_fast = 1.0 / float(UPDATE_FREQ)
period_slow = UPDATE_SLOW_FREQ

HEADER_STRUCT = struct.Struct('<BB')
INFO_STRUCT = struct.Struct('<4s32s20sB10s?')
INPUT_STRUCT = struct.Struct('<I8fb')
DYNAMICS_STRUCT = struct.Struct('<4f3f')
TIMING_STRUCT = struct.Struct('<BIfBIIIH?B')
TYRE_STRUCT = struct.Struct('<10s20f')
AERO_STRUCT = struct.Struct('<7f')
GPS_BASE_STRUCT = struct.Struct('<3fB')
RADAR_BLIP_STRUCT = struct.Struct('<B2f')

PKT_INFO = 1
PKT_INPUT = 2
PKT_SUSP_GFORCE = 3
PKT_LIVE_TIMING = 4
PKT_TYRE = 5
PKT_AERO = 6
PKT_GPS_RADAR = 7

DRIVER_IDX = 1
DRIVER_NAME = "Driver"
CAR_MODEL = "UNKNOWN"
CAR_NUMBER = "0"
TEAM_ID = "DMG"

RADAR_RANGE = 40.0 # 40m
MAX_RADAR_CARS = 30

def load_config():

    global SERVER_IP, SERVER_PORT, UPDATE_FREQ, UPDATE_SLOW_FREQ, DRIVER_IDX, TEAM_ID, DIV_MID, DIV_SLOW, period_fast, period_slow
    global RADAR_RANGE

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
            val = config.get("DRIVER", "DRIVER_IDX", fallback="").strip()
            if val:
                DRIVER_IDX = int(val)

            val = config.get("DRIVER", "TEAM_ID", fallback="").strip()
            if val:
                TEAM_ID = val

        if config.has_section("SENSORS"):
            val = config.get("SENSORS", "RADAR_RANGE", fallback="").strip()
            if val:
                RADAR_RANGE = float(val)
        
        UPDATE_FREQ = int(max(1, min(UPDATE_FREQ, 60)))
        UPDATE_SLOW_FREQ = max(0.5, min(UPDATE_SLOW_FREQ, 60.0))
        period_fast = 1.0 / float(UPDATE_FREQ)
        period_slow = UPDATE_SLOW_FREQ

        RADAR_RANGE = max(5.0, min(RADAR_RANGE, 200.0))

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

def send_udp(pkt_id, binary_body):

    global sock, SERVER_IP, SERVER_PORT, DRIVER_IDX

    if sock:
        try:
            head = HEADER_STRUCT.pack(pkt_id, DRIVER_IDX)
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
        gear = ac.getCarState(0, acsys.CS.Gear) - 1
        throttle = ac.getCarState(0, acsys.CS.Gas)
        brake = ac.getCarState(0, acsys.CS.Brake)
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
            steer,
            fuel,
            kers_charge,
            kers_input
        )

        send_udp(PKT_INPUT, packet_body)
        
    except:
        pass

def send_susp_gforce_data():

    global sim_info

    try:
        if sim_info is None:
            return

        st = sim_info.physics.suspensionTravel
        ag = sim_info.physics.accG     

        packet_body = DYNAMICS_STRUCT.pack(
            st[0], st[1], st[2], st[3], # FL, FR, RL, RR
            ag[0], ag[1], ag[2]         # X, Y, Z
        )

        send_udp(PKT_SUSP_GFORCE, packet_body)
        
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

        send_udp(PKT_LIVE_TIMING, packet_body)

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
        slip = sim_info.physics.wheelSlip

        packet_body = TYRE_STRUCT.pack(
            tyre_compound,
            temps[0], temps[1], temps[2], temps[3],
            press[0], press[1], press[2], press[3],
            dirt[0], dirt[1], dirt[2], dirt[3],
            wear[0], wear[1], wear[2], wear[3],
            slip[0], slip[1], slip[2], slip[3]
        )

        send_udp(PKT_TYRE, packet_body)
        
    except:
        pass

def send_aero_data():

    global sim_info

    try:
        if sim_info is None:
            return

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
            cd_aero,
            *sim_info.physics.rideHeight
        )

        send_udp(PKT_AERO, packet_body)
        
    except:
        pass

def send_gps_radar_data():
    
    global radar_sys, sim_info

    try:
        if sim_info is None:
            return

        nose_dir = sim_info.physics.heading
        pos = ac.getCarState(0, acsys.CS.WorldPosition)

        x = round(pos[0],2)
        z = round(pos[2],2)

        nearby_cars = []
        if radar_sys:
            nearby_cars = radar_sys.get_nearby_cars(x, z, limit=MAX_RADAR_CARS)

        car_count = len(nearby_cars)
        packet_body = GPS_BASE_STRUCT.pack(
            nose_dir,
            x,
            z,
            car_count
        )

        for car in nearby_cars:
            c_id = int(car.get('id', 0))
            c_x = float(car.get('x', 0.0))
            c_z = float(car.get('z', 0.0))
            packet_body += RADAR_BLIP_STRUCT.pack(
                c_id,
                c_x,
                c_z
                )
            
        send_udp(PKT_GPS_RADAR, packet_body)

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

        pos = int(ac.getCarRealTimeLeaderboardPosition(0) + 1)
        tyre_compound = ac.getCarTyreCompound(0) or "NC"
        compound = tyre_compound.encode('utf-8')[:10]
        in_pit_box = bool(ac.isCarInPit(0))

        packet_body = INFO_STRUCT.pack(
            num,
            driver,
            team_id,
            pos,
            compound,
            in_pit_box
        )

        send_udp(PKT_INFO, packet_body)

    except:
        pass

def acMain(ac_version):

    global sock, lbl_status, radar_sys, DRIVER_NAME, CAR_MODEL, CAR_NUMBER
    global sim_info
    
    load_config()

    try:
        sim_info = SimInfo()
    except:
        ac.log("Stint ERROR: Reading sim info")

    try:
        radar_sys = RadarSystem(radar_range=RADAR_RANGE)
        radar_sys.scan_grid()
        
    except:
        ac.log("Stint ERROR: Initializing Radar system")

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
        send_susp_gforce_data()

        if tick % DIV_MID == 0:
            send_live_timing_data()
        elif tick % DIV_MID == 1:
            send_gps_radar_data()

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