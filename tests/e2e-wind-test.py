#!/usr/bin/env python3
"""
E2E Test: With wind enabled, the drone exceeds a 10m corridor;
the security module must automatically initiate a landing.
"""
import sys, time, math
from pymavlink import mavutil
 
# Configuration
DRONE_ADDR  = "tcp:172.28.0.2:5760"
CORRIDOR_M  = 10.0
TIMEOUT_S   = 180
ARM_TIMEOUT = 60
LAND_MODE   = 9   # ArduCopter: LAND mode = 9
 
def dist_meters(lat1, lon1, lat2, lon2):
    """Calculates the distance between two points in meters using GPS coordinates"""
    dlat = (lat1 - lat2) / 1e7 * 111320.0
    dlon = (lon1 - lon2) / 1e7 * 111320.0 * math.cos(math.radians(lat1 / 1e7))
    return math.sqrt(dlat**2 + dlon**2)
 
# Establish connection
master = mavutil.mavlink_connection(DRONE_ADDR)
master.wait_heartbeat(timeout=30)
print(f"[TEST] Connected!")
 
# ARMING the motors
t0 = time.time()
while time.time() - t0 < ARM_TIMEOUT:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        if ack.result == 0:
            print("[TEST] ARM successful!")
            break
    time.sleep(2)
 
# Mission Start
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
time.sleep(3)
 
# Initial position
msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=10)
start_lat, start_lon = msg.lat, msg.lon
print(f"[TEST] Start: lat={start_lat} lon={start_lon}")
 
# Monitoring for corridor breach and landing
t0, max_dist, success = time.time(), 0.0, False
while time.time() - t0 < TIMEOUT_S:
    pos = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    hb  = master.recv_match(type='HEARTBEAT', blocking=False)
    if pos:
        dist = dist_meters(pos.lat, pos.lon, start_lat, start_lon)
        max_dist = max(max_dist, dist)
        print(f"[TEST] dist={dist:.1f}m  max={max_dist:.1f}m", end="\r")
        
        # Verify: is the drone in LAND mode AND did it exceed the allowed distance?
        if hb and hb.custom_mode == LAND_MODE and dist > CORRIDOR_M:
            success = True
            break
    time.sleep(0.5)
 
print()
if success:
    print(f"[TEST] PASS: drone drifted by {max_dist:.1f} m, landing triggered")
    sys.exit(0)
else:
    print(f"[TEST] FAIL: landing not triggered, max deviation {max_dist:.1f} m")
    sys.exit(1)
