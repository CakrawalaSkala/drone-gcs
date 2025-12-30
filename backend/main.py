import asyncio
import json
import math
import websockets
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)
from mavsdk.mission import (MissionItem, MissionPlan)

# --- KONFIGURASI KONEKSI ---
# SITL Default
CONNECTION_STRING = "udp://:14550" 
# Telemetry Radio (Drone Asli)
# CONNECTION_STRING = "serial://COM3:57600"

WS_PORT = 8080

# State Global
current_telemetry = {
    "connected": False, "armed": False, "mode": "UNKNOWN",
    "battery_voltage": 0, "battery_remaining": 0,
    "latitude": 0, "longitude": 0, "altitude_relative": 0,
    "heading": 0, "pitch": 0, "roll": 0,
    "satellites": 0, "ground_speed": 0, "climb_rate": 0,
    "health_gyro": False, "health_accel": False, "health_mag": False, "health_gps": False,
    "health_home": False, # [NEW] Status Home Position
    "rc_rssi": 0          # [NEW] Kekuatan Sinyal Remote
}

connected_clients = set()
drone_system = None
is_offboard_active = False

# --- HELPER: Kalkulasi Koordinat ---
def get_offset_location(lat, lon, d_north, d_east):
    earth_radius = 6378137.0 
    d_lat = d_north / earth_radius
    d_lon = d_east / (earth_radius * math.cos(math.pi * lat / 180))
    new_lat = lat + (d_lat * 180 / math.pi)
    new_lon = lon + (d_lon * 180 / math.pi)
    return new_lat, new_lon

def create_mission_item(lat, lon, alt, speed=5.0, fly_through=True, action=MissionItem.VehicleAction.NONE):
    return MissionItem(
        latitude_deg=lat, longitude_deg=lon, relative_altitude_m=alt,
        speed_m_s=speed, is_fly_through=fly_through,
        gimbal_pitch_deg=float('nan'), gimbal_yaw_deg=float('nan'),
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=float('nan'), camera_photo_interval_s=float('nan'),
        acceptance_radius_m=2.0, yaw_deg=float('nan'), camera_photo_distance_m=float('nan'),
        vehicle_action=action
    )

# --- MISSION GENERATORS ---

async def generate_figure8_mission(altitude=15):
    print("Generating Figure 8 Mission...")
    items = []
    clat, clon = current_telemetry["latitude"], current_telemetry["longitude"]
    if abs(clat) < 0.001: return None

    # Takeoff Point
    items.append(create_mission_item(clat, clon, altitude))

    radius, steps, loops = 25, 16, 1 # Config
    
    for k in range(loops):
        # Circle Right
        for i in range(steps):
            angle = (2 * math.pi * i) / steps
            dn, de = radius * math.cos(angle), radius + (radius * math.sin(angle)) 
            lat, lon = get_offset_location(clat, clon, dn, de)
            items.append(create_mission_item(lat, lon, altitude, 8.0))
        # Circle Left
        for i in range(steps):
            angle = (2 * math.pi * i) / steps
            dn, de = radius * math.cos(-angle), -radius + (radius * math.sin(-angle))
            lat, lon = get_offset_location(clat, clon, dn, de)
            items.append(create_mission_item(lat, lon, altitude, 8.0))

    # Land
    items.append(create_mission_item(clat, clon, 0, 5.0, False, MissionItem.VehicleAction.LAND))
    return MissionPlan(items)

async def generate_square_mission(altitude=20):
    print("Generating Square Mapping Mission...")
    items = []
    clat, clon = current_telemetry["latitude"], current_telemetry["longitude"]
    if abs(clat) < 0.001: return None

    side = 60 # meter
    # Waypoints: Center -> FL -> FR -> BR -> BL -> Center -> Land
    offsets = [
        (0, 0),                 # Center (Up)
        (side/2, -side/2),      # Front Left
        (side/2, side/2),       # Front Right
        (-side/2, side/2),      # Back Right
        (-side/2, -side/2),     # Back Left
        (0, 0)                  # Center (Return)
    ]

    for dn, de in offsets:
        lat, lon = get_offset_location(clat, clon, dn, de)
        items.append(create_mission_item(lat, lon, altitude, 10.0))
    
    # Land Item
    items.append(create_mission_item(clat, clon, 0, 5.0, False, MissionItem.VehicleAction.LAND))
    return MissionPlan(items)

async def generate_scan_mission(altitude=25):
    print("Generating Grid/Scan Mission...")
    items = []
    clat, clon = current_telemetry["latitude"], current_telemetry["longitude"]
    if abs(clat) < 0.001: return None

    # Parameters
    width = 80   # Total width (East-West)
    length = 100 # Total length (North-South)
    spacing = 20 # Distance between lines
    
    # Start at bottom-left relative to drone
    start_n, start_e = -length/2, -width/2
    num_lines = int(width / spacing) + 1

    items.append(create_mission_item(clat, clon, altitude)) # Takeoff

    for i in range(num_lines):
        # Current East offset
        current_e = start_e + (i * spacing)
        
        # Zig-Zag Logic: Even lines go North, Odd lines go South
        if i % 2 == 0:
            # Go South (Start of line) -> North (End of line)
            l1, ln1 = get_offset_location(clat, clon, start_n, current_e)
            l2, ln2 = get_offset_location(clat, clon, length/2, current_e)
            items.append(create_mission_item(l1, ln1, altitude, 12.0))
            items.append(create_mission_item(l2, ln2, altitude, 12.0))
        else:
            # Go North -> South
            l1, ln1 = get_offset_location(clat, clon, length/2, current_e)
            l2, ln2 = get_offset_location(clat, clon, start_n, current_e)
            items.append(create_mission_item(l1, ln1, altitude, 12.0))
            items.append(create_mission_item(l2, ln2, altitude, 12.0))

    # Return & Land
    items.append(create_mission_item(clat, clon, altitude, 8.0))
    items.append(create_mission_item(clat, clon, 0, 5.0, False, MissionItem.VehicleAction.LAND))
    return MissionPlan(items)

async def generate_spiral_mission(altitude=30):
    print("Generating Spiral Search Mission...")
    items = []
    clat, clon = current_telemetry["latitude"], current_telemetry["longitude"]
    if abs(clat) < 0.001: return None

    items.append(create_mission_item(clat, clon, altitude))

    max_radius = 50
    coils = 5
    steps_per_coil = 16
    total_steps = coils * steps_per_coil
    
    for i in range(total_steps):
        angle = (2 * math.pi * i) / steps_per_coil
        # Radius grows linearly
        current_radius = (i / total_steps) * max_radius
        
        dn = current_radius * math.cos(angle)
        de = current_radius * math.sin(angle)
        
        lat, lon = get_offset_location(clat, clon, dn, de)
        items.append(create_mission_item(lat, lon, altitude, 10.0))

    # Return & Land
    items.append(create_mission_item(clat, clon, altitude, 8.0))
    items.append(create_mission_item(clat, clon, 0, 5.0, False, MissionItem.VehicleAction.LAND))
    return MissionPlan(items)

# --- LOOPS ---
async def telemetry_loop():
    global drone_system
    
    async def get_position():
        async for position in drone_system.telemetry.position():
            current_telemetry["latitude"] = position.latitude_deg
            current_telemetry["longitude"] = position.longitude_deg
            current_telemetry["altitude_relative"] = position.relative_altitude_m

    async def get_velocity():
        async for velocity in drone_system.telemetry.velocity_ned():
            speed = math.sqrt(velocity.north_m_s**2 + velocity.east_m_s**2)
            current_telemetry["ground_speed"] = speed
            current_telemetry["climb_rate"] = -velocity.down_m_s

    async def get_attitude():
        async for attitude in drone_system.telemetry.attitude_euler():
            current_telemetry["roll"] = attitude.roll_deg
            current_telemetry["pitch"] = attitude.pitch_deg
            current_telemetry["heading"] = attitude.yaw_deg

    async def get_battery():
        async for battery in drone_system.telemetry.battery():
            current_telemetry["battery_voltage"] = battery.voltage_v
            current_telemetry["battery_remaining"] = battery.remaining_percent * 100

    async def get_flight_mode():
        async for mode in drone_system.telemetry.flight_mode():
            current_telemetry["mode"] = str(mode)

    async def get_armed():
        async for is_armed in drone_system.telemetry.armed():
            current_telemetry["armed"] = is_armed

    async def get_gps_info():
        async for gps_info in drone_system.telemetry.gps_info():
            current_telemetry["satellites"] = gps_info.num_satellites

    async def get_health():
        async for health in drone_system.telemetry.health():
            current_telemetry["health_gyro"] = health.is_gyrometer_calibration_ok
            current_telemetry["health_accel"] = health.is_accelerometer_calibration_ok
            current_telemetry["health_mag"] = health.is_magnetometer_calibration_ok
            current_telemetry["health_gps"] = health.is_global_position_ok
            current_telemetry["health_home"] = health.is_home_position_ok

    async def get_rc_status():
        async for rc in drone_system.telemetry.rc_status():
            current_telemetry["rc_rssi"] = rc.signal_strength_percent

    try:
        await asyncio.gather(
            get_position(), get_velocity(), get_attitude(), get_battery(),
            get_flight_mode(), get_armed(), get_gps_info(), get_health(), get_rc_status()
        )
    except Exception as e:
        print(f"Telemetry Loop Error: {e}")

async def broadcast_loop():
    while True:
        if connected_clients:
            try:
                data = current_telemetry.copy()
                for k in ["health_gyro", "health_accel", "health_mag", "health_gps", "health_home", "rc_rssi"]:
                    data.pop(k, None)
                message = json.dumps(data)
                await asyncio.gather(*[client.send(message) for client in connected_clients])
            except Exception as e:
                print(f"Broadcast Error: {e}")
        # Broadcast rate increased to 20Hz (0.05s) for smoother 3D visualization
        await asyncio.sleep(0.05)

async def perform_preflight_check():
    report = []
    imu_ok = current_telemetry["health_gyro"] and current_telemetry["health_accel"]
    report.append({"id": "imu", "label": "IMU Sensors", "status": "PASS" if imu_ok else "FAIL", "detail": "Calibrated" if imu_ok else "Calibration Required"})
    
    mag_ok = current_telemetry["health_mag"]
    report.append({"id": "mag", "label": "Magnetometer", "status": "PASS" if mag_ok else "FAIL", "detail": "Ready" if mag_ok else "Interference"})
    
    sats = current_telemetry["satellites"]
    gps_ok = sats >= 6
    report.append({"id": "gps", "label": f"GPS Lock ({sats} Sats)", "status": "PASS" if gps_ok else "WARN", "detail": "3D Lock" if gps_ok else "Acquiring..."})
    
    bat_rem = current_telemetry["battery_remaining"]
    bat_ok = bat_rem > 30
    report.append({"id": "bat", "label": f"Battery ({int(bat_rem)}%)", "status": "PASS" if bat_ok else "WARN", "detail": "Good" if bat_ok else "Low"})
    
    # [NEW] Tambahan Cek RC & Home
    rc_signal = current_telemetry["rc_rssi"]
    rc_ok = rc_signal > 0
    report.append({"id": "rc", "label": f"RC Signal ({rc_signal}%)", "status": "PASS" if rc_ok else "WARN", "detail": "Connected" if rc_ok else "No Signal/Lost"})

    home_ok = current_telemetry["health_home"]
    report.append({"id": "home", "label": "Home Position", "status": "PASS" if home_ok else "FAIL", "detail": "Set" if home_ok else "Not Set (Wait GPS)"})

    return report

async def monitor_mission_progress():
    print("Monitoring Mission Progress...")
    async for progress in drone_system.mission.mission_progress():
        if progress.total > 0 and progress.current >= progress.total:
            if current_telemetry["altitude_relative"] > 1.0:
                print(">>> MISSION COMPLETED. LANDING... <<<")
                try: await drone_system.action.land()
                except: pass
                await asyncio.sleep(5)

async def websocket_handler(websocket):
    global is_offboard_active
    connected_clients.add(websocket)
    print(">>> React Client Terhubung")
    
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                msg_type = data.get('type')

                if msg_type == 'REQ_PREFLIGHT':
                    report = await perform_preflight_check()
                    await websocket.send(json.dumps({"type": "PREFLIGHT_REPORT", "report": report}))

                elif msg_type == 'COMMAND_LONG':
                    if data['command'] == 'MAV_CMD_COMPONENT_ARM_DISARM':
                        should_arm = data['param1'] == 1
                        try:
                            if should_arm: await drone_system.action.arm()
                            else: await drone_system.action.disarm()
                        except Exception as e: print(f"Arm Error: {e}")

                elif msg_type == 'SET_MODE':
                    mode = data['mode']
                    print(f"CMD: Set Mode {mode}")
                    try:
                        if mode == 'MISSION':
                            if is_offboard_active:
                                try: await drone_system.offboard.stop()
                                except: pass
                                is_offboard_active = False
                            if not current_telemetry["armed"]:
                                try: await drone_system.action.arm()
                                except: pass
                            await drone_system.mission.start_mission()
                            print(">>> MISSION STARTED <<<")

                        elif mode == 'OFFBOARD':
                            await drone_system.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                            try: 
                                await drone_system.offboard.start()
                                is_offboard_active = True
                            except OffboardError: pass     
                        else:
                            if is_offboard_active:
                                try: await drone_system.offboard.stop()
                                except: pass
                                is_offboard_active = False
                            if mode == 'RTL': await drone_system.action.return_to_launch()
                            elif mode == 'TAKEOFF': await drone_system.action.takeoff()
                            elif mode == 'LAND': await drone_system.action.land()
                            elif mode == 'HOLD': await drone_system.action.hold()     
                    except Exception as e:
                        print(f"Mode Error: {e}")

                elif msg_type == 'MANUAL_CONTROL':
                    if is_offboard_active:
                        try:
                            await drone_system.offboard.set_velocity_body(
                                VelocityBodyYawspeed(float(data['x']), float(data['y']), float(data['z']), float(data['r']))
                            )
                        except: pass
                
                # --- MISSION UPLOADS ---
                elif msg_type == 'UPLOAD_MISSION_FIGURE8':
                    print("CMD: Upload Figure 8...")
                    plan = await generate_figure8_mission()
                    if plan:
                        await drone_system.mission.clear_mission()
                        await drone_system.mission.upload_mission(plan)
                        print(">>> Uploaded Figure 8")

                elif msg_type == 'UPLOAD_MISSION_SQUARE':
                    print("CMD: Upload Square...")
                    plan = await generate_square_mission()
                    if plan:
                        await drone_system.mission.clear_mission()
                        await drone_system.mission.upload_mission(plan)
                        print(">>> Uploaded Square")

                elif msg_type == 'UPLOAD_MISSION_SCAN':
                    print("CMD: Upload Grid Scan...")
                    plan = await generate_scan_mission()
                    if plan:
                        await drone_system.mission.clear_mission()
                        await drone_system.mission.upload_mission(plan)
                        print(">>> Uploaded Grid Scan")

                elif msg_type == 'UPLOAD_MISSION_SPIRAL':
                    print("CMD: Upload Spiral...")
                    plan = await generate_spiral_mission()
                    if plan:
                        await drone_system.mission.clear_mission()
                        await drone_system.mission.upload_mission(plan)
                        print(">>> Uploaded Spiral")
            
            except Exception as e:
                print(f"WS Msg Error: {e}")

    except websockets.exceptions.ConnectionClosed: pass
    finally: connected_clients.remove(websocket)

async def main():
    global drone_system
    drone_system = System()
    print(f"--- SKYNET GCS BACKEND ---")
    print(f"Waiting for drone at: {CONNECTION_STRING}...")
    await drone_system.connect(system_address=CONNECTION_STRING)
    print("Waiting for Heartbeat...")
    async for state in drone_system.core.connection_state():
        if state.is_connected:
            print(f"*** DRONE CONNECTED! ***")
            current_telemetry["connected"] = True
            break
    server = await websockets.serve(websocket_handler, "localhost", WS_PORT)
    print(f"WebSocket listening on ws://localhost:{WS_PORT}")
    await asyncio.gather(telemetry_loop(), broadcast_loop(), monitor_mission_progress(), server.wait_closed())

if __name__ == "__main__":
    try: asyncio.run(main())
    except KeyboardInterrupt: print("\nServer Stopped.")