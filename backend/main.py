import asyncio
import json
import math
import websockets
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)
from mavsdk.mission import (MissionItem, MissionPlan)

# --- KONFIGURASI KONEKSI ---
# SITL (Simulator)
CONNECTION_STRING = "udp://:14550" 
# Hardware (Drone Asli - Uncomment)
# CONNECTION_STRING = "serial://COM3:57600"

WS_PORT = 8080

# State Global
current_telemetry = {
    "connected": False,
    "armed": False,
    "mode": "UNKNOWN",
    "battery_voltage": 0,
    "battery_remaining": 0,
    "latitude": 0,
    "longitude": 0,
    "altitude_relative": 0,
    "heading": 0,
    "pitch": 0,
    "roll": 0,
    "satellites": 0,
    "ground_speed": 0, 
    "climb_rate": 0,
    "health_gyro": False,
    "health_accel": False,
    "health_mag": False,
    "health_gps": False
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

async def generate_figure8_mission(altitude=15):
    print("Generating Lemniscate Figure 8 Mission...")
    mission_items = []
    
    center_lat = current_telemetry["latitude"]
    center_lon = current_telemetry["longitude"]
    
    if abs(center_lat) < 0.001 and abs(center_lon) < 0.001:
        print("Error: GPS invalid. Cannot generate mission.")
        return None

    # Config Angka 8
    size_factor = 30  # Ukuran
    loops = 2         # [FIX] Cukup 1 kali putaran
    steps = 30        # Titik per putaran
    speed_ms = 8.0 

    # 1. Waypoint Awal: Naik ke ketinggian misi di posisi sekarang
    mission_items.append(MissionItem(
        latitude_deg=center_lat,
        longitude_deg=center_lon,
        relative_altitude_m=altitude,
        speed_m_s=5.0,
        is_fly_through=True,
        gimbal_pitch_deg=float('nan'), gimbal_yaw_deg=float('nan'),
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=float('nan'), camera_photo_interval_s=float('nan'),
        acceptance_radius_m=1.0, yaw_deg=float('nan'), camera_photo_distance_m=float('nan'),
        vehicle_action=MissionItem.VehicleAction.NONE 
    ))

    # 2. Pola Lemniscate (Angka 8)
    for k in range(loops):
        for i in range(steps):
            # t dari 0 sampai 2*PI
            t = (2 * math.pi * i) / steps
            
            # Rumus Lemniscate (Vertikal 8)
            # x (East) = a * cos(t) / (1 + sin²(t))
            # y (North) = a * sin(t) * cos(t) / (1 + sin²(t))
            denom = 1 + math.sin(t)**2
            d_east = (size_factor * math.cos(t)) / denom
            d_north = (size_factor * math.sin(t) * math.cos(t)) / denom
            
            # Rotasi 90 derajat jika ingin Horizontal, tapi default ini sudah cantik
            
            wp_lat, wp_lon = get_offset_location(center_lat, center_lon, d_north, d_east)
            
            mission_items.append(MissionItem(
                latitude_deg=wp_lat,
                longitude_deg=wp_lon,
                relative_altitude_m=altitude,
                speed_m_s=speed_ms,
                is_fly_through=True,
                gimbal_pitch_deg=float('nan'), gimbal_yaw_deg=float('nan'),
                camera_action=MissionItem.CameraAction.NONE, 
                loiter_time_s=float('nan'), camera_photo_interval_s=float('nan'), 
                acceptance_radius_m=2.0, # Radius toleransi
                yaw_deg=float('nan'), camera_photo_distance_m=float('nan'),
                vehicle_action=MissionItem.VehicleAction.NONE
            ))

    # 3. Waypoint Akhir: Kembali ke Center (Tanpa Land Item)
    # Kita akan handle landing via Logic Monitor agar pasti landing
    mission_items.append(MissionItem(
        latitude_deg=center_lat,
        longitude_deg=center_lon,
        relative_altitude_m=altitude,
        speed_m_s=5.0,
        is_fly_through=False,
        gimbal_pitch_deg=float('nan'), gimbal_yaw_deg=float('nan'),
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=float('nan'), camera_photo_interval_s=float('nan'),
        acceptance_radius_m=1.0, yaw_deg=float('nan'), camera_photo_distance_m=float('nan'),
        vehicle_action=MissionItem.VehicleAction.NONE 
    ))

    print(f"Mission Generated: {len(mission_items)} items")
    return MissionPlan(mission_items)

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

    try:
        await asyncio.gather(
            get_position(), get_velocity(), get_attitude(), get_battery(),
            get_flight_mode(), get_armed(), get_gps_info(), get_health()
        )
    except Exception as e:
        print(f"Telemetry Loop Error: {e}")

async def broadcast_loop():
    while True:
        if connected_clients:
            try:
                data = current_telemetry.copy()
                for k in ["health_gyro", "health_accel", "health_mag", "health_gps"]:
                    data.pop(k, None)
                message = json.dumps(data)
                await asyncio.gather(*[client.send(message) for client in connected_clients])
            except Exception as e:
                print(f"Broadcast Error: {e}")
        await asyncio.sleep(0.1)

# [FIX] MONITOR PROGRESS UNTUK AUTO LAND
async def monitor_mission_progress():
    print("Monitoring Mission Progress...")
    async for progress in drone_system.mission.mission_progress():
        # print(f"Mission: {progress.current}/{progress.total}")
        
        # Jika misi selesai (current == total) DAN kita masih di udara
        if progress.total > 0 and progress.current >= progress.total:
            # Cek apakah sedang terbang (altitude > 1 meter)
            if current_telemetry["altitude_relative"] > 1.0:
                print(">>> MISSION COMPLETED. SWITCHING TO LAND MODE... <<<")
                try:
                    await drone_system.action.land()
                except Exception as e:
                    print(f"Auto Land Failed: {e}")
                
                # Beri jeda agar tidak spam command Land
                await asyncio.sleep(5)

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
    return report

async def websocket_handler(websocket):
    global is_offboard_active
    connected_clients.add(websocket)
    print(">>> React Client Connected")
    
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

                            print("- Starting Mission...")
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
                
                elif msg_type == 'UPLOAD_MISSION_FIGURE8':
                    print("CMD: Uploading Mission...")
                    plan = await generate_figure8_mission()
                    if plan:
                        try:
                            await drone_system.mission.clear_mission()
                            await drone_system.mission.upload_mission(plan)
                            print(">>> Mission Uploaded Successfully!")
                        except Exception as e:
                            print(f"Upload Failed: {e}")
            
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

    # [FIX] Tambahkan monitor_mission_progress ke loop utama
    await asyncio.gather(
        telemetry_loop(), 
        broadcast_loop(), 
        monitor_mission_progress(), # <--- New Task
        server.wait_closed()
    )

if __name__ == "__main__":
    try: asyncio.run(main())
    except KeyboardInterrupt: print("\nServer Stopped.")