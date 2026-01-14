import asyncio
import json
import math
import threading
import queue
import sys
import os

import websockets
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)
from mavsdk.mission import (MissionItem, MissionPlan)
import sounddevice as sd
from vosk import Model, KaldiRecognizer

# --- KONFIGURASI KONEKSI ---
# SITL
CONNECTION_STRING = "udp://:14550" 
# Hardware (Radio)
# CONNECTION_STRING = "serial://COM3:57600"

WS_PORT = 8080

if not os.path.exists("model"):
    print("WARNING: Folder 'model' tidak ditemukan. Voice command tidak akan jalan.")
    print("Download dari: https://alphacephei.com/vosk/models")
    voice_model = None
else:
    print("Loading Offline Voice Model...")
    voice_model = Model("model")

voice_cmd_queue = asyncio.Queue()
is_voice_listening = False

current_telemetry = {
    "connected": False, "armed": False, "mode": "UNKNOWN",
    "battery_voltage": 0, "battery_remaining": 0,
    "latitude": 0, "longitude": 0, "altitude_relative": 0,
    "heading": 0, "pitch": 0, "roll": 0,
    "satellites": 0, "ground_speed": 0, 
    "health_gyro": False, "health_gps": False
}

connected_clients = set()
drone_system = None
is_offboard_active = False

def voice_worker(loop):
    global is_voice_listening
    
    if not voice_model:
        return

    q = queue.Queue()

    def callback(indata, frames, time, status):
        if status:
            print(status, file=sys.stderr)
        q.put(bytes(indata))

    with sd.RawInputStream(samplerate=16000, blocksize=8000, device=None, dtype='int16',
                           channels=1, callback=callback):
        rec = KaldiRecognizer(voice_model, 16000)
        
        while True:
            if not is_voice_listening:
                sd.sleep(100)
                continue
            
            data = q.get()
            if rec.AcceptWaveform(data):
                res = json.loads(rec.Result())
                text = res['text']
                if text:
                    print(f"[VOICE HEARD]: {text}")
                    asyncio.run_coroutine_threadsafe(voice_cmd_queue.put(text), loop)

def get_offset_location(lat, lon, d_north, d_east):
    earth_radius = 6378137.0 
    d_lat = d_north / earth_radius
    d_lon = d_east / (earth_radius * math.cos(math.pi * lat / 180))
    new_lat = lat + (d_lat * 180 / math.pi)
    new_lon = lon + (d_lon * 180 / math.pi)
    return new_lat, new_lon

async def generate_figure8_mission(altitude=15):
    mission_items = []
    center_lat = current_telemetry["latitude"]
    center_lon = current_telemetry["longitude"]
    
    if abs(center_lat) < 0.001: return None

    radius = 30; loops = 1; steps = 24
    
    mission_items.append(MissionItem(
        center_lat, center_lon, altitude, 5.0, True, float('nan'), float('nan'),
        MissionItem.CameraAction.NONE, float('nan'), float('nan'), 1.0, float('nan'), float('nan'),
        MissionItem.VehicleAction.NONE
    ))

    for k in range(loops):
        for i in range(steps):
            t = (2 * math.pi * i) / steps
            denom = 1 + math.sin(t)**2
            d_east = (radius * math.cos(t)) / denom
            d_north = (radius * math.sin(t) * math.cos(t)) / denom
            lat, lon = get_offset_location(center_lat, center_lon, d_north, d_east)
            mission_items.append(MissionItem(
                lat, lon, altitude, 8.0, True, float('nan'), float('nan'),
                MissionItem.CameraAction.NONE, float('nan'), float('nan'), 2.0, float('nan'), float('nan'),
                MissionItem.VehicleAction.NONE
            ))

    mission_items.append(MissionItem(
        center_lat, center_lon, 0, 5.0, False, float('nan'), float('nan'),
        MissionItem.CameraAction.NONE, float('nan'), float('nan'), 1.0, float('nan'), float('nan'),
        MissionItem.VehicleAction.LAND
    ))
    return MissionPlan(mission_items)

async def telemetry_loop():
    global drone_system
    async def get_pos():
        async for p in drone_system.telemetry.position():
            current_telemetry["latitude"] = p.latitude_deg
            current_telemetry["longitude"] = p.longitude_deg
            current_telemetry["altitude_relative"] = p.relative_altitude_m
    async def get_meta():
        async for m in drone_system.telemetry.flight_mode(): current_telemetry["mode"] = str(m)
        async for a in drone_system.telemetry.armed(): current_telemetry["armed"] = a
        async for b in drone_system.telemetry.battery(): 
            current_telemetry["battery_voltage"] = b.voltage_v
            current_telemetry["battery_remaining"] = b.remaining_percent * 100
        async for g in drone_system.telemetry.gps_info(): current_telemetry["satellites"] = g.num_satellites
        async for h in drone_system.telemetry.health():
            current_telemetry["health_gyro"] = h.is_gyrometer_calibration_ok
            current_telemetry["health_gps"] = h.is_global_position_ok
        async for v in drone_system.telemetry.velocity_ned():
            current_telemetry["ground_speed"] = math.sqrt(v.north_m_s**2 + v.east_m_s**2)

    try:
        await asyncio.gather(get_pos(), get_meta())
    except: pass

async def broadcast_loop():
    while True:
        if connected_clients:
            try:
                data = current_telemetry.copy()
                data.pop("health_gyro", None); data.pop("health_gps", None)
                await asyncio.gather(*[client.send(json.dumps(data)) for client in connected_clients])
            except: pass
        await asyncio.sleep(0.05)

async def voice_command_processor():
    global is_offboard_active
    while True:
        text = await voice_cmd_queue.get() # Tunggu teks dari thread voice
        print(f"Executing Voice Command: {text}")
        
        if "arm" in text and "disarm" not in text:
            print("VOICE: ARMING")
            try: await drone_system.action.arm()
            except: pass
        
        elif "disarm" in text:
            print("VOICE: DISARMING")
            try: await drone_system.action.disarm()
            except: pass
            
        elif "take off" in text or "takeoff" in text:
            print("VOICE: TAKEOFF")
            try: await drone_system.action.takeoff()
            except: pass
            
        elif "land" in text:
            print("VOICE: LANDING")
            try: await drone_system.action.land()
            except: pass
            
        elif "mission" in text or "start" in text:
            print("VOICE: START MISSION")
            if is_offboard_active:
                try: await drone_system.offboard.stop()
                except: pass
                is_offboard_active = False
            try: await drone_system.mission.start_mission()
            except: pass

async def websocket_handler(websocket):
    global is_offboard_active, is_voice_listening
    connected_clients.add(websocket)
    
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                msg_type = data.get('type')

                if msg_type == 'TOGGLE_VOICE_BACKEND':
                    state = data.get('state', False)
                    is_voice_listening = state
                    print(f"Voice Listening: {state}")

                elif msg_type == 'COMMAND_LONG':
                    if data['param1'] == 1: await drone_system.action.arm()
                    else: await drone_system.action.disarm()

                elif msg_type == 'SET_MODE':
                    mode = data['mode']
                    if mode == 'MISSION':
                        if is_offboard_active:
                            try: await drone_system.offboard.stop()
                            except: pass
                            is_offboard_active = False
                        if not current_telemetry["armed"]: await drone_system.action.arm()
                        await drone_system.mission.start_mission()
                    elif mode == 'OFFBOARD':
                        await drone_system.offboard.set_velocity_body(VelocityBodyYawspeed(0.0,0.0,0.0,0.0))
                        await drone_system.offboard.start()
                        is_offboard_active = True
                    elif mode == 'TAKEOFF': await drone_system.action.takeoff()
                    elif mode == 'LAND': await drone_system.action.land()
                    elif mode == 'RTL': await drone_system.action.return_to_launch()
                    elif mode == 'HOLD': await drone_system.action.hold()

                elif msg_type == 'MANUAL_CONTROL' and is_offboard_active:
                    await drone_system.offboard.set_velocity_body(
                        VelocityBodyYawspeed(float(data['x']), float(data['y']), float(data['z']), float(data['r']))
                    )

                elif msg_type == 'UPLOAD_MISSION_FIGURE8':
                    plan = await generate_figure8_mission()
                    if plan:
                        await drone_system.mission.clear_mission()
                        await drone_system.mission.upload_mission(plan)
                        print("Mission Uploaded")

                elif msg_type == 'REQ_PREFLIGHT':
                    report = [
                        {"id": "gps", "label": f"GPS ({current_telemetry['satellites']})", "status": "PASS" if current_telemetry["health_gps"] else "WARN", "detail": "Ready"},
                        {"id": "bat", "label": "Battery", "status": "PASS" if current_telemetry["battery_remaining"]>20 else "WARN", "detail": f"{int(current_telemetry['battery_remaining'])}%"}
                    ]
                    await websocket.send(json.dumps({"type": "PREFLIGHT_REPORT", "report": report}))

            except Exception as e: print(f"Msg Error: {e}")
    except: pass
    finally: connected_clients.remove(websocket)

async def main():
    global drone_system
    drone_system = System()
    print(f"Waiting drone at {CONNECTION_STRING}")
    await drone_system.connect(system_address=CONNECTION_STRING)
    
    print("Waiting Heartbeat...")
    async for state in drone_system.core.connection_state():
        if state.is_connected:
            print(">>> DRONE CONNECTED")
            current_telemetry["connected"] = True
            break
            
    loop = asyncio.get_running_loop()
    t = threading.Thread(target=voice_worker, args=(loop,), daemon=True)
    t.start()
    print("Offline Voice Thread Started (Ready)")

    server = await websockets.serve(websocket_handler, "localhost", WS_PORT)
    print(f"WS Server: ws://localhost:{WS_PORT}")
    
    await asyncio.gather(
        telemetry_loop(), 
        broadcast_loop(), 
        voice_command_processor(),
        server.wait_closed()
    )

if __name__ == "__main__":
    try: asyncio.run(main())
    except KeyboardInterrupt: pass