import React, { useState, useEffect, useRef } from 'react';
import { 
  Activity, BatteryCharging, Compass, Power, Send, Signal, Wifi, 
  AlertTriangle, ArrowUp, Gamepad2, Anchor, LayoutDashboard, 
  Map as MapIcon, Video, ClipboardCheck, Home, ArrowDown, Play, 
  CheckCircle, XCircle, RefreshCw, UploadCloud, Camera, Settings
} from 'lucide-react';

// --- STYLES ---
const styles = `
  ::-webkit-scrollbar { width: 6px; }
  ::-webkit-scrollbar-track { background: #111827; }
  ::-webkit-scrollbar-thumb { background: #374151; border-radius: 3px; }
  ::-webkit-scrollbar-thumb:hover { background: #4b5563; }

  .horizon-container { overflow: hidden; position: relative; border-radius: 50%; border: 4px solid #374151; }
  .horizon-sky { background: #3b82f6; width: 100%; height: 200%; position: absolute; top: -50%; transition: transform 0.1s linear; }
  .horizon-ground { background: #854d0e; width: 100%; height: 50%; position: absolute; bottom: 0; }
  .hud-overlay { position: absolute; top: 0; left: 0; right: 0; bottom: 0; z-index: 20; display: flex; align-items: center; justify-content: center; pointer-events: none; }

  #map-container { width: 100%; height: 100%; z-index: 0; background: #111827; }
  .leaflet-pane { z-index: 1 !important; }
  .leaflet-bottom { z-index: 10 !important; }
`;

// --- TYPES ---
interface TelemetryData {
  connected: boolean; armed: boolean; mode: string;
  battery_voltage: number; battery_remaining: number;
  latitude: number; longitude: number;
  altitude_relative: number; heading: number; pitch: number; roll: number;
  satellites: number; ground_speed: number; climb_rate: number;
}

interface LogMessage { id: number; timestamp: string; type: string; message: string; }
interface PreflightItem { id: string; label: string; status: 'PENDING' | 'PASS' | 'FAIL' | 'WARN'; detail: string; }

// --- COMPONENT: LIVE VIDEO FEED (NEW) ---
const LiveVideoFeed = ({ telemetry }: { telemetry: TelemetryData }) => {
    const videoRef = useRef<HTMLVideoElement>(null);
    const [devices, setDevices] = useState<MediaDeviceInfo[]>([]);
    const [selectedDeviceId, setSelectedDeviceId] = useState<string>('');
    const [error, setError] = useState<string>('');

    // 1. Get List of Cameras (Video Inputs)
    useEffect(() => {
        const getCameras = async () => {
            try {
                // Minta izin akses kamera
                await navigator.mediaDevices.getUserMedia({ video: true });
                const allDevices = await navigator.mediaDevices.enumerateDevices();
                const videoDevices = allDevices.filter(device => device.kind === 'videoinput');
                setDevices(videoDevices);
                if (videoDevices.length > 0) {
                    setSelectedDeviceId(videoDevices[0].deviceId);
                }
            } catch (err) {
                setError("Camera permission denied or no device found.");
            }
        };
        getCameras();
    }, []);

    // 2. Stream Selected Camera
    useEffect(() => {
        if (!selectedDeviceId) return;
        
        const startStream = async () => {
            try {
                const stream = await navigator.mediaDevices.getUserMedia({
                    video: { deviceId: { exact: selectedDeviceId } }
                });
                if (videoRef.current) {
                    videoRef.current.srcObject = stream;
                }
            } catch (err) {
                setError("Failed to access camera stream.");
            }
        };
        startStream();

        // Cleanup function
        return () => {
            if (videoRef.current && videoRef.current.srcObject) {
                const tracks = (videoRef.current.srcObject as MediaStream).getTracks();
                tracks.forEach(track => track.stop());
            }
        };
    }, [selectedDeviceId]);

    return (
        <div className="w-full h-full relative bg-black flex flex-col items-center justify-center overflow-hidden">
            {/* Video Element */}
            <video 
                ref={videoRef} 
                autoPlay 
                playsInline 
                muted 
                className="w-full h-full object-cover"
            />

            {/* Error Message */}
            {error && (
                <div className="absolute inset-0 flex items-center justify-center bg-black/80 z-20">
                    <div className="text-red-500 font-mono text-center">
                        <Video size={48} className="mx-auto mb-2"/>
                        <p>{error}</p>
                        <p className="text-xs text-gray-400 mt-2">Connect USB Capture Card & Allow Permission</p>
                    </div>
                </div>
            )}

            {/* Device Selector (Top Right) */}
            <div className="absolute top-4 right-4 z-30">
                <div className="bg-black/50 backdrop-blur rounded-lg p-1 flex items-center border border-gray-600">
                    <Settings size={14} className="text-gray-400 ml-2"/>
                    <select 
                        value={selectedDeviceId}
                        onChange={(e) => setSelectedDeviceId(e.target.value)}
                        className="bg-transparent text-xs text-white p-1 outline-none cursor-pointer max-w-[150px]"
                    >
                        {devices.map(device => (
                            <option key={device.deviceId} value={device.deviceId}>
                                {device.label || `Camera ${device.deviceId.slice(0,5)}...`}
                            </option>
                        ))}
                    </select>
                </div>
            </div>

            {/* OSD (On Screen Display) Overlay */}
            <div className="absolute inset-4 border-2 border-white/20 rounded-lg pointer-events-none flex flex-col justify-between p-4 z-20">
                <div className="flex justify-between text-green-400 font-mono text-sm shadow-black drop-shadow-md bg-black/20 p-1 rounded">
                    <span className="flex items-center gap-2"><div className="w-2 h-2 bg-red-600 rounded-full animate-pulse"></div> LIVE</span>
                    <span>BAT: {telemetry.battery_voltage.toFixed(1)}V ({Math.round(telemetry.battery_remaining)}%)</span>
                </div>
                <div className="flex justify-center text-white/50">
                    <div className="w-8 h-8 border border-white/50 rounded-full flex items-center justify-center">+</div>
                </div>
                <div className="flex justify-between text-green-400 font-mono text-sm shadow-black drop-shadow-md bg-black/20 p-1 rounded">
                    <span>ALT: {telemetry.altitude_relative.toFixed(1)}m</span>
                    <span>SPD: {telemetry.ground_speed.toFixed(1)}m/s</span>
                </div>
            </div>
        </div>
    );
};

// --- SUB-COMPONENTS ---
const FlightModeButton = ({ mode, icon: Icon, label, currentMode, onClick }: any) => {
    const isActive = currentMode.includes(mode);
    return (
      <button onClick={() => onClick(mode)} title={label} className={`w-10 h-10 rounded-full flex items-center justify-center transition-all border-2 ${isActive ? 'bg-blue-600 border-blue-400 text-white shadow-[0_0_10px_rgba(59,130,246,0.5)]' : 'bg-gray-700 border-gray-600 text-gray-400 hover:bg-gray-600 hover:text-white'}`}><Icon size={18} /></button>
    )
};

const ArtificialHorizon = ({ pitch, roll }: { pitch: number, roll: number }) => {
  const pitchOffset = pitch * 2; 
  return (
    <div className="w-48 h-48 mx-auto horizon-container bg-blue-500 shadow-lg mb-4">
      <div className="w-full h-full relative" style={{ transform: `rotate(${-roll}deg)` }}>
          <div className="horizon-sky" style={{ transform: `translateY(${pitchOffset}px)` }}> <div className="horizon-ground"></div> </div>
          <div className="absolute top-0 left-0 w-full h-full flex flex-col items-center justify-center opacity-50 text-white text-xs font-mono">
              <div className="border-b border-white w-12 mb-2">10</div> <div className="border-b border-white w-20 mb-2"></div> <div className="border-b border-white w-12">10</div>
          </div>
      </div>
      <div className="hud-overlay"> <div className="w-16 h-1 bg-yellow-400 opacity-80" style={{clipPath: 'polygon(0 0, 40% 0, 50% 100%, 60% 0, 100% 0, 100% 100%, 0 100%)', height: '4px'}}></div> </div>
    </div>
  );
};

// --- MAIN COMPONENT ---
const App = () => {
  const [activeTab, setActiveTab] = useState<'dashboard' | 'missions' | 'preflight'>('dashboard');
  const [viewMode, setViewMode] = useState<'map' | 'camera'>('map');
  const [wsStatus, setWsStatus] = useState<string>('DISCONNECTED');

  const [telemetry, setTelemetry] = useState<TelemetryData>({
    connected: false, armed: false, mode: 'DISARMED',
    battery_voltage: 0, battery_remaining: 0, latitude: 0, longitude: 0, 
    altitude_relative: 0, heading: 0, pitch: 0, roll: 0,
    satellites: 0, ground_speed: 0, climb_rate: 0
  });

  const [preflightData, setPreflightData] = useState<PreflightItem[]>([
      { id: 'imu', label: 'IMU Sensors (Gyro/Accel)', status: 'PENDING', detail: 'Waiting for check...' },
      { id: 'mag', label: 'Magnetometer / Compass', status: 'PENDING', detail: 'Waiting for check...' },
      { id: 'gps', label: 'GPS Lock (> 6 Sats)', status: 'PENDING', detail: 'Waiting for check...' },
      { id: 'bat', label: 'Battery Level', status: 'PENDING', detail: 'Waiting for check...' },
      { id: 'home', label: 'Global Position Estimate', status: 'PENDING', detail: 'Waiting for check...' }
  ]);

  const [logs, setLogs] = useState<LogMessage[]>([]);
  const logsEndRef = useRef<HTMLDivElement>(null);
  
  const mapRef = useRef<any>(null);
  const markerRef = useRef<any>(null);
  const polylineRef = useRef<any>(null);
  const pathDataRef = useRef<[number, number][]>([]);
  const wsRef = useRef<WebSocket | null>(null);
  const hasLockedRef = useRef<boolean>(false);
  const currentModeRef = useRef<string>("DISARMED");

  const addLog = (type: string, message: string) => {
    const now = new Date();
    setLogs(prev => [...prev.slice(-49), { id: Date.now(), timestamp: `${now.getHours()}:${now.getMinutes()}:${now.getSeconds()}`, type, message }]);
  };

  useEffect(() => { if (activeTab === 'dashboard') logsEndRef.current?.scrollIntoView({ behavior: "smooth" }); }, [logs, activeTab]);

  // --- WEBSOCKET ---
  useEffect(() => {
    const wsUrl = "ws://localhost:8080/telemetry";
    setWsStatus('CONNECTING');
    const ws = new WebSocket(wsUrl);
    ws.onopen = () => { setWsStatus('CONNECTED'); setTelemetry(t => ({ ...t, connected: true })); addLog('SYS', 'Connected to Backend'); };
    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            if (data.type === 'PREFLIGHT_REPORT') { setPreflightData(data.report); addLog('INFO', 'Pre-flight check completed.'); return; }
            if(data.mode) currentModeRef.current = data.mode;
            setTelemetry(prev => ({ ...prev, ...data }));
        } catch (e) {}
    };
    ws.onclose = () => { setWsStatus('DISCONNECTED'); setTelemetry(t => ({ ...t, connected: false })); addLog('ERR', 'Disconnected'); };
    wsRef.current = ws;
    return () => ws.close();
  }, []);

  // --- COMMANDS ---
  const sendCommand = (payload: any) => { if(wsRef.current && wsRef.current.readyState === WebSocket.OPEN) wsRef.current.send(JSON.stringify(payload)); };
  const handleArm = () => { if (!telemetry.armed && telemetry.mode.includes("STABILIZE")) addLog('WARN', 'Mode is STABILIZE. Switch to HOLD/LOITER first!'); sendCommand({ type: 'COMMAND_LONG', command: 'MAV_CMD_COMPONENT_ARM_DISARM', param1: telemetry.armed ? 0 : 1 }); };
  const handleModeChange = (newMode: string) => sendCommand({ type: 'SET_MODE', mode: newMode });
  const runPreflightCheck = () => { addLog('CMD', 'Running Diagnostics...'); setPreflightData(prev => prev.map(p => ({...p, status: 'PENDING', detail: 'Checking...'}))); sendCommand({ type: 'REQ_PREFLIGHT' }); };
  const uploadFigure8 = () => { addLog('CMD', 'Uploading Mission...'); sendCommand({ type: 'UPLOAD_MISSION_FIGURE8' }); };
  const startMission = () => { addLog('CMD', 'Sending START Command...'); sendCommand({ type: 'SET_MODE', mode: 'MISSION' }); };

  // --- MAP ---
  useEffect(() => {
    if (activeTab !== 'dashboard' || viewMode !== 'map') return;
    if (!document.getElementById('leaflet-css')) { const link = document.createElement("link"); link.id = 'leaflet-css'; link.rel = "stylesheet"; link.href = "https://unpkg.com/leaflet@1.7.1/dist/leaflet.css"; document.head.appendChild(link); }
    const styleSheet = document.createElement("style"); styleSheet.innerText = styles; document.head.appendChild(styleSheet);
    if (!document.getElementById('leaflet-js')) { const script = document.createElement("script"); script.id = 'leaflet-js'; script.src = "https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"; script.async = true; script.onload = () => initMapInstance(); document.head.appendChild(script); } else { setTimeout(initMapInstance, 100); }
    return () => { if (mapRef.current) { mapRef.current.remove(); mapRef.current = null; } };
  }, [activeTab, viewMode]);

  const initMapInstance = () => {
    const L = (window as any).L;
    if (!L || mapRef.current || !document.getElementById('map-container')) return;
    const startLat = telemetry.latitude !== 0 ? telemetry.latitude : -6.2088;
    const startLng = telemetry.longitude !== 0 ? telemetry.longitude : 106.8456;
    const map = L.map('map-container', { zoomControl: false, attributionControl: false }).setView([startLat, startLng], 16);
    L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', { maxZoom: 19 }).addTo(map);
    const droneIcon = L.divIcon({ html: `<div style="width: 32px; height: 32px; transform: translate(-50%, -50%);"><svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="#ef4444" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" style="filter: drop-shadow(0 0 4px #000);"><path d="M12 2L2 7l10 5 10-5-10-5zM2 17l10 5 10-5M2 12l10 5 10-5"/></svg></div>`, className: 'custom-drone-icon', iconSize: [32, 32], iconAnchor: [16, 16] });
    const marker = L.marker([startLat, startLng], { icon: droneIcon }); marker.addTo(map);
    const polyline = L.polyline(pathDataRef.current, { color: 'red', weight: 2, opacity: 0.6 }).addTo(map);
    mapRef.current = map; markerRef.current = marker; polylineRef.current = polyline; hasLockedRef.current = false; 
  };

  useEffect(() => {
    const L = (window as any).L;
    if (!L || !mapRef.current || !markerRef.current) return;
    if (Math.abs(telemetry.latitude) < 0.001 && Math.abs(telemetry.longitude) < 0.001) return;
    const newLatLng = [telemetry.latitude, telemetry.longitude] as [number, number];
    markerRef.current.setLatLng(newLatLng);
    if (telemetry.connected) { if (!hasLockedRef.current) { mapRef.current.setView(newLatLng, 16); hasLockedRef.current = true; } else { mapRef.current.panTo(newLatLng, { animate: true, duration: 0.1 }); } }
    if (telemetry.armed) { pathDataRef.current.push(newLatLng); if (polylineRef.current) polylineRef.current.setLatLngs(pathDataRef.current); }
  }, [telemetry.latitude, telemetry.longitude, telemetry.connected]); 

  // --- KEYBOARD ---
  useEffect(() => {
    const SPEED = 5.0, YAW_SPEED = 40.0, CLIMB_SPEED = 2.0; 
    const handleKeyDown = (e: KeyboardEvent) => {
      if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) return;
      if (!currentModeRef.current.includes("OFFBOARD")) return;
      if(["ArrowUp","ArrowDown","ArrowLeft","ArrowRight","Space"," "].indexOf(e.key) > -1 || e.code === "Space") e.preventDefault();
      let x = 0, y = 0, z = 0, r = 0;
      if (e.key === 'w' || e.key === 'W') x = SPEED; if (e.key === 's' || e.key === 'S') x = -SPEED;   
      if (e.key === 'a' || e.key === 'A') y = -SPEED; if (e.key === 'd' || e.key === 'D') y = SPEED;    
      if (e.key === 'ArrowLeft') r = -YAW_SPEED; if (e.key === 'ArrowRight') r = YAW_SPEED;
      if (e.key === 'ArrowUp') z = -CLIMB_SPEED; if (e.key === 'ArrowDown') z = CLIMB_SPEED;
      if (e.code === 'Space') z = -CLIMB_SPEED;
      if (x !== 0 || y !== 0 || z !== 0 || r !== 0) wsRef.current.send(JSON.stringify({ type: 'MANUAL_CONTROL', x, y, z, r }));
    };
    const handleKeyUp = () => { if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN && currentModeRef.current.includes("OFFBOARD")) wsRef.current.send(JSON.stringify({ type: 'MANUAL_CONTROL', x: 0, y: 0, z: 0, r: 0 })); };
    window.addEventListener('keydown', handleKeyDown); window.addEventListener('keyup', handleKeyUp);
    return () => { window.removeEventListener('keydown', handleKeyDown); window.removeEventListener('keyup', handleKeyUp); };
  }, []);

  return (
    <div className="flex h-screen bg-gray-900 text-gray-100 font-sans overflow-hidden">
      <nav className="w-20 bg-gray-800 border-r border-gray-700 flex flex-col items-center py-6 gap-6 z-30 shadow-xl">
         <div className="bg-blue-600 p-3 rounded-xl mb-4 shadow-lg shadow-blue-900/50"><Send size={24} className="text-white" /></div>
         <button onClick={() => setActiveTab('dashboard')} className={`p-3 rounded-xl transition-all ${activeTab === 'dashboard' ? 'bg-gray-700 text-blue-400 border border-gray-600' : 'text-gray-400 hover:text-white hover:bg-gray-700/50'}`} title="Dashboard"><LayoutDashboard size={24} /></button>
         <button onClick={() => setActiveTab('missions')} className={`p-3 rounded-xl transition-all ${activeTab === 'missions' ? 'bg-gray-700 text-blue-400 border border-gray-600' : 'text-gray-400 hover:text-white hover:bg-gray-700/50'}`} title="Mission Planner"><MapIcon size={24} /></button>
         <button onClick={() => setActiveTab('preflight')} className={`p-3 rounded-xl transition-all ${activeTab === 'preflight' ? 'bg-gray-700 text-blue-400 border border-gray-600' : 'text-gray-400 hover:text-white hover:bg-gray-700/50'}`} title="Pre-Flight Check"><ClipboardCheck size={24} /></button>
         <div className="flex-1"></div>
         <div className="text-[10px] text-gray-500 font-mono rotate-180" style={{writingMode: 'vertical-rl'}}>CAKSA GCS v2.5</div>
      </nav>

      <div className="flex-1 flex flex-col min-w-0">
          <header className="h-20 bg-gray-800 border-b border-gray-700 flex items-center justify-between px-6 shadow-md z-20">
             <div className="flex flex-col">
                <h1 className="font-bold text-xl tracking-wider text-white">CAKSA <span className="text-blue-400">GCS</span></h1>
                <div className="flex items-center gap-2 mt-1">
                    <span className={`w-2 h-2 rounded-full ${telemetry.connected ? 'bg-green-500' : 'bg-red-500'}`}></span>
                    <span className="text-xs text-gray-400 font-mono">{telemetry.connected ? 'LINK ACTIVE' : 'NO LINK'}</span>
                </div>
             </div>
             <div className="flex items-center gap-4 bg-gray-900/50 px-6 py-2 rounded-full border border-gray-700">
                <span className="text-xs text-gray-500 font-bold mr-2">MODES</span>
                <FlightModeButton mode="HOLD" icon={Anchor} label="LOITER / HOLD" currentMode={telemetry.mode} onClick={handleModeChange} />
                <FlightModeButton mode="TAKEOFF" icon={ArrowUp} label="TAKEOFF" currentMode={telemetry.mode} onClick={handleModeChange} />
                <FlightModeButton mode="LAND" icon={ArrowDown} label="LAND" currentMode={telemetry.mode} onClick={handleModeChange} />
                <FlightModeButton mode="RTL" icon={Home} label="RETURN TO LAUNCH" currentMode={telemetry.mode} onClick={handleModeChange} />
                <div className="w-px h-8 bg-gray-700 mx-2"></div>
                <FlightModeButton mode="OFFBOARD" icon={Gamepad2} label="OFFBOARD (MANUAL)" currentMode={telemetry.mode} onClick={handleModeChange} />
             </div>
             <div className="flex items-center gap-6">
                <div className="flex items-center gap-2">
                    <BatteryCharging size={24} className={telemetry.battery_remaining < 20 ? 'text-red-500' : 'text-green-500'} />
                    <div className="flex flex-col items-end leading-none"><span className="text-xl font-mono font-bold">{telemetry.battery_voltage.toFixed(1)}V</span><span className="text-xs text-gray-400">{Math.round(telemetry.battery_remaining)}%</span></div>
                </div>
                <button onClick={handleArm} className={`px-6 py-2 rounded-full font-bold tracking-widest transition-all shadow-lg flex items-center gap-2 ${telemetry.armed ? 'bg-red-600 hover:bg-red-700 shadow-red-900/50 text-white animate-pulse' : 'bg-green-600 hover:bg-green-700 shadow-green-900/50 text-white'}`}><Power size={18} />{telemetry.armed ? 'DISARM' : 'ARM'}</button>
             </div>
          </header>

          <main className="flex-1 flex overflow-hidden relative">
             {/* DASHBOARD */}
             {activeTab === 'dashboard' && (
                 <>
                    <div className="flex-1 relative bg-gray-900">
                        <div className="absolute top-4 left-4 z-[50] bg-gray-800/90 backdrop-blur p-1 rounded-lg border border-gray-600 flex gap-1 shadow-lg">
                            <button onClick={() => setViewMode('map')} className={`px-3 py-1.5 rounded-md text-xs font-bold flex items-center gap-2 transition-all ${viewMode === 'map' ? 'bg-blue-600 text-white shadow' : 'text-gray-400 hover:text-white'}`}><MapIcon size={14} /> MAP</button>
                            <button onClick={() => setViewMode('camera')} className={`px-3 py-1.5 rounded-md text-xs font-bold flex items-center gap-2 transition-all ${viewMode === 'camera' ? 'bg-blue-600 text-white shadow' : 'text-gray-400 hover:text-white'}`}><Video size={14} /> CAM</button>
                        </div>
                        
                        <div id="map-container" className={viewMode === 'map' ? 'block' : 'hidden'}></div>
                        
                        {/* --- NEW CAMERA FEED COMPONENT --- */}
                        {viewMode === 'camera' && <LiveVideoFeed telemetry={telemetry} />}

                        {viewMode === 'map' && (
                            <div className="absolute bottom-4 left-4 z-[40] bg-gray-900/80 backdrop-blur p-2 rounded border border-gray-600 text-xs font-mono text-white pointer-events-none"><div>LAT: {telemetry.latitude.toFixed(6)}</div><div>LNG: {telemetry.longitude.toFixed(6)}</div></div>
                        )}
                    </div>
                    <div className="w-80 bg-gray-800 border-l border-gray-700 flex flex-col overflow-y-auto z-20 shadow-xl">
                        <div className="p-6 border-b border-gray-700 flex flex-col items-center">
                            <h3 className="text-gray-400 text-xs font-bold uppercase mb-4 w-full text-left">Artificial Horizon</h3>
                            <ArtificialHorizon pitch={telemetry.pitch} roll={telemetry.roll} />
                            <div className="flex w-full justify-between px-2 font-mono text-sm"><div className="text-blue-400 text-center"><span className="text-xs text-gray-500 block">PITCH</span>{telemetry.pitch.toFixed(1)}°</div><div className="text-green-400 text-center"><span className="text-xs text-gray-500 block">ROLL</span>{telemetry.roll.toFixed(1)}°</div></div>
                        </div>
                        <div className="grid grid-cols-2 gap-px bg-gray-700 border-b border-gray-700">
                            <div className="bg-gray-800 p-4 flex flex-col items-center"><ArrowUp size={20} className="text-blue-400 mb-1" /><span className="text-2xl font-mono font-bold">{Math.max(0, telemetry.altitude_relative).toFixed(1)}</span><span className="text-xs text-gray-500">ALT (m)</span></div>
                            <div className="bg-gray-800 p-4 flex flex-col items-center"><Activity size={20} className="text-yellow-400 mb-1" /><span className="text-2xl font-mono font-bold">{telemetry.ground_speed.toFixed(1)}</span><span className="text-xs text-gray-500">SPEED (m/s)</span></div>
                            <div className="bg-gray-800 p-4 flex flex-col items-center"><Compass size={20} className="text-red-400 mb-1" /><span className="text-2xl font-mono font-bold">{telemetry.heading.toFixed(0)}°</span><span className="text-xs text-gray-500">HEADING</span></div>
                            <div className="bg-gray-800 p-4 flex flex-col items-center"><Signal size={20} className="text-green-400 mb-1" /><span className="text-2xl font-mono font-bold">{telemetry.satellites}</span><span className="text-xs text-gray-500">SATS</span></div>
                        </div>
                        <div className="flex-1 flex flex-col min-h-0 bg-black">
                            <div className="bg-gray-700 px-3 py-1 text-xs font-bold text-gray-300 flex justify-between items-center"><span>CONSOLE</span><span className="bg-green-500 w-2 h-2 rounded-full animate-pulse"></span></div>
                            <div className="flex-1 overflow-y-auto p-2 font-mono text-xs space-y-1">
                                {logs.map((log) => (<div key={log.id} className="border-b border-gray-800/50 pb-1"><span className="text-gray-500">[{log.timestamp}]</span> <span className="text-blue-400">{log.type}:</span> <span className="text-gray-400">{log.message}</span></div>))}
                                <div ref={logsEndRef} />
                            </div>
                        </div>
                    </div>
                 </>
             )}

             {/* MISSION PLANNER */}
             {activeTab === 'missions' && (
                 <div className="flex-1 bg-gray-900 p-8 overflow-y-auto">
                    <h2 className="text-2xl font-bold text-white mb-6 flex items-center gap-3"><MapIcon/> Mission Planner</h2>
                    <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                        <div className="bg-gray-800 rounded-xl border border-gray-700 p-6">
                            <h3 className="text-lg font-bold text-blue-400 mb-4">Mission Control</h3>
                            <div className="space-y-4">
                                <div className="bg-gray-700/50 p-4 rounded-lg flex items-center justify-between border border-gray-600">
                                    <div className="flex items-center gap-3">
                                        <div className="w-12 h-12 bg-blue-900/50 rounded flex items-center justify-center text-blue-400 font-bold"><RefreshCw size={24}/></div>
                                        <div><div className="font-bold text-white">Figure 8 (Infinite)</div><div className="text-xs text-gray-400">Pattern: 8 • Alt: 15m • Auto Land</div></div>
                                    </div>
                                    <div className="flex flex-col gap-2">
                                        <button onClick={uploadFigure8} className="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-lg flex items-center gap-2 text-xs font-bold shadow-lg"><UploadCloud size={14}/> UPLOAD</button>
                                        <button onClick={startMission} className="bg-green-600 hover:bg-green-700 text-white px-4 py-2 rounded-lg flex items-center gap-2 text-xs font-bold shadow-lg"><Play size={14}/> START</button>
                                    </div>
                                </div>
                                <div className="p-4 bg-yellow-900/20 border border-yellow-700/30 rounded-lg text-xs text-yellow-200"><strong>How to run:</strong> <br/>1. <strong>Arm</strong> drone first.<br/>2. Click <strong>UPLOAD</strong> to send waypoints.<br/>3. Click <strong>START</strong> to fly mission.<br/>Drone will fly Figure 8 pattern then land at origin.</div>
                            </div>
                        </div>
                        <div className="bg-gray-800 rounded-xl border border-gray-700 p-6 flex flex-col items-center justify-center min-h-[300px]"><MapIcon size={48} className="text-gray-600 mb-4 opacity-50"/><p className="text-gray-500">Mission Map Preview</p><p className="text-xs text-gray-600">(Map view is on Dashboard)</p></div>
                    </div>
                 </div>
             )}

             {/* PRE-FLIGHT CHECK */}
             {activeTab === 'preflight' && (
                 <div className="flex-1 bg-gray-900 p-8 overflow-y-auto">
                    <h2 className="text-2xl font-bold text-white mb-6 flex items-center gap-3"><ClipboardCheck/> Pre-Flight Checklist</h2>
                    <div className="bg-gray-800 rounded-xl border border-gray-700 max-w-3xl overflow-hidden">
                        {preflightData.map((item) => (
                            <div key={item.id} className="flex items-center justify-between p-5 border-b border-gray-700 last:border-0 hover:bg-gray-750">
                                <div className="flex items-center gap-4">
                                    <div className={`w-8 h-8 rounded-full flex items-center justify-center ${item.status === 'PASS' ? 'bg-green-500/20 text-green-500' : item.status === 'FAIL' ? 'bg-red-500/20 text-red-500' : item.status === 'WARN' ? 'bg-yellow-500/20 text-yellow-500' : 'bg-gray-700 text-gray-500'}`}>
                                        {item.status === 'PASS' ? <CheckCircle size={18}/> : item.status === 'FAIL' ? <XCircle size={18}/> : item.status === 'WARN' ? <AlertTriangle size={18}/> : <Activity size={18}/>}
                                    </div>
                                    <div><span className="text-gray-200 text-lg block">{item.label}</span><span className={`text-xs ${item.status === 'PASS' ? 'text-green-400' : item.status === 'FAIL' ? 'text-red-400' : 'text-gray-500'}`}>{item.detail}</span></div>
                                </div>
                                <div className={`px-3 py-1 rounded text-xs font-bold ${item.status === 'PASS' ? 'bg-green-900 text-green-300' : item.status === 'FAIL' ? 'bg-red-900 text-red-300' : item.status === 'WARN' ? 'bg-yellow-900 text-yellow-300' : 'bg-gray-700 text-gray-400'}`}>{item.status}</div>
                            </div>
                        ))}
                    </div>
                    <div className="mt-6 flex justify-end max-w-3xl"><button onClick={runPreflightCheck} className="px-8 py-3 bg-blue-600 hover:bg-blue-700 text-white rounded-lg font-bold shadow-lg flex items-center gap-2"><RefreshCw size={20} className={preflightData.some(i => i.status === 'PENDING' && i.detail === 'Checking...') ? 'animate-spin' : ''}/> RUN DIAGNOSTICS</button></div>
                 </div>
             )}
          </main>
      </div>
    </div>
  );
};

export default App;