import React, { useState, useEffect, useRef } from 'react';
import { 
  Activity, BatteryCharging, Compass, Power, Send, Signal, 
  AlertTriangle, ArrowUp, Gamepad2, Anchor, LayoutDashboard, 
  Map as MapIcon, Video, ClipboardCheck, Home, ArrowDown, Play, 
  CheckCircle, XCircle, RefreshCw, UploadCloud, Mic, MicOff, Maximize,
  Grid, Box, Target, Box as Cube
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

  #map-container-main, #map-container-pip { width: 100%; height: 100%; z-index: 0; background: #111827; }
  
  .mic-active { animation: pulse-red 1.5s infinite; }
  @keyframes pulse-red {
    0% { box-shadow: 0 0 0 0 rgba(239, 68, 68, 0.7); }
    70% { box-shadow: 0 0 0 10px rgba(239, 68, 68, 0); }
    100% { box-shadow: 0 0 0 0 rgba(239, 68, 68, 0); }
  }

  /* Rotasi Marker di Peta */
  .drone-marker-arrow {
    transition: transform 0.2s linear;
    transform-origin: center center;
  }
`;

// --- TYPES ---
interface TelemetryData {
  connected: boolean; armed: boolean; mode: string;
  battery_voltage: number; battery_remaining: number; latitude: 0; longitude: 0;
  altitude_relative: number; heading: number; pitch: number; roll: number;
  satellites: number; ground_speed: number; climb_rate: number;
}

interface LogMessage { id: number; timestamp: string; type: string; message: string; }
interface PreflightItem { id: string; label: string; status: 'PENDING' | 'PASS' | 'FAIL' | 'WARN'; detail: string; }

// --- 3D DRONE COMPONENT ---
const Drone3DView = ({ telemetry, isMain }: { telemetry: TelemetryData, isMain: boolean }) => {
    const containerRef = useRef<HTMLDivElement>(null);
    const sceneRef = useRef<any>(null);
    const droneRef = useRef<any>(null);
    const rendererRef = useRef<any>(null);
    const cameraRef = useRef<any>(null);

    useEffect(() => {
        if (!document.getElementById('three-js')) {
            const script = document.createElement("script"); script.id = 'three-js'; script.src = "https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"; script.async = true; 
            script.onload = init3D; document.head.appendChild(script);
        } else { setTimeout(init3D, 100); }

        function init3D() {
            const THREE = (window as any).THREE;
            if (!THREE || !containerRef.current || sceneRef.current) return;

            const scene = new THREE.Scene(); scene.background = new THREE.Color(0x111827);
            const camera = new THREE.PerspectiveCamera(75, containerRef.current.clientWidth / containerRef.current.clientHeight, 0.1, 1000);
            camera.position.set(0, 2, 3); camera.lookAt(0, 0, 0);
            
            const renderer = new THREE.WebGLRenderer({ antialias: true });
            renderer.setSize(containerRef.current.clientWidth, containerRef.current.clientHeight);
            containerRef.current.innerHTML = ''; 
            containerRef.current.appendChild(renderer.domElement);
            
            // Grid & Light
            scene.add(new THREE.GridHelper(20, 20, 0x444444, 0x222222));
            scene.add(new THREE.AmbientLight(0xffffff, 0.6));
            const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
            dirLight.position.set(5, 10, 7);
            scene.add(dirLight);

            // Drone Model (Simplified Quad)
            const droneGroup = new THREE.Group();
            const body = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.1, 0.8), new THREE.MeshStandardMaterial({ color: 0x3b82f6 }));
            const arm1 = new THREE.Mesh(new THREE.BoxGeometry(1.2, 0.05, 0.05), new THREE.MeshStandardMaterial({ color: 0x4b5563 }));
            arm1.rotation.y = Math.PI / 4;
            const arm2 = new THREE.Mesh(new THREE.BoxGeometry(1.2, 0.05, 0.05), new THREE.MeshStandardMaterial({ color: 0x4b5563 }));
            arm2.rotation.y = -Math.PI / 4;
            const nose = new THREE.Mesh(new THREE.ConeGeometry(0.1, 0.3, 8), new THREE.MeshStandardMaterial({ color: 0xff0000 }));
            nose.rotation.x = Math.PI / 2;
            nose.position.z = -0.5;

            droneGroup.add(body, arm1, arm2, nose);
            scene.add(droneGroup);
            
            sceneRef.current = scene;
            cameraRef.current = camera;
            rendererRef.current = renderer;
            droneRef.current = droneGroup;

            const animate = () => {
                requestAnimationFrame(animate);
                renderer.render(scene, camera);
            };
            animate();
        }
    }, []);

    useEffect(() => {
        if (droneRef.current) {
            droneRef.current.rotation.x = telemetry.pitch * (Math.PI/180);
            droneRef.current.z = -telemetry.roll * (Math.PI/180);
            droneRef.current.y = -telemetry.heading * (Math.PI/180);
        }
    }, [telemetry]);

    useEffect(() => {
        if (rendererRef.current && cameraRef.current && containerRef.current) {
            const w = containerRef.current.clientWidth;
            const h = containerRef.current.clientHeight;
            cameraRef.current.aspect = w / h;
            cameraRef.current.updateProjectionMatrix();
            rendererRef.current.setSize(w, h);
        }
    }, [isMain]);

    return <div ref={containerRef} className="w-full h-full bg-[#111827] relative">
        {!isMain && <div className="absolute top-2 left-2 bg-black/50 px-2 py-1 rounded text-[10px] font-bold text-white">3D</div>}
    </div>;
};

// --- LIVE VIDEO FEED ---
const LiveVideoFeed = ({ telemetry, isMain }: { telemetry: TelemetryData, isMain: boolean }) => {
    const videoRef = useRef<HTMLVideoElement>(null);
    useEffect(() => {
        navigator.mediaDevices.getUserMedia({ video: true }).then(stream => { if(videoRef.current) videoRef.current.srcObject = stream; }).catch(()=>{});
    }, []);
    return (
        <div className="w-full h-full relative bg-black flex flex-col items-center justify-center overflow-hidden">
            <video ref={videoRef} autoPlay playsInline muted className="w-full h-full object-cover" />
            {isMain && <div className="absolute inset-4 border-2 border-white/20 rounded-lg pointer-events-none p-4 z-20"><span className="bg-red-600 px-2 py-1 rounded text-xs text-white animate-pulse">LIVE FEED</span></div>}
            {!isMain && <div className="absolute top-2 left-2 bg-black/50 px-2 py-1 rounded text-[10px] font-bold text-white">CAM</div>}
        </div>
    );
};

// --- MAP VIEW ---
const MapView = ({ id, telemetry, pathData, isMain }: any) => {
    const mapRef = useRef<any>(null);
    const markerRef = useRef<any>(null);
    const polylineRef = useRef<any>(null);

    useEffect(() => {
        if (!document.getElementById('leaflet-js')) {
            const s = document.createElement("script"); s.id = 'leaflet-js'; s.src = "https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"; s.async = true; s.onload = init; document.head.appendChild(s);
            const l = document.createElement("link"); l.id = 'leaflet-css'; l.rel = "stylesheet"; l.href = "https://unpkg.com/leaflet@1.7.1/dist/leaflet.css"; document.head.appendChild(l);
        } else { setTimeout(init, 100); }
        function init() {
            const L = (window as any).L;
            if (!L || mapRef.current || !document.getElementById(id)) return;
            
            const startLat = telemetry.latitude !== 0 ? telemetry.latitude : -6.2088;
            const startLng = telemetry.longitude !== 0 ? telemetry.longitude : 106.8456;

            const map = L.map(id, { zoomControl: false, attributionControl: false }).setView([startLat, startLng], 16);
            L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', { maxZoom: 19 }).addTo(map);
            
            // Custom Drone Icon (SVG)
            const droneIcon = L.divIcon({ 
                html: `<div class="drone-marker-arrow" style="width: 40px; height: 40px; display: flex; align-items: center; justify-content: center;">
                        <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="#ef4444" stroke="white" stroke-width="1.5" style="filter: drop-shadow(0 0 4px rgba(0,0,0,0.8)); width: 100%; height: 100%;">
                            <path d="M12 2L2 22l10-5 10 5-10-20z"/>
                        </svg>
                       </div>`, 
                className: '', 
                iconSize: [40, 40], 
                iconAnchor: [20, 20] 
            });
            
            const marker = L.marker([startLat, startLng], { icon: droneIcon }).addTo(map);
            const polyline = L.polyline(pathData, { color: 'red', weight: 3, opacity: 0.7 }).addTo(map);
            
            mapRef.current = map; 
            markerRef.current = marker;
            polylineRef.current = polyline;
        }
    }, [id]);

    // Update Map Position & Rotation
    useEffect(() => {
        if(!mapRef.current || telemetry.latitude===0) return;
        const pos = [telemetry.latitude, telemetry.longitude];
        
        // Update Marker Position
        if(markerRef.current) {
            markerRef.current.setLatLng(pos);
            
            // Update Rotation (Heading)
            const iconEl = markerRef.current.getElement();
            if (iconEl) {
                const arrow = iconEl.querySelector('.drone-marker-arrow');
                if (arrow) arrow.style.transform = `rotate(${telemetry.heading}deg)`;
            }
        }

        // Update Path
        if(polylineRef.current) {
            polylineRef.current.setLatLngs(pathData);
        }

        if(isMain && telemetry.connected) mapRef.current.panTo(pos, { animate: true, duration: 0.2 });
        else if(!isMain) mapRef.current.setView(pos);
    }, [telemetry.latitude, telemetry.longitude, telemetry.heading, pathData]);

    return <div id={id} className="w-full h-full relative">{!isMain && <div className="absolute top-2 left-2 bg-black/50 px-2 py-1 rounded text-[10px] text-white">MAP</div>}</div>;
};

// --- SUB-COMPONENTS ---
const FlightModeButton = ({ mode, icon: Icon, label, currentMode, onClick }: any) => {
    const isActive = currentMode.includes(mode);
    return (
      <button onClick={() => onClick(mode)} title={label} className={`w-10 h-10 rounded-full flex items-center justify-center transition-all border-2 ${isActive ? 'bg-blue-600 border-blue-400 text-white shadow-[0_0_10px_rgba(59,130,246,0.5)]' : 'bg-gray-700 border-gray-600 text-gray-400 hover:bg-gray-600 hover:text-white'}`}><Icon size={18} /></button>
    )
};

const MissionCard = ({ title, icon: Icon, description, onUpload }: any) => (
    <div className="bg-gray-700/50 p-4 rounded-lg border border-gray-600 hover:border-blue-500 transition-all cursor-pointer group" onClick={onUpload}>
        <div className="flex items-center gap-3 mb-2">
            <div className="w-10 h-10 bg-blue-900/50 rounded-lg flex items-center justify-center text-blue-400 group-hover:bg-blue-600 group-hover:text-white transition-colors"><Icon size={20} /></div>
            <h4 className="font-bold text-gray-200 group-hover:text-white">{title}</h4>
        </div>
        <p className="text-xs text-gray-400 mb-3">{description}</p>
        <div className="flex items-center gap-2 text-xs text-blue-400 font-bold"><UploadCloud size={14}/> CLICK TO UPLOAD</div>
    </div>
);

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
  const [mainView, setMainView] = useState<'map' | 'camera' | '3d'>('map'); 
  const [wsStatus, setWsStatus] = useState<string>('DISCONNECTED');
  const [isVoiceActive, setIsVoiceActive] = useState(false); 

  const [telemetry, setTelemetry] = useState<TelemetryData>({
    connected: false, armed: false, mode: 'DISARMED',
    battery_voltage: 0, battery_remaining: 0, latitude: 0, longitude: 0, 
    altitude_relative: 0, heading: 0, pitch: 0, roll: 0,
    satellites: 0, ground_speed: 0, climb_rate: 0
  });

  const [preflightData, setPreflightData] = useState<PreflightItem[]>([
      { id: 'imu', label: 'IMU Sensors (Gyro/Accel)', status: 'PENDING', detail: 'Waiting Check...' },
      { id: 'mag', label: 'Magnetometer', status: 'PENDING', detail: 'Waiting Check...' },
      { id: 'gps', label: 'GPS Lock (>6 Sats)', status: 'PENDING', detail: 'Waiting Check...' },
      { id: 'bat', label: 'Battery Voltage', status: 'PENDING', detail: 'Waiting Check...' },
      { id: 'rc', label: 'RC Signal Strength', status: 'PENDING', detail: 'Waiting Check...' },
      { id: 'home', label: 'Home Position Set', status: 'PENDING', detail: 'Waiting Check...' }
  ]);

  const [logs, setLogs] = useState<LogMessage[]>([]);
  const logsEndRef = useRef<HTMLDivElement>(null);
  
  const pathDataRef = useRef<[number, number][]>([]);
  const wsRef = useRef<WebSocket | null>(null);
  const currentModeRef = useRef<string>("DISARMED");
  const recognitionRef = useRef<any>(null);

  const addLog = (type: string, message: string) => {
    setLogs(prev => [...prev.slice(-49), { id: Date.now(), timestamp: new Date().toLocaleTimeString(), type, message }]);
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
            setTelemetry(prev => {
                // Record Path if Armed and Position Valid
                if(prev.armed && data.latitude !== 0 && data.longitude !== 0) {
                    pathDataRef.current.push([data.latitude, data.longitude]);
                }
                return { ...prev, ...data }
            });
        } catch (e) {}
    };
    ws.onclose = () => { setWsStatus('DISCONNECTED'); setTelemetry(t => ({ ...t, connected: false })); addLog('ERR', 'Disconnected'); };
    wsRef.current = ws;
    return () => ws.close();
  }, []);

  // --- COMMANDS ---
  const sendCommand = (payload: any) => { if(wsRef.current?.readyState === WebSocket.OPEN) wsRef.current.send(JSON.stringify(payload)); };
  const handleModeChange = (newMode: string) => sendCommand({ type: 'SET_MODE', mode: newMode });
  const handleArm = () => sendCommand({ type: 'COMMAND_LONG', command: 'MAV_CMD_COMPONENT_ARM_DISARM', param1: telemetry.armed ? 0 : 1 });
  const runPreflightCheck = () => { addLog('CMD', 'Running Diagnostics...'); setPreflightData(prev => prev.map(p => ({...p, status: 'PENDING', detail: 'Checking...'}))); sendCommand({ type: 'REQ_PREFLIGHT' }); };
  const uploadFigure8 = () => { addLog('CMD', 'Uploading Mission...'); sendCommand({ type: 'UPLOAD_MISSION_FIGURE8' }); };
  const uploadSquare = () => { addLog('CMD', 'Uploading Square...'); sendCommand({ type: 'UPLOAD_MISSION_SQUARE' }); };
  const uploadScan = () => { addLog('CMD', 'Uploading Grid Scan...'); sendCommand({ type: 'UPLOAD_MISSION_SCAN' }); };
  const uploadSpiral = () => { addLog('CMD', 'Uploading Spiral...'); sendCommand({ type: 'UPLOAD_MISSION_SPIRAL' }); };
  const startMission = () => { addLog('CMD', 'Start Mission...'); sendCommand({ type: 'SET_MODE', mode: 'MISSION' }); };

  // --- VOICE CONTROL ---
  useEffect(() => {
      const SpeechRecognition = (window as any).SpeechRecognition || (window as any).webkitSpeechRecognition;
      if (!SpeechRecognition) return;
      const recognition = new SpeechRecognition();
      recognition.continuous = true; recognition.interimResults = false; 
      recognition.lang = 'id-ID'; // Set to Indonesian

      recognition.onresult = (event: any) => {
          const t = event.results[event.results.length - 1][0].transcript.trim().toLowerCase();
          addLog('VOICE', `"${t}"`);
          
          // Basic Modes
          if (t.includes('take off') || t.includes('terbang')) handleModeChange('TAKEOFF');
          else if (t.includes('land') || t.includes('mendarat')) handleModeChange('LAND');
          else if (t.includes('rtl') || t.includes('pulang')) handleModeChange('RTL');
          else if (t.includes('hold') || t.includes('diam') || t.includes('stop')) handleModeChange('HOLD');
          
          // Arming
          else if (t.includes('nyalakan mesin') || t.includes('arm')) {
              if(!telemetry.armed) handleArm();
          }
          else if (t.includes('matikan mesin') || t.includes('disarm')) {
              if(telemetry.armed) handleArm();
          }

          // Manual Control (Nudge)
          // "Maju" -> Forward 5m/s
          else if (t.includes('maju') || t.includes('forward')) sendCommand({ type: 'MANUAL_CONTROL', x: 5, y: 0, z: 0, r: 0 });
          else if (t.includes('mundur') || t.includes('backward')) sendCommand({ type: 'MANUAL_CONTROL', x: -5, y: 0, z: 0, r: 0 });
          else if (t.includes('kiri') || t.includes('left')) sendCommand({ type: 'MANUAL_CONTROL', x: 0, y: -5, z: 0, r: 0 });
          else if (t.includes('kanan') || t.includes('right')) sendCommand({ type: 'MANUAL_CONTROL', x: 0, y: 5, z: 0, r: 0 });
          else if (t.includes('naik') || t.includes('up') || t.includes('terbang lebih tinggi')) sendCommand({ type: 'MANUAL_CONTROL', x: 0, y: 0, z: -2, r: 0 }); // NED Z- is Up
          else if (t.includes('turun') || t.includes('down')) sendCommand({ type: 'MANUAL_CONTROL', x: 0, y: 0, z: 2, r: 0 });
      };
      recognition.onend = () => { if (isVoiceActive) recognition.start(); };
      recognitionRef.current = recognition;
  }, [isVoiceActive, telemetry.armed]); // Re-bind on arm change to update state closure

  const toggleVoice = () => {
      if (isVoiceActive) { recognitionRef.current?.stop(); setIsVoiceActive(false); addLog('SYS', 'Voice OFF'); } 
      else { try { recognitionRef.current?.start(); setIsVoiceActive(true); addLog('SYS', 'Voice ON (ID)'); } catch(e) {} }
  };

  // --- KEYBOARD ---
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (!currentModeRef.current.includes("OFFBOARD")) return;
      if(["ArrowUp","ArrowDown","Space"].indexOf(e.code) > -1) e.preventDefault();
      let x = 0, y = 0, z = 0, r = 0;
      if (e.key === 'w') x = 5; if (e.key === 's') x = -5;   
      if (e.key === 'a') y = -5; if (e.key === 'd') y = 5;    
      if (e.key === 'ArrowLeft') r = -40; if (e.key === 'ArrowRight') r = 40;
      if (e.key === 'ArrowUp') z = -2; if (e.key === 'ArrowDown') z = 2;
      if (e.code === 'Space') z = -2;
      if (x||y||z||r) sendCommand({ type: 'MANUAL_CONTROL', x, y, z, r });
    };
    const handleKeyUp = () => { if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN && currentModeRef.current.includes("OFFBOARD")) wsRef.current.send(JSON.stringify({ type: 'MANUAL_CONTROL', x: 0, y: 0, z: 0, r: 0 })); };
    window.addEventListener('keydown', handleKeyDown); window.addEventListener('keyup', handleKeyUp);
    return () => { window.removeEventListener('keydown', handleKeyDown); window.removeEventListener('keyup', handleKeyUp); };
  }, []);

  // --- VIEW SWITCHER ---
  const switchView = (target: 'map' | 'camera' | '3d') => { setMainView(target); };

  return (
    <div className="flex h-screen bg-gray-900 text-gray-100 font-sans overflow-hidden">
      <style>{styles}</style>
      <nav className="w-20 bg-gray-800 border-r border-gray-700 flex flex-col items-center py-6 gap-6 z-30 shadow-xl">
         <div className="bg-blue-600 p-3 rounded-xl mb-4 shadow-lg shadow-blue-900/50"><Send size={24} className="text-white" /></div>
         <button onClick={() => setActiveTab('dashboard')} className={`p-3 rounded-xl transition-all ${activeTab === 'dashboard' ? 'bg-gray-700 text-blue-400' : 'text-gray-400 hover:text-white'}`}><LayoutDashboard size={24} /></button>
         <button onClick={() => setActiveTab('missions')} className={`p-3 rounded-xl transition-all ${activeTab === 'missions' ? 'bg-gray-700 text-blue-400' : 'text-gray-400 hover:text-white'}`}><MapIcon size={24} /></button>
         <button onClick={() => setActiveTab('preflight')} className={`p-3 rounded-xl transition-all ${activeTab === 'preflight' ? 'bg-gray-700 text-blue-400' : 'text-gray-400 hover:text-white'}`}><ClipboardCheck size={24} /></button>
         <div className="flex-1"></div>
         <div className="text-[10px] text-gray-500 font-mono rotate-180" style={{writingMode: 'vertical-rl'}}>CAKSA GCS v5.0</div>
      </nav>

      <div className="flex-1 flex flex-col min-w-0">
          <header className="h-20 bg-gray-800 border-b border-gray-700 flex items-center justify-between px-6 shadow-md z-20">
             <div className="flex flex-col">
                <h1 className="font-bold text-xl tracking-wider text-white">CAKSA <span className="text-blue-400">GCS</span></h1>
                <span className="text-xs text-gray-400 font-mono">{telemetry.connected ? 'LINK ACTIVE' : 'NO LINK'}</span>
             </div>
             
             <div className="flex items-center gap-4 bg-gray-900/50 px-6 py-2 rounded-full border border-gray-700">
                <span className="text-xs text-gray-500 font-bold mr-2">MODES</span>
                <FlightModeButton mode="HOLD" icon={Anchor} label="LOITER / HOLD" currentMode={telemetry.mode} onClick={handleModeChange} />
                <FlightModeButton mode="TAKEOFF" icon={ArrowUp} label="TAKEOFF" currentMode={telemetry.mode} onClick={handleModeChange} />
                <FlightModeButton mode="LAND" icon={ArrowDown} label="LAND" currentMode={telemetry.mode} onClick={handleModeChange} />
                <FlightModeButton mode="RTL" icon={Home} label="RETURN TO LAUNCH" currentMode={telemetry.mode} onClick={handleModeChange} />
                <FlightModeButton mode="OFFBOARD" icon={Gamepad2} label="OFFBOARD (MANUAL)" currentMode={telemetry.mode} onClick={handleModeChange} />
                <div className="w-px h-8 bg-gray-700 mx-2"></div>
                <button onClick={toggleVoice} className={`w-10 h-10 rounded-full flex items-center justify-center border-2 ${isVoiceActive ? 'bg-red-600 border-red-400 mic-active' : 'bg-gray-700 border-gray-600'}`}>{isVoiceActive ? <Mic size={18}/> : <MicOff size={18}/>}</button>
             </div>

             <div className="flex items-center gap-6">
                <div className="flex items-center gap-2">
                    <BatteryCharging size={24} className={telemetry.battery_remaining < 20 ? 'text-red-500' : 'text-green-500'} />
                    <div className="flex flex-col items-end leading-none">
                        <span className="text-xl font-mono font-bold">{telemetry.battery_voltage.toFixed(1)}V</span>
                        <span className="text-xs text-gray-400">{Math.round(telemetry.battery_remaining)}%</span>
                    </div>
                </div>
                <button onClick={handleArm} className={`px-6 py-2 rounded-full font-bold tracking-widest transition-all shadow-lg flex items-center gap-2 ${telemetry.armed ? 'bg-red-600 hover:bg-red-700 animate-pulse' : 'bg-green-600 hover:bg-green-700'}`}><Power size={18} />{telemetry.armed ? 'DISARM' : 'ARM'}</button>
             </div>
          </header>

          <main className="flex-1 flex overflow-hidden relative">
             {activeTab === 'dashboard' && (
                 <>
                    <div className="flex-1 relative bg-gray-900 overflow-hidden">
                        <div className="absolute top-4 left-4 z-[50] bg-gray-800/90 backdrop-blur p-1 rounded-lg border border-gray-600 flex gap-1 shadow-lg">
                            <button onClick={() => setMainView('map')} className={`px-3 py-1.5 rounded-md text-xs font-bold flex items-center gap-2 transition-all ${mainView === 'map' ? 'bg-blue-600 text-white shadow' : 'text-gray-400 hover:text-white'}`}><MapIcon size={14} /> MAP</button>
                            <button onClick={() => setMainView('camera')} className={`px-3 py-1.5 rounded-md text-xs font-bold flex items-center gap-2 transition-all ${mainView === 'camera' ? 'bg-blue-600 text-white shadow' : 'text-gray-400 hover:text-white'}`}><Video size={14} /> CAM</button>
                            <button onClick={() => setMainView('3d')} className={`px-3 py-1.5 rounded-md text-xs font-bold flex items-center gap-2 transition-all ${mainView === '3d' ? 'bg-blue-600 text-white shadow' : 'text-gray-400 hover:text-white'}`}><Cube size={14} /> 3D</button>
                        </div>
                        <div className="absolute inset-0 z-0">
                            {mainView === 'map' && <MapView id="map-container-main" telemetry={telemetry} pathData={pathDataRef.current} isMain={true} />}
                            {mainView === 'camera' && <LiveVideoFeed telemetry={telemetry} isMain={true} />}
                            {mainView === '3d' && <Drone3DView telemetry={telemetry} isMain={true} />}
                        </div>
                        <div onClick={() => switchView(mainView === 'map' ? 'camera' : 'map')} className="absolute top-4 right-4 w-72 h-48 bg-black rounded-xl border-2 border-white/30 shadow-2xl z-50 overflow-hidden cursor-pointer hover:border-blue-500 hover:scale-105 transition-all group">
                            {mainView === 'map' ? <LiveVideoFeed telemetry={telemetry} isMain={false} /> : <MapView id="map-container-pip" telemetry={telemetry} pathData={pathDataRef.current} isMain={false} />}
                            <div className="absolute inset-0 bg-black/40 flex items-center justify-center opacity-0 group-hover:opacity-100 transition-opacity"><Maximize className="text-white drop-shadow-md"/></div>
                        </div>
                    </div>
                    <div className="w-80 bg-gray-800 border-l border-gray-700 flex flex-col overflow-y-auto z-20 shadow-xl">
                        <div className="p-6 border-b border-gray-700 flex flex-col items-center">
                            <h3 className="text-gray-400 text-xs font-bold uppercase mb-4 w-full text-left">Artificial Horizon</h3>
                            <ArtificialHorizon pitch={telemetry.pitch} roll={telemetry.roll} />
                            <div className="flex w-full justify-between px-2 font-mono text-sm"><div className="text-blue-400 text-center"><span className="text-xs text-gray-500 block">PITCH</span>{telemetry.pitch.toFixed(1)}°</div><div className="text-green-400 text-center"><span className="text-xs text-gray-500 block">ROLL</span>{telemetry.roll.toFixed(1)}°</div></div>
                        </div>
                        <div className="grid grid-cols-2 gap-px bg-gray-700 border-b border-gray-700">
                            <div className="bg-gray-800 p-4 flex flex-col items-center"><ArrowUp size={20} className="text-blue-400 mb-1" /><span className="text-2xl font-mono font-bold">{Math.max(0, telemetry.altitude_relative).toFixed(1)}</span><span className="text-xs text-gray-500">ALT (m)</span></div>
                            <div className="bg-gray-800 p-4 flex flex-col items-center"><Activity size={20} className="text-yellow-400 mb-1" /><span className="text-2xl font-mono font-bold">{telemetry.ground_speed.toFixed(1)}</span><span className="text-xs text-gray-500">SPD (m/s)</span></div>
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

             {activeTab === 'missions' && (
                 <div className="flex-1 bg-gray-900 p-8 overflow-y-auto">
                    <h2 className="text-2xl font-bold text-white mb-6 flex items-center gap-3"><MapIcon/> Mission Planner</h2>
                    <div className="max-w-4xl mx-auto">
                        <div className="bg-gray-800 rounded-xl border border-gray-700 p-6 mb-6 flex items-center justify-between">
                            <div>
                                <h3 className="text-lg font-bold text-white">Mission Status</h3>
                                <p className="text-sm text-gray-400">{telemetry.armed ? "System Armed & Ready" : "System Disarmed"} • Mode: {telemetry.mode}</p>
                            </div>
                            <button onClick={startMission} className={`px-6 py-3 rounded-lg font-bold shadow-lg flex items-center gap-2 transition-all ${telemetry.armed ? 'bg-green-600 hover:bg-green-700 text-white' : 'bg-gray-700 text-gray-400 cursor-not-allowed'}`} disabled={!telemetry.armed}><Play size={20}/> START MISSION</button>
                        </div>
                        <div className="grid grid-cols-2 gap-4">
                            <MissionCard title="Figure 8 Infinite" icon={RefreshCw} description="Terbang pola angka 8 (Lemniscate) secara terus menerus." onUpload={uploadFigure8} />
                            <MissionCard title="Square Mapping" icon={Box} description="Terbang pola persegi mengelilingi titik awal." onUpload={uploadSquare} />
                            <MissionCard title="Grid Scan" icon={Grid} description="Pola Zig-Zag untuk survei foto udara." onUpload={uploadScan} />
                            <MissionCard title="Spiral Search" icon={Target} description="Terbang melingkar untuk pencarian area." onUpload={uploadSpiral} />
                        </div>
                    </div>
                 </div>
             )}

             {activeTab === 'preflight' && (
                 <div className="flex-1 bg-gray-900 p-8 overflow-y-auto">
                    <h2 className="text-2xl font-bold text-white mb-6 flex items-center gap-3"><ClipboardCheck/> Pre-Flight Checklist</h2>
                    <div className="bg-gray-800 rounded-xl border border-gray-700 max-w-3xl overflow-hidden">
                        {preflightData.map((item) => (
                            <div key={item.id} className="flex items-center justify-between p-5 border-b border-gray-700 last:border-0 hover:bg-gray-750">
                                <div className="flex items-center gap-4">
                                    <div className={`w-8 h-8 rounded-full flex items-center justify-center ${item.status === 'PASS' ? 'bg-green-500/20 text-green-500' : item.status === 'FAIL' ? 'bg-red-500/20 text-red-500' : item.status === 'WARN' ? 'bg-yellow-500/20 text-yellow-500' : 'bg-gray-700 text-gray-500'}`}>{item.status === 'PASS' ? <CheckCircle size={18}/> : item.status === 'FAIL' ? <XCircle size={18}/> : item.status === 'WARN' ? <AlertTriangle size={18}/> : <Activity size={18}/>}</div>
                                    <div><span className="text-gray-200 text-lg block">{item.label}</span><span className={`text-xs ${item.status === 'PASS' ? 'text-green-400' : item.status === 'FAIL' ? 'text-red-400' : 'text-gray-500'}`}>{item.detail}</span></div>
                                </div>
                                <div className={`px-3 py-1 rounded text-xs font-bold ${item.status === 'PASS' ? 'bg-green-900 text-green-300' : item.status === 'FAIL' ? 'bg-red-900 text-red-300' : item.status === 'WARN' ? 'bg-yellow-900 text-yellow-300' : 'bg-gray-700 text-gray-400'}`}>{item.status}</div>
                            </div>
                        ))}
                    </div>
                    <div className="mt-6 flex justify-end max-w-3xl"><button onClick={runPreflightCheck} className="px-8 py-3 bg-blue-600 hover:bg-blue-700 text-white rounded-lg font-bold shadow-lg flex items-center gap-2"><RefreshCw size={20} className={preflightData.some(i => i.status === 'PENDING') ? 'animate-spin' : ''}/> RUN DIAGNOSTICS</button></div>
                 </div>
             )}
          </main>
      </div>
    </div>
  );
};

export default App;