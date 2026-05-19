import spidev
import time
import json
import math
import re
import ctypes
import struct
import logging
import os
from multiprocessing import Process, Value, Array, Queue
from flask import Flask, request, jsonify, render_template
from flask_socketio import SocketIO, emit
import threading

# --- LOGGING ---
logging.basicConfig(
    filename='galvo_perf.log',
    level=logging.DEBUG,
    format='%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins="*")

# --- GLOBAL OPTIMIZATIONS ---
GCODE_TOKENIZER = re.compile(r'([A-Za-z])([-.\d]*)')
CMD_STRUCT = struct.Struct('<HHBBxxI') 

class GalvoCmd(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_uint16),
        ("y", ctypes.c_uint16),
        ("laser_on", ctypes.c_uint8),
        ("pwm_val", ctypes.c_uint8),
        ("delay_us", ctypes.c_uint32)
    ]

try:
    galvo_c = ctypes.CDLL('./galvo_core.so')
    galvo_c.run_trajectory.argtypes = [
        ctypes.POINTER(GalvoCmd), ctypes.c_int, ctypes.POINTER(ctypes.c_int), 
        ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), 
        ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_double) 
    ]
except OSError:
    logger.error("galvo_core.so not found! C-backend will not work.")
    galvo_c = None

# --- SHARED MEMORY ---
STATUS_MAP = {0: "idle", 1: "parsing", 2: "framing", 3: "printing", 4: "paused", 5: "stopped", 6: "completed"}

c_status = Value('i', 0) 
c_progress = Value('i', 0)
c_current_x = Value('i', 2048)
c_current_y = Value('i', 2048)
c_current_pwm = Value('i', 0)
c_bounds = Array('i', [1024, 3072, 1024, 3072]) 

c_speed_factor = Value('d', 1.0)
c_laser_pwm_max = Value('i', 254)

preview_queue = Queue()

spi = spidev.SpiDev()
spi.open(0, 0) 
spi.max_speed_hz = 5000000 

def set_dac_value(channel, value):
    config = 0b00110000 
    if channel == 1: config |= 0b10000000 
    value = max(0, min(4095, int(value)))
    spi.xfer2([config | (value >> 8), value & 0xFF])

def build_c_array(gcode_text, pwm_max, preview_q, progress_var=None):
    def push_progress(pct):
        if progress_var and progress_var.value != pct:
            progress_var.value = pct
            time.sleep(0.01) 

    push_progress(1)

    text = re.sub(r';[^\n]*', '', gcode_text).replace(' ', '').upper()
    lines = text.split('\n')
    total_lines = len(lines) if len(lines) > 0 else 1

    commands = []
    append_cmd = commands.append 
    curr_x, curr_y = 0.0, 0.0
    is_abs = True
    last_cmd = 'G0'
    min_x, max_x, min_y, max_y = float('inf'), float('-inf'), float('inf'), float('-inf')
    max_s = 255.0

    for idx, line in enumerate(lines):
        if idx % 5000 == 0: push_progress(1 + int((idx / total_lines) * 49))

        if not line: continue
        parts = GCODE_TOKENIZER.findall(line)
        if not parts: continue

        c, v = parts[0]
        if c == 'G':
            if v.startswith('0') and len(v) > 1: v = v[1:] 
            cmd = c + v
            last_cmd = cmd
        elif c != 'M' and c != 'F' and c != 'S':
            cmd = last_cmd
            parts.insert(0, (cmd[0], cmd[1:])) 
        else:
            cmd = c + v
                
        if cmd == 'G90': is_abs = True
        elif cmd == 'G91': is_abs = False
        elif cmd in ('G0', 'G1'):
            target_x, target_y = curr_x, curr_y
            has_move, s_val, f_val = False, None, None
            for c2, v2 in parts[1:]:
                if not v2: continue
                val = float(v2)
                if c2 == 'X': target_x = val if is_abs else curr_x + val; has_move = True
                elif c2 == 'Y' or c2 == 'A': target_y = val if is_abs else curr_y + val; has_move = True
                elif c2 == 'S': s_val = val; max_s = max(max_s, val)
                elif c2 == 'F': f_val = val

            if has_move:
                min_x, max_x = min(min_x, target_x), max(max_x, target_x)
                min_y, max_y = min(min_y, target_y), max(max_y, target_y)
                append_cmd((cmd, target_x, target_y, s_val, f_val))
                curr_x, curr_y = target_x, target_y
            elif s_val is not None or f_val is not None:
                append_cmd((cmd, target_x, target_y, s_val, f_val))

        elif cmd in ('G2', 'G3'):
            target_x, target_y = curr_x, curr_y
            i, j, s_val, f_val = 0.0, 0.0, None, None
            for c2, v2 in parts[1:]:
                if not v2: continue
                val = float(v2)
                if c2 == 'X': target_x = val if is_abs else curr_x + val
                elif c2 == 'Y' or c2 == 'A': target_y = val if is_abs else curr_y + val
                elif c2 == 'I': i = val; elif c2 == 'J': j = val
                elif c2 == 'S': s_val = val; max_s = max(max_s, val)
                elif c2 == 'F': f_val = val

            center_x, center_y = curr_x + i, curr_y + j
            radius = math.hypot(i, j)
            if radius < 0.001:
                min_x, max_x = min(min_x, target_x), max(max_x, target_x)
                min_y, max_y = min(min_y, target_y), max(max_y, target_y)
                append_cmd(("G1", target_x, target_y, s_val, f_val))
                curr_x, curr_y = target_x, target_y
                continue

            start_angle = math.atan2(curr_y - center_y, curr_x - center_x)
            end_angle = math.atan2(target_y - center_y, target_x - center_x)
            is_cw = (cmd == 'G2')
            if is_cw and end_angle > start_angle: end_angle -= 2 * math.pi
            elif not is_cw and end_angle < start_angle: end_angle += 2 * math.pi
            
            angle_diff = end_angle - start_angle
            num_segments = max(2, int(abs(angle_diff * radius) * 10))

            for step in range(1, num_segments):
                theta = start_angle + angle_diff * (step / num_segments)
                px, py = center_x + radius * math.cos(theta), center_y + radius * math.sin(theta)
                min_x, max_x = min(min_x, px), max(max_x, px)
                min_y, max_y = min(min_y, py), max(max_y, py)
                append_cmd(("G1", px, py, s_val if step == 1 else None, f_val if step == 1 else None))

            min_x, max_x = min(min_x, target_x), max(max_x, target_x)
            min_y, max_y = min(min_y, target_y), max(max_y, target_y)
            append_cmd(("G1", target_x, target_y, s_val if num_segments == 1 else None, f_val if num_segments == 1 else None))
            curr_x, curr_y = target_x, target_y

        elif cmd in ('M3', 'M5'):
            s_val = None
            for c2, v2 in parts[1:]:
                if c2 == 'S' and v2: s_val = float(v2); max_s = max(max_s, s_val)
            append_cmd((cmd, s_val))
        elif cmd == 'G4':
            dwell = 0.0
            for c2, v2 in parts[1:]:
                if not v2: continue
                if c2 == 'S': dwell = float(v2)
                elif c2 == 'P': dwell = float(v2) / 1000.0
            if dwell > 0: append_cmd(("G4", dwell))

    if min_x == float('inf'): return None, 0

    range_x, range_y = max_x - min_x, max_y - min_y
    max_range = max(range_x, range_y)
    scale = 90.0 / max_range if max_range > 90.0 else 1.0
    center_x, center_y = (min_x + max_x) / 2.0, (min_y + max_y) / 2.0
    offset_x, offset_y = 50.0 - (center_x * scale), 50.0 - (center_y * scale)

    c_bounds[0] = max(0, min(4095, int(((min_x * scale) + offset_x) * 40.95)))
    c_bounds[1] = max(0, min(4095, int(((max_x * scale) + offset_x) * 40.95)))
    c_bounds[2] = max(0, min(4095, int(((min_y * scale) + offset_y) * 40.95)))
    c_bounds[3] = max(0, min(4095, int(((max_y * scale) + offset_y) * 40.95)))
    
    # Generate Toolpath Preview
    preview_path = []
    last_px, last_py, last_laser, laser_enabled = -1, -1, -1, True
    for cmd in commands:
        if cmd[0] in ('G0', 'G1'):
            tx, ty = (cmd[1] * scale) + offset_x, (cmd[2] * scale) + offset_y
            dac_x, dac_y = max(0, min(4095, int(tx * 40.95))), max(0, min(4095, int(ty * 40.95)))
            is_laser = 1 if (laser_enabled and cmd[0] == 'G1') else 0
            if abs(dac_x - last_px) + abs(dac_y - last_py) > 10 or is_laser != last_laser:
                preview_path.append((dac_x, dac_y, is_laser))
                last_px, last_py, last_laser = dac_x, dac_y, is_laser
        elif cmd[0] == 'M3': laser_enabled = True
        elif cmd[0] == 'M5': laser_enabled = False

    preview_q.put({"path": preview_path, "bounds": {"min_x": c_bounds[0], "max_x": c_bounds[1], "min_y": c_bounds[2], "max_y": c_bounds[3]}})
    push_progress(50)

    # PHASE 2: Binary Packing
    pack = CMD_STRUCT.pack
    buffer = bytearray()
    curr_x, curr_y = 50.0, 50.0
    laser_enabled, current_s, current_f = True, max_s, 1000.0
    last_dac_x, last_dac_y = c_current_x.value, c_current_y.value
    ui_multiplier = pwm_max / max_s
    num_segments = 0
    total_cmds = len(commands) if len(commands) > 0 else 1

    for idx, cmd in enumerate(commands):
        if idx % 5000 == 0: push_progress(50 + int((idx / total_cmds) * 49))
        action = cmd[0]
        if action == 'M3': laser_enabled = True; current_s = cmd[1] if cmd[1] is not None else current_s
        elif action == 'M5': laser_enabled = False
        elif action == 'G4':
            buffer.extend(pack(last_dac_x, last_dac_y, 0, 0, int(cmd[1] * 1000000)))
            num_segments += 1
        elif action in ('G0', 'G1'):
            target_x, target_y = (cmd[1] * scale) + offset_x, (cmd[2] * scale) + offset_y
            if cmd[3] is not None: current_s = cmd[3]
            if cmd[4] is not None: current_f = cmd[4]

            dist = math.hypot(target_x - curr_x, target_y - curr_y)
            ns = max(1, int(dist * 5))
            
            is_cutting = 1 if (laser_enabled and action == 'G1') else 0
            pwm_val = max(0, min(255, int(current_s * ui_multiplier))) if is_cutting else 0
            
            spd = (current_f if is_cutting else 5000.0) / 60.0
            if spd <= 0: spd = 16.66
            
            # 10 Microsecond Hardware Limit Floor
            base_delay_us = int(max(0.00001, (dist / spd) / ns) * 1000000)
            
            if ns == 1:
                dac_x, dac_y = max(0, min(4095, int(target_x * 40.95))), max(0, min(4095, int(target_y * 40.95)))
                buffer.extend(pack(dac_x, dac_y, is_cutting, pwm_val, base_delay_us))
                last_dac_x, last_dac_y = dac_x, dac_y
            else:
                for step in range(1, ns + 1):
                    px, py = curr_x + (target_x - curr_x) * (step/ns), curr_y + (target_y - curr_y) * (step/ns)
                    dac_x, dac_y = max(0, min(4095, int(px * 40.95))), max(0, min(4095, int(py * 40.95)))
                    buffer.extend(pack(dac_x, dac_y, is_cutting, pwm_val, base_delay_us))
                    last_dac_x, last_dac_y = dac_x, dac_y
                    
            num_segments += ns
            curr_x, curr_y = target_x, target_y

    cmd_array = (GalvoCmd * num_segments)()
    ctypes.memmove(ctypes.addressof(cmd_array), bytes(buffer), len(buffer))
    push_progress(100)

    return cmd_array, num_segments

def run_frame():
    while c_status.value == 2:
        pts = [(c_bounds[0], c_bounds[2]), (c_bounds[1], c_bounds[2]), (c_bounds[1], c_bounds[3]), (c_bounds[0], c_bounds[3])]
        for i in range(4):
            if c_status.value != 2: break
            start, end = pts[i], pts[(i+1)%4]
            for step in range(1, 11):
                if c_status.value != 2: break
                cx = int(start[0] + (end[0]-start[0]) * (step/10.0))
                cy = int(start[1] + (end[1]-start[1]) * (step/10.0))
                set_dac_value(0, cx); set_dac_value(1, cy)
                c_current_x.value = cx; c_current_y.value = cy
                time.sleep(0.005)

def process_job(gcode_text, action, pwm_max, preview_q):
    c_status.value = 1; c_progress.value = 0
    cmd_array, num_cmds = build_c_array(gcode_text, pwm_max, preview_q, c_progress)
    
    if action == 'frame': c_status.value = 2; run_frame()
    elif num_cmds > 0 and galvo_c:
        c_status.value = 3; c_progress.value = 0
        try: spi.close() 
        except: pass
        
        # Elevate to Real-Time Scheduler
        try:
            param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
        except: pass

        galvo_c.run_trajectory(
            cmd_array, num_cmds, ctypes.byref(c_status.get_obj()), ctypes.byref(c_progress.get_obj()),
            ctypes.byref(c_current_x.get_obj()), ctypes.byref(c_current_y.get_obj()), ctypes.byref(c_current_pwm.get_obj()),
            ctypes.byref(c_speed_factor.get_obj()) 
        )
        
        # Restore Scheduler
        try: os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
        except: pass

        if c_status.value != 5: c_status.value = 6; c_progress.value = 100

def preprocess_gcode_to_text(gcode_text):
    commands = []
    append_cmd = commands.append
    curr_x, curr_y = 0.0, 0.0
    is_abs = True
    last_cmd = 'G0'
    min_x, max_x, min_y, max_y = float('inf'), float('-inf'), float('inf'), float('-inf')

    for line in gcode_text.split('\n'):
        original_line = line
        c_idx = line.find(';')
        if c_idx != -1: line = line[:c_idx]
        line = line.replace(' ', '').upper()
        if not line: append_cmd(("RAW", original_line)); continue
            
        parts = GCODE_TOKENIZER.findall(line)
        if not parts: continue

        c, v = parts[0]
        if c == 'G':
            if v.startswith('0') and len(v) > 1: v = v[1:] 
            cmd = c + v; last_cmd = cmd
        elif c != 'M' and c != 'F' and c != 'S': cmd = last_cmd; parts.insert(0, (cmd[0], cmd[1:])) 
        else: cmd = c + v
                
        if cmd == 'G90': is_abs = True; append_cmd(("RAW", original_line))
        elif cmd == 'G91': is_abs = False; append_cmd(("RAW", original_line))
        elif cmd in ('G0', 'G1'):
            target_x, target_y = curr_x, curr_y
            has_move, s_val, f_val = False, None, None
            for c2, v2 in parts[1:]:
                if not v2: continue
                val = float(v2)
                if c2 == 'X': target_x = val if is_abs else curr_x + val; has_move = True
                elif c2 == 'Y' or c2 == 'A': target_y = val if is_abs else curr_y + val; has_move = True
                elif c2 == 'S': s_val = str(val)
                elif c2 == 'F': f_val = str(val)

            if has_move:
                min_x, max_x = min(min_x, target_x), max(max_x, target_x)
                min_y, max_y = min(min_y, target_y), max(max_y, target_y)
                append_cmd((cmd, target_x, target_y, s_val, f_val))
                curr_x, curr_y = target_x, target_y
            else: append_cmd(("RAW", original_line))
        elif cmd in ('G2', 'G3'):
            target_x, target_y = curr_x, curr_y
            i, j, s_val, f_val = 0.0, 0.0, None, None
            for c2, v2 in parts[1:]:
                if not v2: continue
                val = float(v2)
                if c2 == 'X': target_x = val if is_abs else curr_x + val
                elif c2 == 'Y' or c2 == 'A': target_y = val if is_abs else curr_y + val
                elif c2 == 'I': i = val; elif c2 == 'J': j = val
                elif c2 == 'S': s_val = str(val)
                elif c2 == 'F': f_val = str(val)

            center_x, center_y = curr_x + i, curr_y + j
            radius = math.hypot(i, j)
            if radius < 0.001:
                append_cmd(("G1", target_x, target_y, s_val, f_val))
                curr_x, curr_y = target_x, target_y
                continue

            start_angle = math.atan2(curr_y - center_y, curr_x - center_x)
            end_angle = math.atan2(target_y - center_y, target_x - center_x)
            is_cw = (cmd == 'G2')
            if is_cw and end_angle > start_angle: end_angle -= 2 * math.pi
            elif not is_cw and end_angle < start_angle: end_angle += 2 * math.pi
            angle_diff = end_angle - start_angle
            num_segments = max(2, int(abs(angle_diff * radius) * 10))

            for step in range(1, num_segments):
                theta = start_angle + angle_diff * (step / num_segments)
                px, py = center_x + radius * math.cos(theta), center_y + radius * math.sin(theta)
                min_x, max_x = min(min_x, px), max(max_x, px)
                min_y, max_y = min(min_y, py), max(max_y, py)
                append_cmd(("G1", px, py, s_val if step == 1 else None, f_val if step == 1 else None))

            min_x, max_x = min(min_x, target_x), max(max_x, target_x)
            min_y, max_y = min(min_y, target_y), max(max_y, target_y)
            append_cmd(("G1", target_x, target_y, s_val if num_segments == 1 else None, f_val if num_segments == 1 else None))
            curr_x, curr_y = target_x, target_y
        else: append_cmd(("RAW", original_line))
            
    if min_x == float('inf'): return "\n".join([c[1] for c in commands if c[0] == "RAW"])
        
    range_x, range_y = max_x - min_x, max_y - min_y
    max_range = max(range_x, range_y)
    scale = 90.0 / max_range if max_range > 90.0 else 1.0
    center_x, center_y = (min_x + max_x) / 2.0, (min_y + max_y) / 2.0
    offset_x, offset_y = 50.0 - (center_x * scale), 50.0 - (center_y * scale)
    
    output_lines = ["; --- Auto-Centered & Interpolated ---", "G90"]
    curr_x, curr_y = 50.0, 50.0 
    
    for cmd in commands:
        if cmd[0] == "RAW": output_lines.append(cmd[1])
        elif cmd[0] in ('G0', 'G1'):
            target_x, target_y = (cmd[1] * scale) + offset_x, (cmd[2] * scale) + offset_y
            s_val = cmd[3] if len(cmd) > 3 else None
            f_val = cmd[4] if len(cmd) > 4 else None
            dist = math.hypot(target_x - curr_x, target_y - curr_y)
            num_segments = max(1, int(dist * 5)) 
            for step in range(1, num_segments + 1):
                px, py = curr_x + (target_x - curr_x) * (step/num_segments), curr_y + (target_y - curr_y) * (step/num_segments)
                out_cmd = f"{cmd[0]} X{px:.4f} Y{py:.4f}"
                if s_val and step == 1: out_cmd += f" S{s_val}"
                if f_val and step == 1: out_cmd += f" F{f_val}"
                output_lines.append(out_cmd)
            curr_x, curr_y = target_x, target_y
    return "\n".join(output_lines)

# --- WEB ROUTES ---
@app.route('/')
def index(): return render_template('index.html')

def emit_telemetry():
    last_state = None
    while True:
        time.sleep(0.05) 
        current_state = (c_current_x.value, c_current_y.value, c_current_pwm.value, c_status.value, c_progress.value)
        if current_state != last_state:
            last_state = current_state
            socketio.emit('telemetry', {
                "current_x": c_current_x.value, "current_y": c_current_y.value,
                "laser_on": c_current_pwm.value > 0, "current_pwm": c_current_pwm.value,
                "status": STATUS_MAP.get(c_status.value, "idle"), "progress": c_progress.value,
                "bounds": {"min_x": c_bounds[0], "max_x": c_bounds[1], "min_y": c_bounds[2], "max_y": c_bounds[3]}
            })

def preview_listener():
    while True:
        data = preview_queue.get()
        if data: socketio.emit('render_preview', data)

@socketio.on('jog_cmd')
def handle_jog(data):
    if c_status.value in [1, 2, 3]: return 
    x = max(0, min(4095, data.get('x', 2048)))
    y = max(0, min(4095, data.get('y', 2048)))
    set_dac_value(0, x); set_dac_value(1, y)
    c_current_x.value = x; c_current_y.value = y

@app.route('/settings', methods=['POST'])
def update_settings():
    data = request.json
    if 'speed' in data: c_speed_factor.value = float(data['speed']) / 100.0
    if 'pwm' in data: c_laser_pwm_max.value = max(0, min(254, int(data['pwm'])))
    return jsonify({"status": "success"})

@app.route('/upload', methods=['POST'])
def upload_gcode():
    if 'file' not in request.files: return jsonify({"error": "No file part"}), 400
    file = request.files['file']
    if file and (file.filename.endswith('.gcode') or file.filename.endswith('.nc')):
        p = Process(target=process_job, args=(file.read().decode('utf-8', errors='replace'), request.form.get('action', 'run'), c_laser_pwm_max.value, preview_queue))
        p.daemon = True; p.start()
        return jsonify({"status": "success", "message": "Job dispatched"})
    return jsonify({"error": "Invalid file type"}), 400

@app.route('/toggle_pause', methods=['POST'])
def toggle_pause():
    if c_status.value == 3: c_status.value = 4 
    elif c_status.value == 4: c_status.value = 3 
    return jsonify({"status": STATUS_MAP.get(c_status.value, "idle")})

@app.route('/stop', methods=['POST'])
def stop_galvo():
    c_status.value = 5; return jsonify({"status": "success"})

@app.route('/convert', methods=['POST'])
def convert_gcode():
    if 'file' not in request.files: return jsonify({"error": "No file part"}), 400
    file = request.files['file']
    if file and (file.filename.endswith('.gcode') or file.filename.endswith('.nc')):
        return jsonify({"status": "success", "converted_data": preprocess_gcode_to_text(file.read().decode('utf-8', errors='replace'))})
    return jsonify({"error": "Invalid file type"}), 400

if __name__ == '__main__':
    threading.Thread(target=emit_telemetry, daemon=True).start()
    threading.Thread(target=preview_listener, daemon=True).start()
    # allow_unsafe_werkzeug=True bypasses the strict production check while allowing us to keep debug off
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)
