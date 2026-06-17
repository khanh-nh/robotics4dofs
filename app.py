import streamlit as st
import serial
import serial.tools.list_ports
import time
import re

# --- Setup session state for serial connection ---
if 'serial_conn' not in st.session_state:
    st.session_state.serial_conn = None
if 'connected_port' not in st.session_state:
    st.session_state.connected_port = None
if 'held_tool' not in st.session_state:
    st.session_state.held_tool = "none"
if 'station_slot' not in st.session_state:
    st.session_state.station_slot = 0
if 'magnet_on' not in st.session_state:
    st.session_state.magnet_on = False
if 'tool_power_on' not in st.session_state:
    st.session_state.tool_power_on = False
if 'last_serial_response' not in st.session_state:
    st.session_state.last_serial_response = ""
if 'last_serial_command' not in st.session_state:
    st.session_state.last_serial_command = ""
if 'last_live_command' not in st.session_state:
    st.session_state.last_live_command = ""
if 'auto_status_refresh' not in st.session_state:
    st.session_state.auto_status_refresh = True
if 'last_status_poll_time' not in st.session_state:
    st.session_state.last_status_poll_time = 0.0
for joint_name in ["pitch", "roll", "yaw", "elbow"]:
    if f"{joint_name}_val" not in st.session_state:
        st.session_state[f"{joint_name}_val"] = 0.0
    if f"live_{joint_name}_val" not in st.session_state:
        st.session_state[f"live_{joint_name}_val"] = 0.0

# --- Helper functions ---
TOOL_SLOTS = {"hand": 0, "drill": 1, "gripper": 2}
SLOT_TOOLS = {0: "Hand", 1: "Drill", 2: "Gripper"}
ARM_REST_POSE = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0, "elbow": 50.0}
JOINT_LIMITS = {
    "pitch": ("Shoulder Pitch (S0)", -50.0, 130.0),
    "roll": ("Shoulder Roll (S1)", -20.0, 160.0),
    "yaw": ("Shoulder Yaw (S2)", -30.0, 150.0),
    "elbow": ("Elbow Pitch (S3)", -55.0, 125.0),
}
GEOMETRY_DEFAULTS = {
    "base": (0.0, 70.0, 515.0),
    "p01": (0.0, 36.0, -14.0),
    "p12": (0.0, 0.0, -85.0),
    "p23": (0.0, 29.0, -205.0),
    "p34": (0.0, 25.0, -140.0),
}

for point_name, coords in GEOMETRY_DEFAULTS.items():
    for axis, default_value in zip(["x", "y", "z"], coords):
        key = f"geom_{point_name}_{axis}"
        if key not in st.session_state:
            st.session_state[key] = default_value

TOOL_CHANGE_POSE_DEFAULTS = {
    "pitch": -3.0,
    "roll": 0.0,
    "yaw": 0.0,
    "elbow": -3.0,
}

for joint_name, default_value in TOOL_CHANGE_POSE_DEFAULTS.items():
    key = f"change_pose_{joint_name}"
    if key not in st.session_state:
        st.session_state[key] = default_value

def get_available_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def normalize_tool_name(tool):
    tool = tool.strip().lower()
    if tool in ["1", "gripper"]:
        return "gripper"
    if tool in ["2", "hand", "static", "model"]:
        return "hand"
    if tool in ["3", "drill", "dc", "motor"]:
        return "drill"
    if tool in ["0", "none"]:
        return "none"
    return None

def geometry_command(point_name):
    x = st.session_state[f"geom_{point_name}_x"]
    y = st.session_state[f"geom_{point_name}_y"]
    z = st.session_state[f"geom_{point_name}_z"]
    return f"geom {point_name} {x} {y} {z}"

def tool_change_pose_command():
    pitch = st.session_state.change_pose_pitch
    roll = st.session_state.change_pose_roll
    yaw = st.session_state.change_pose_yaw
    elbow = st.session_state.change_pose_elbow
    return f"changepose {pitch} {roll} {yaw} {elbow}"

def joint_offset_command(values):
    return f"q {values['pitch']} {values['roll']} {values['yaw']} {values['elbow']}"

def current_joint_values(prefix=""):
    return {
        "pitch": st.session_state[f"{prefix}pitch_val"],
        "roll": st.session_state[f"{prefix}roll_val"],
        "yaw": st.session_state[f"{prefix}yaw_val"],
        "elbow": st.session_state[f"{prefix}elbow_val"],
    }

def copy_current_joints_to_change_pose():
    st.session_state.change_pose_pitch = st.session_state.pitch_val
    st.session_state.change_pose_roll = st.session_state.roll_val
    st.session_state.change_pose_yaw = st.session_state.yaw_val
    st.session_state.change_pose_elbow = st.session_state.elbow_val

def reset_change_pose_defaults():
    for joint_name, default_value in TOOL_CHANGE_POSE_DEFAULTS.items():
        st.session_state[f"change_pose_{joint_name}"] = default_value

def read_serial_response(wait_time=0.35):
    conn = st.session_state.serial_conn
    if not conn or not conn.is_open:
        return ""

    time.sleep(wait_time)
    chunks = []
    while conn.in_waiting:
        chunks.append(conn.read(conn.in_waiting).decode("utf-8", errors="replace"))
        time.sleep(0.05)

    return "".join(chunks).strip()

def parse_status_response(response):
    if not response:
        return

    q_match = re.search(
        r"Current q\s*=\s*\[\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*\]",
        response,
        re.IGNORECASE,
    )
    if q_match:
        for joint_name, value in zip(["pitch", "roll", "yaw", "elbow"], q_match.groups()):
            st.session_state[f"{joint_name}_val"] = float(value)

    tool_match = re.search(r"Active tool:\s*(\w+)", response, re.IGNORECASE)
    if tool_match:
        tool = normalize_tool_name(tool_match.group(1))
        if tool:
            st.session_state.held_tool = tool

    slot_match = re.search(r"Tool station slot:\s*(-?\d+)", response, re.IGNORECASE)
    if slot_match:
        st.session_state.station_slot = int(slot_match.group(1)) % 3

    magnet_match = re.search(r"Magnet relay 1(?:\s+is|\s*:)\s*(ON|OFF)", response, re.IGNORECASE)
    if magnet_match:
        st.session_state.magnet_on = magnet_match.group(1).upper() == "ON"

    power_match = re.search(r"Tool power relay 2(?:\s+is|\s*:)\s*(ON|OFF)", response, re.IGNORECASE)
    if power_match:
        st.session_state.tool_power_on = power_match.group(1).upper() == "ON"

def request_status(silent=False):
    conn = st.session_state.serial_conn
    if not conn or not conn.is_open:
        return

    try:
        conn.reset_input_buffer()
        conn.write(b"status\n")
        response = read_serial_response(0.45)
        if response:
            parse_status_response(response)
            st.session_state.last_serial_response = response
            st.session_state.last_serial_command = "status"
            st.session_state.last_status_poll_time = time.time()
        elif not silent:
            st.toast("No status response received yet.")
    except Exception as e:
        if not silent:
            st.error(f"Error requesting status: {e}")
        disconnect_serial()

def remember_command_state(cmd):
    parts = cmd.strip().lower().split()
    if not parts:
        return

    if cmd in ["magnet on", "relay on"]:
        st.session_state.magnet_on = True
        return
    if cmd in ["magnet off", "relay off"]:
        st.session_state.magnet_on = False
        return
    if cmd == "toolpower on":
        st.session_state.tool_power_on = True
        return
    if cmd == "toolpower off":
        st.session_state.tool_power_on = False
        return

    if parts[0] == "station" and len(parts) >= 2:
        tool = normalize_tool_name(parts[1])
        if tool in TOOL_SLOTS:
            st.session_state.station_slot = TOOL_SLOTS[tool]
        elif parts[1].isdigit():
            st.session_state.station_slot = int(parts[1]) % 3
        return

    if parts[0] == "tool" and len(parts) >= 2:
        tool = normalize_tool_name(parts[1])
        if tool:
            st.session_state.held_tool = tool
            st.session_state.tool_power_on = tool not in ["none", "hand"]
        return

    if parts[0] == "pickup" and len(parts) >= 2:
        tool = normalize_tool_name(parts[1])
        if tool in TOOL_SLOTS and st.session_state.held_tool == "none":
            st.session_state.held_tool = tool
            st.session_state.station_slot = TOOL_SLOTS[tool]
            st.session_state.magnet_on = True
            st.session_state.tool_power_on = tool != "hand"
        return

    if parts[0] == "change" and len(parts) >= 2:
        target = normalize_tool_name(parts[-1])
        if target in TOOL_SLOTS:
            st.session_state.held_tool = target
            st.session_state.station_slot = TOOL_SLOTS[target]
            st.session_state.magnet_on = True
            st.session_state.tool_power_on = target != "hand"
        return

    if cmd in ["remove tool", "remove", "drop tool"]:
        if st.session_state.held_tool in TOOL_SLOTS:
            st.session_state.station_slot = TOOL_SLOTS[st.session_state.held_tool]
        st.session_state.held_tool = "none"
        st.session_state.magnet_on = False
        st.session_state.tool_power_on = False
        return

    if cmd == "tool":
        if st.session_state.held_tool in ["gripper", "hand", "drill"]:
            st.session_state.tool_power_on = st.session_state.held_tool != "hand"
        return

    if cmd == "rest":
        for joint_name, rest_value in ARM_REST_POSE.items():
            st.session_state[f"{joint_name}_val"] = rest_value
        return

    if parts[0] == "q" and len(parts) >= 5:
        try:
            st.session_state.pitch_val = float(parts[1])
            st.session_state.roll_val = float(parts[2])
            st.session_state.yaw_val = float(parts[3])
            st.session_state.elbow_val = float(parts[4])
        except ValueError:
            pass

def connect_serial(port, baudrate=115200):
    try:
        # Close existing connection if any
        if st.session_state.serial_conn and st.session_state.serial_conn.is_open:
            st.session_state.serial_conn.close()
            
        conn = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2) # Wait for ESP32 to reset after connection
        st.session_state.serial_conn = conn
        st.session_state.connected_port = port
        st.session_state.held_tool = "none"
        st.session_state.station_slot = 0
        st.session_state.magnet_on = False
        st.session_state.tool_power_on = False
        request_status(silent=True)
        st.success(f"Connected to {port} at {baudrate} baud")
    except Exception as e:
        st.error(f"Error connecting to {port}: {e}")

def disconnect_serial():
    if st.session_state.serial_conn and st.session_state.serial_conn.is_open:
        st.session_state.serial_conn.close()
    st.session_state.serial_conn = None
    st.session_state.connected_port = None
    st.success("Disconnected")

def send_command(cmd, expect_response=False):
    if st.session_state.serial_conn and st.session_state.serial_conn.is_open:
        try:
            st.session_state.serial_conn.reset_input_buffer()
            full_cmd = f"{cmd}\n"
            st.session_state.serial_conn.write(full_cmd.encode('utf-8'))
            remember_command_state(cmd)
            st.session_state.last_serial_command = cmd

            if expect_response:
                response = read_serial_response()
                st.session_state.last_serial_response = response or "(No response received yet.)"
                parse_status_response(response)
                st.toast(st.session_state.last_serial_response[:220])
            else:
                st.toast(f"Sent: {cmd}")
        except Exception as e:
            st.error(f"Error sending command: {e}")
            disconnect_serial()
    else:
        st.warning("Not connected to serial port. Please connect first.")

def status_card(label, value, state="neutral"):
    st.markdown(
        f"""
        <div class="status-card status-{state}">
          <div class="status-label">{label}</div>
          <div class="status-value">{value}</div>
        </div>
        """,
        unsafe_allow_html=True,
    )

def relay_toggle_changed(state_name, on_cmd, off_cmd, key):
    desired_state = st.session_state[key]
    if desired_state != st.session_state[state_name]:
        send_command(on_cmd if desired_state else off_cmd)

def relay_toggle(label, state_name, on_cmd, off_cmd, key):
    st.session_state[key] = st.session_state[state_name]

    st.toggle(
        label,
        key=key,
        on_change=relay_toggle_changed,
        args=(state_name, on_cmd, off_cmd, key),
    )

def render_system_status(auto_poll=False):
    if auto_poll and st.session_state.serial_conn and st.session_state.serial_conn.is_open:
        if st.session_state.auto_status_refresh and time.time() - st.session_state.last_status_poll_time > 1.5:
            request_status(silent=True)

    st.subheader("System Status")
    status_col1, status_col2, status_col3, status_col4 = st.columns(4)
    with status_col1:
        tool_state = "off" if st.session_state.held_tool == "none" else "ok"
        status_card("Held Tool", st.session_state.held_tool.title(), tool_state)
    with status_col2:
        slot_tool = SLOT_TOOLS.get(st.session_state.station_slot, "Unknown")
        status_card("Station Slot", f"{st.session_state.station_slot} - {slot_tool}")
    with status_col3:
        status_card("Magnet Relay 1", "ON" if st.session_state.magnet_on else "OFF", "ok" if st.session_state.magnet_on else "off")
    with status_col4:
        status_card("Tool Power Relay 2", "ON" if st.session_state.tool_power_on else "OFF", "warn" if st.session_state.tool_power_on else "off")

    status_actions = st.columns([1, 3])
    with status_actions[0]:
        if st.button("Sync Status", use_container_width=True):
            request_status()
    with status_actions[1]:
        st.caption("Status is parsed from the ESP32 `status` response when synced or auto-refreshed.")

    if st.session_state.last_serial_response:
        with st.expander("Firmware Output", expanded=True):
            st.caption(f"Last command: {st.session_state.last_serial_command}")
            st.code(st.session_state.last_serial_response)

# --- UI Layout ---
st.set_page_config(page_title="Humanoid Arm Control", layout="wide")
st.markdown(
    """
    <style>
      .block-container {
        padding-top: 1.5rem;
        padding-bottom: 2rem;
        max-width: 1380px;
      }
      h1, h2, h3 {
        letter-spacing: 0;
      }
      h1 {
        display: none;
      }
      h2 {
        font-size: 1.25rem;
      }
      h3 {
        font-size: 1rem;
      }
      .status-card {
        border: 1px solid #d8dee8;
        border-left-width: 5px;
        border-radius: 8px;
        padding: 0.78rem 0.9rem;
        background: #ffffff;
        min-height: 82px;
      }
      .status-label {
        color: #5f6b7a;
        font-size: 0.78rem;
        font-weight: 600;
        text-transform: uppercase;
      }
      .status-value {
        color: #172033;
        font-size: 1.45rem;
        font-weight: 700;
        line-height: 1.2;
        margin-top: 0.35rem;
        word-break: break-word;
      }
      .status-ok {
        border-left-color: #1f9d66;
      }
      .status-warn {
        border-left-color: #d98c00;
      }
      .status-off {
        border-left-color: #8a94a6;
      }
      .status-neutral {
        border-left-color: #3772ff;
      }
      .section-note {
        color: #5f6b7a;
        margin-top: -0.4rem;
        margin-bottom: 0.8rem;
      }
      .app-title {
        color: #172033;
        font-size: 2rem;
        font-weight: 750;
        line-height: 1.2;
        margin-bottom: 0.35rem;
      }
      div.stButton > button {
        border-radius: 8px;
        min-height: 2.55rem;
        font-weight: 600;
      }
      div[data-testid="stExpander"] {
        border: 1px solid #d8dee8;
        border-radius: 8px;
      }
    </style>
    """,
    unsafe_allow_html=True,
)
st.title("4-DOF Humanoid Arm Controller")

st.markdown('<div class="app-title">4-DOF Humanoid Arm Controller</div>', unsafe_allow_html=True)
st.markdown('<div class="section-note">Operator panel for arm motion, tool power, and rotary tool exchange.</div>', unsafe_allow_html=True)

if hasattr(st, "fragment"):
    @st.fragment(run_every="2s")
    def system_status_fragment():
        render_system_status(auto_poll=True)

    system_status_fragment()
else:
    render_system_status(auto_poll=True)

with st.expander("Robot Geometry Config"):
    st.caption("Runtime FK/IK geometry in millimeters. Changes are sent to the ESP32 and remain active until reset or reboot.")

    for point_name in ["base", "p01", "p12", "p23", "p34"]:
        cols = st.columns([1.1, 1, 1, 1, 1])
        with cols[0]:
            st.write(point_name)
        with cols[1]:
            st.number_input("X", key=f"geom_{point_name}_x", step=1.0, label_visibility="collapsed")
        with cols[2]:
            st.number_input("Y", key=f"geom_{point_name}_y", step=1.0, label_visibility="collapsed")
        with cols[3]:
            st.number_input("Z", key=f"geom_{point_name}_z", step=1.0, label_visibility="collapsed")
        with cols[4]:
            if st.button("Send", key=f"send_geom_{point_name}", use_container_width=True):
                send_command(geometry_command(point_name), expect_response=True)

    cfg_col1, cfg_col2 = st.columns(2)
    with cfg_col1:
        if st.button("Send All Geometry", use_container_width=True):
            for name in ["base", "p01", "p12", "p23", "p34"]:
                send_command(geometry_command(name), expect_response=True)
    with cfg_col2:
        if st.button("Print Geometry", use_container_width=True):
            send_command("geom", expect_response=True)

with st.expander("Tool Exchange Position"):
    st.caption("Joint-offset pose in degrees used before pickup, remove, and tool-to-tool changes.")

    pose_cols = st.columns(4)
    with pose_cols[0]:
        st.number_input("Pitch", min_value=-50.0, max_value=130.0, step=1.0, key="change_pose_pitch")
    with pose_cols[1]:
        st.number_input("Roll", min_value=-20.0, max_value=160.0, step=1.0, key="change_pose_roll")
    with pose_cols[2]:
        st.number_input("Yaw", min_value=-30.0, max_value=150.0, step=1.0, key="change_pose_yaw")
    with pose_cols[3]:
        st.number_input("Elbow", min_value=-55.0, max_value=125.0, step=1.0, key="change_pose_elbow")

    pose_btn1, pose_btn2, pose_btn3 = st.columns(3)
    with pose_btn1:
        if st.button("Save Exchange Pose", use_container_width=True):
            send_command(tool_change_pose_command(), expect_response=True)
    with pose_btn2:
        if st.button("Move To Exchange Pose", use_container_width=True):
            send_command("goto changepose")
    with pose_btn3:
        if st.button("Print Exchange Pose", use_container_width=True):
            send_command("changepose", expect_response=True)

    pose_btn4, pose_btn5 = st.columns(2)
    with pose_btn4:
        if st.button("Use Current Joint Pose", use_container_width=True):
            copy_current_joints_to_change_pose()
            send_command(tool_change_pose_command(), expect_response=True)
    with pose_btn5:
        if st.button("Reset Exchange Pose", use_container_width=True):
            reset_change_pose_defaults()
            send_command(tool_change_pose_command(), expect_response=True)

# Sidebar for connection
with st.sidebar:
    st.header("Connection Settings")
    
    # Auto-refresh ports logic could be implemented, for now get on load
    available_ports = get_available_ports()
    
    selected_port = st.selectbox("Select COM Port", available_ports)
    baud_rate = st.selectbox("Baud Rate", [115200, 9600, 19200, 38400, 57600], index=0)
    
    col1, col2 = st.columns(2)
    with col1:
        if st.button("Connect", use_container_width=True):
            if selected_port:
                connect_serial(selected_port, baud_rate)
            else:
                st.warning("No COM port available/selected")
    with col2:
        if st.button("Disconnect", use_container_width=True):
            disconnect_serial()
            
    if st.session_state.connected_port:
        st.success(f"Status: Connected to {st.session_state.connected_port}")
    else:
        st.error("Status: Disconnected")

    st.session_state.auto_status_refresh = st.checkbox(
        "Auto-sync status",
        value=st.session_state.auto_status_refresh,
        help="Poll the ESP32 status every 2 seconds while connected.",
    )
        
    st.divider()
    st.header("Arm Settings")
    speed = st.slider("Motion Speed (ms delay)", min_value=5, max_value=60, value=20, help="Smaller value = faster motion")
    if st.button("Update Speed"):
        send_command(f"speed {speed}")

    st.divider()
    st.header("Tool Lock / Power")
    st.caption("Relay 1 = magnet lock. Relay 2 = pogo VCC for the current tool.")
    relay_toggle("Magnet", "magnet_on", "magnet on", "magnet off", "sidebar_magnet_toggle")
    relay_toggle("Tool Power", "tool_power_on", "toolpower on", "toolpower off", "sidebar_power_toggle")

    st.divider()
    st.header("Active Tool")
    selected_tool = st.selectbox("Tool", ["gripper", "hand", "drill", "none"])
    if st.button("Set Active Tool", use_container_width=True):
        send_command(f"tool {selected_tool}")

# Main content tabs
tab1, tab2, tab3, tab4, tab5, tab6 = st.tabs(["Preset Motions", "Cartesian Control (IK)", "Joint Control (FK)", "Live Demo", "Tool Station", "Stepper Motor"])

with tab1:
    st.header("Preset Poses & Motions")
    
    st.subheader("Basic Poses")
    c1, c2, c3, c4 = st.columns(4)
    with c1:
        if st.button("Rest Pose", use_container_width=True): send_command("rest")
    with c2:
        if st.button("Raise", use_container_width=True): send_command("raise")
    with c3:
        if st.button("Side", use_container_width=True): send_command("side")
    with c4:
        if st.button("Bend", use_container_width=True): send_command("bend")
        
    st.subheader("Complex Motions")
    c5, c6, c7, c8 = st.columns(4)
    with c5:
        if st.button("Wave", use_container_width=True): send_command("wave")
    with c6:
        if st.button("Hello", use_container_width=True): send_command("hello")
    with c7:
        if st.button("Handshake", use_container_width=True): send_command("handshake")
    with c8:
        if st.button("Stroke", use_container_width=True): send_command("stroke")
        
    c9, c10, c11, c12 = st.columns(4)
    with c9:
        if st.button("Pick & Place", use_container_width=True): send_command("pickplace")
    with c10:
        if st.button("Front Pick", use_container_width=True): send_command("frontpick")
    with c11:
        if st.button("Run Active Tool", use_container_width=True): send_command("tool")

    with c12:
        if st.button("Tool Status", use_container_width=True): send_command("status", expect_response=True)

    if st.button("Tool Showcase Sequence", type="primary", use_container_width=True):
        send_command("showcase")

    st.subheader("Gripper")
    cg1, cg2 = st.columns(2)
    with cg1:
        if st.button("Gripper Open", use_container_width=True): send_command("gripper open")
    with cg2:
        if st.button("Gripper Close", use_container_width=True): send_command("gripper close")

with tab2:
    st.header("Cartesian Coordinate Control (IK)")
    st.write("Send XYZ coordinates (mm) for the end effector to reach.")
    
    colx, coly, colz = st.columns(3)
    with colx:
        x_val = st.number_input("X (Forward/Back)", value=120.0, step=10.0)
    with coly:
        y_val = st.number_input("Y (Left/Right)", value=160.0, step=10.0)
    with colz:
        z_val = st.number_input("Z (Up/Down)", value=250.0, step=10.0)
        
    if st.button("Go to XYZ", type="primary"):
        send_command(f"go {x_val} {y_val} {z_val}")

with tab3:
    st.header("Joint Offset Control")
    st.write("Control joint angles relative to rest pose. Rest is 0 degrees.")

    use_number_inputs = st.checkbox("Use number input instead of sliders")

    def joint_command_values(override_joint=None, override_value=0.0):
        values = {
            "pitch": st.session_state.pitch_val,
            "roll": st.session_state.roll_val,
            "yaw": st.session_state.yaw_val,
            "elbow": st.session_state.elbow_val,
        }
        if override_joint:
            values[override_joint] = override_value
        return values

    def send_joint_offsets(values):
        send_command(f"q {values['pitch']} {values['roll']} {values['yaw']} {values['elbow']}")

    colp, colr, colw, cole = st.columns(4)
    for col, joint in zip([colp, colr, colw, cole], ["pitch", "roll", "yaw", "elbow"]):
        label, min_value, max_value = JOINT_LIMITS[joint]
        with col:
            if use_number_inputs:
                joint_value = st.number_input(
                    label,
                    min_value=min_value,
                    max_value=max_value,
                    value=st.session_state[f"{joint}_val"],
                    step=1.0,
                )
            else:
                joint_value = st.slider(
                    label,
                    min_value=min_value,
                    max_value=max_value,
                    value=st.session_state[f"{joint}_val"],
                    step=1.0,
                )
            st.session_state[f"{joint}_val"] = joint_value

            if st.button("Rest", key=f"{joint}_rest", use_container_width=True):
                st.session_state[f"{joint}_val"] = 0.0
                send_joint_offsets(joint_command_values(joint, 0.0))

    c_apply, c_rest_all = st.columns(2)
    with c_apply:
        if st.button("Set Joint Offsets", type="primary", use_container_width=True):
            send_joint_offsets(joint_command_values())
    with c_rest_all:
        if st.button("Rest All Joints", use_container_width=True):
            for joint_name, rest_value in ARM_REST_POSE.items():
                st.session_state[f"{joint_name}_val"] = rest_value
            send_command("rest")

    st.divider()
    if st.button("Run Arm Servo Diagnostic", use_container_width=True):
        send_command("servotest", expect_response=True)

with tab4:
    st.header("Live Demo")
    st.write("Move joint sliders and send joint offsets immediately while live mode is enabled.")

    live_enabled = st.checkbox("Enable live joint control")
    live_cols = st.columns(4)

    for col, joint in zip(live_cols, ["pitch", "roll", "yaw", "elbow"]):
        label, min_value, max_value = JOINT_LIMITS[joint]
        with col:
            live_value = st.slider(
                label,
                min_value=min_value,
                max_value=max_value,
                value=st.session_state[f"live_{joint}_val"],
                step=1.0,
                key=f"live_slider_{joint}",
            )
            st.session_state[f"live_{joint}_val"] = live_value

    live_values = current_joint_values("live_")
    live_command = joint_offset_command(live_values)

    live_btn1, live_btn2, live_btn3 = st.columns(3)
    with live_btn1:
        if st.button("Sync From Joint Tab", use_container_width=True):
            for name in ["pitch", "roll", "yaw", "elbow"]:
                st.session_state[f"live_{name}_val"] = st.session_state[f"{name}_val"]
    with live_btn2:
        if st.button("Rest Live Pose", use_container_width=True):
            for name, rest_value in ARM_REST_POSE.items():
                st.session_state[f"live_{name}_val"] = rest_value
            send_command("rest")
            st.session_state.last_live_command = "rest"
    with live_btn3:
        if st.button("Send Current Live Pose", type="primary", use_container_width=True):
            send_command(live_command)
            st.session_state.last_live_command = live_command

    if live_enabled:
        if live_command != st.session_state.last_live_command:
            send_command(live_command)
            st.session_state.last_live_command = live_command
        st.caption(f"Live command: {live_command}")
    else:
        st.caption("Live mode is disabled. Adjust sliders without moving the arm, then enable live mode or send manually.")

with tab5:
    st.header("Tool Exchange Station")
    st.write("The station has 3 tools on a 360 degree plate. Each slot is 120 degrees apart.")

    st.subheader("Automatic Tool Change")
    st.write("The robot remembers the held tool. At startup it assumes no tool is held.")

    ca1, ca2, ca3 = st.columns(3)
    with ca1:
        if st.button("Change/Pick Gripper", use_container_width=True):
            send_command("change gripper")
    with ca2:
        if st.button("Change/Pick Hand", use_container_width=True):
            send_command("change hand")
    with ca3:
        if st.button("Change/Pick Drill", use_container_width=True):
            send_command("change drill")

    ci1, ci2, ci3, ci4 = st.columns(4)
    with ci1:
        if st.button("Initial Pickup Gripper", use_container_width=True):
            send_command("pickup gripper")
    with ci2:
        if st.button("Initial Pickup Hand", use_container_width=True):
            send_command("pickup hand")
    with ci3:
        if st.button("Initial Pickup Drill", use_container_width=True):
            send_command("pickup drill")
    with ci4:
        if st.button("Remove Held Tool", use_container_width=True):
            send_command("remove tool")

    st.divider()
    st.subheader("Manual Station Rotation")

    cs1, cs2, cs3 = st.columns(3)
    with cs1:
        if st.button("Slot 0: Hand", use_container_width=True):
            send_command("station hand")
    with cs2:
        if st.button("Slot 1: Drill", use_container_width=True):
            send_command("station drill")
    with cs3:
        if st.button("Slot 2: Gripper", use_container_width=True):
            send_command("station gripper")

    station_slot = st.number_input("Station slot", min_value=0, max_value=2, value=0, step=1)
    if st.button("Rotate to Slot", type="primary", use_container_width=True):
        send_command(f"station {station_slot}")

    st.divider()
    st.write("Manual relay control for tool pickup/dropoff.")
    relay_toggle("Magnet", "magnet_on", "magnet on", "magnet off", "station_magnet_toggle")
    relay_toggle("Tool Power", "tool_power_on", "toolpower on", "toolpower off", "station_power_toggle")

with tab6:
    st.header("Stepper Motor Control")
    st.write("Control the separated stepper motor (A4988 driver).")
    
    stepper_deg = st.number_input("Degrees to move", min_value=-3600.0, max_value=3600.0, value=360.0, step=10.0, help="Positive for forward, negative for backward")
    
    if st.button("Move Stepper", type="primary", use_container_width=True):
        send_command(f"stepper {stepper_deg}")
        
    st.divider()

    col_step1, col_step2 = st.columns(2)
    with col_step1:
        if st.button("Move +120 deg (Forward)", use_container_width=True):
            send_command("stepper 120")
    with col_step2:
        if st.button("Move -120 deg (Backward)", use_container_width=True):
            send_command("stepper -120")

st.divider()
if st.button("Request Arm Status"):
    send_command("status", expect_response=True)
