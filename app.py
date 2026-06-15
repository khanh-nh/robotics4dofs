import streamlit as st
import serial
import serial.tools.list_ports
import time

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
for joint_name in ["pitch", "roll", "yaw", "elbow"]:
    if f"{joint_name}_val" not in st.session_state:
        st.session_state[f"{joint_name}_val"] = 0.0

# --- Helper functions ---
TOOL_SLOTS = {"gripper": 0, "vacuum": 1, "pump": 1, "drill": 2}
SLOT_TOOLS = {0: "Gripper", 1: "Vacuum", 2: "Drill"}

def get_available_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def normalize_tool_name(tool):
    tool = tool.strip().lower()
    if tool in ["1", "gripper"]:
        return "gripper"
    if tool in ["2", "vacuum", "pump"]:
        return "vacuum"
    if tool in ["3", "drill", "dc", "motor"]:
        return "drill"
    if tool in ["0", "none"]:
        return "none"
    return None

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
            st.session_state.tool_power_on = tool != "none"
        return

    if parts[0] == "pickup" and len(parts) >= 2:
        tool = normalize_tool_name(parts[1])
        if tool in TOOL_SLOTS and st.session_state.held_tool == "none":
            st.session_state.held_tool = tool
            st.session_state.station_slot = TOOL_SLOTS[tool]
            st.session_state.magnet_on = True
            st.session_state.tool_power_on = True
        return

    if parts[0] == "change" and len(parts) >= 2:
        target = normalize_tool_name(parts[-1])
        if target in TOOL_SLOTS:
            st.session_state.held_tool = target
            st.session_state.station_slot = TOOL_SLOTS[target]
            st.session_state.magnet_on = True
            st.session_state.tool_power_on = True
        return

    if cmd in ["remove tool", "remove", "drop tool"]:
        if st.session_state.held_tool in TOOL_SLOTS:
            st.session_state.station_slot = TOOL_SLOTS[st.session_state.held_tool]
        st.session_state.held_tool = "none"
        st.session_state.magnet_on = False
        st.session_state.tool_power_on = False
        return

    if cmd == "tool":
        if st.session_state.held_tool in ["gripper", "vacuum", "drill"]:
            st.session_state.tool_power_on = True
        return

    if cmd == "rest":
        st.session_state.pitch_val = 0.0
        st.session_state.roll_val = 0.0
        st.session_state.yaw_val = 0.0
        st.session_state.elbow_val = 0.0
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
        st.success(f"Connected to {port} at {baudrate} baud")
    except Exception as e:
        st.error(f"Error connecting to {port}: {e}")

def disconnect_serial():
    if st.session_state.serial_conn and st.session_state.serial_conn.is_open:
        st.session_state.serial_conn.close()
    st.session_state.serial_conn = None
    st.session_state.connected_port = None
    st.success("Disconnected")

def send_command(cmd):
    if st.session_state.serial_conn and st.session_state.serial_conn.is_open:
        try:
            full_cmd = f"{cmd}\n"
            st.session_state.serial_conn.write(full_cmd.encode('utf-8'))
            remember_command_state(cmd)
            st.toast(f"Sent: {cmd}")
        except Exception as e:
            st.error(f"Error sending command: {e}")
            disconnect_serial()
    else:
        st.warning("Not connected to serial port. Please connect first.")

# --- UI Layout ---
st.set_page_config(page_title="Humanoid Arm Control", layout="wide")
st.title("🦾 4-DOF Humanoid Arm Controller")

st.subheader("Current Tool Status")
status_col1, status_col2, status_col3, status_col4 = st.columns(4)
with status_col1:
    st.metric("Held Tool", st.session_state.held_tool.title())
with status_col2:
    slot_tool = SLOT_TOOLS.get(st.session_state.station_slot, "Unknown")
    st.metric("Station Slot", f"{st.session_state.station_slot} - {slot_tool}")
with status_col3:
    st.metric("Magnet Relay 1", "ON" if st.session_state.magnet_on else "OFF")
with status_col4:
    st.metric("Tool Power Relay 2", "ON" if st.session_state.tool_power_on else "OFF")
st.caption("Status is tracked by this app after commands it sends. Use the firmware status command for serial monitor confirmation.")

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
        
    st.divider()
    st.header("Arm Settings")
    speed = st.slider("Motion Speed (ms delay)", min_value=5, max_value=60, value=20, help="Smaller value = faster motion")
    if st.button("Update Speed"):
        send_command(f"speed {speed}")

    st.divider()
    st.header("Tool Lock / Power")
    st.caption("Relay 1 = magnet lock. Relay 2 = pogo VCC for the current tool.")
    col_relay1, col_relay2 = st.columns(2)
    with col_relay1:
        if st.button("Magnet ON", use_container_width=True):
            send_command("magnet on")
        if st.button("Power ON", use_container_width=True):
            send_command("toolpower on")
    with col_relay2:
        if st.button("Magnet OFF", use_container_width=True):
            send_command("magnet off")
        if st.button("Power OFF", use_container_width=True):
            send_command("toolpower off")

    st.divider()
    st.header("Active Tool")
    selected_tool = st.selectbox("Tool", ["gripper", "vacuum", "drill", "none"])
    if st.button("Set Active Tool", use_container_width=True):
        send_command(f"tool {selected_tool}")

# Main content tabs
tab1, tab2, tab3, tab4, tab5 = st.tabs(["Preset Motions", "Cartesian Control (IK)", "Joint Control (FK)", "Tool Station", "Stepper Motor"])

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
        if st.button("Wave 👋", use_container_width=True): send_command("wave")
    with c6:
        if st.button("Hello 🖐", use_container_width=True): send_command("hello")
    with c7:
        if st.button("Handshake 🤝", use_container_width=True): send_command("handshake")
    with c8:
        if st.button("Stroke 🐈", use_container_width=True): send_command("stroke")
        
    c9, c10, c11, c12 = st.columns(4)
    with c9:
        if st.button("Pick & Place 📦", use_container_width=True): send_command("pickplace")
    with c10:
        if st.button("Front Pick 🔽", use_container_width=True): send_command("frontpick")
    with c11:
        if st.button("Run Active Tool", use_container_width=True): send_command("tool")

    with c12:
        if st.button("Tool Status", use_container_width=True): send_command("status")

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

    joint_limits = {
        "pitch": ("Shoulder Pitch (S0)", -50.0, 130.0),
        "roll": ("Shoulder Roll (S1)", 0.0, 160.0),
        "yaw": ("Shoulder Yaw (S2)", -30.0, 150.0),
        "elbow": ("Elbow Pitch (S3)", -55.0, 125.0),
    }

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
        label, min_value, max_value = joint_limits[joint]
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
            st.session_state.pitch_val = 0.0
            st.session_state.roll_val = 0.0
            st.session_state.yaw_val = 0.0
            st.session_state.elbow_val = 0.0
            send_command("rest")

with tab4:
    st.header("Tool Exchange Station")
    st.write("The station has 3 tools on a 360 degree plate. Each slot is 120 degrees apart.")

    st.subheader("Automatic Tool Change")
    st.write("The robot remembers the held tool. At startup it assumes no tool is held.")

    ca1, ca2, ca3 = st.columns(3)
    with ca1:
        if st.button("Change/Pick Gripper", use_container_width=True):
            send_command("change gripper")
    with ca2:
        if st.button("Change/Pick Vacuum", use_container_width=True):
            send_command("change vacuum")
    with ca3:
        if st.button("Change/Pick Drill", use_container_width=True):
            send_command("change drill")

    ci1, ci2, ci3, ci4 = st.columns(4)
    with ci1:
        if st.button("Initial Pickup Gripper", use_container_width=True):
            send_command("pickup gripper")
    with ci2:
        if st.button("Initial Pickup Vacuum", use_container_width=True):
            send_command("pickup vacuum")
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
        if st.button("Slot 0: Gripper", use_container_width=True):
            send_command("station gripper")
    with cs2:
        if st.button("Slot 1: Vacuum", use_container_width=True):
            send_command("station vacuum")
    with cs3:
        if st.button("Slot 2: Drill", use_container_width=True):
            send_command("station drill")

    station_slot = st.number_input("Station slot", min_value=0, max_value=2, value=0, step=1)
    if st.button("Rotate to Slot", type="primary", use_container_width=True):
        send_command(f"station {station_slot}")

    st.divider()
    st.write("Manual relay control for tool pickup/dropoff.")
    cm1, cm2 = st.columns(2)
    with cm1:
        if st.button("Attach Tool: Magnet ON", use_container_width=True):
            send_command("magnet on")
    with cm2:
        if st.button("Release Tool: Magnet OFF", use_container_width=True):
            send_command("magnet off")

with tab5:
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
    send_command("status")
    st.info("Status command sent to ESP32. (Check physical serial monitor for output, or expand this UI later to read bi-directional serial).")
