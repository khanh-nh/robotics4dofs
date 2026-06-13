import streamlit as st
import serial
import serial.tools.list_ports
import time

# --- Setup session state for serial connection ---
if 'serial_conn' not in st.session_state:
    st.session_state.serial_conn = None
if 'connected_port' not in st.session_state:
    st.session_state.connected_port = None

# --- Helper functions ---
def get_available_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def connect_serial(port, baudrate=115200):
    try:
        # Close existing connection if any
        if st.session_state.serial_conn and st.session_state.serial_conn.is_open:
            st.session_state.serial_conn.close()
            
        conn = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2) # Wait for ESP32 to reset after connection
        st.session_state.serial_conn = conn
        st.session_state.connected_port = port
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
            st.toast(f"Sent: {cmd}")
        except Exception as e:
            st.error(f"Error sending command: {e}")
            disconnect_serial()
    else:
        st.warning("Not connected to serial port. Please connect first.")

# --- UI Layout ---
st.set_page_config(page_title="Humanoid Arm Control", layout="wide")
st.title("🦾 4-DOF Humanoid Arm Controller")

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

# Main content tabs
tab1, tab2, tab3, tab4 = st.tabs(["Preset Motions", "Cartesian Control (IK)", "Joint Control (FK)", "Stepper Motor"])

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
        
    c9, c10, c11, _ = st.columns(4)
    with c9:
        if st.button("Pick & Place 📦", use_container_width=True): send_command("pickplace")
    with c10:
        if st.button("Front Pick 🔽", use_container_width=True): send_command("frontpick")
    with c11:
        if st.button("Toggle Tool/Gripper 🔧", use_container_width=True): send_command("tool")

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
    st.header("Joint Angle Control (FK)")
    st.write("Directly control the angles of each joint.")
    
    # Initialize session state for joints
    joint_defaults = {'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0, 'elbow': 0.0}
    for j, default in joint_defaults.items():
        if f'{j}_val' not in st.session_state:
            st.session_state[f'{j}_val'] = default

    def update_joint(joint, source):
        st.session_state[f'{joint}_val'] = st.session_state[f'{joint}_{source}']
    
    colp, colr, colw, cole = st.columns(4)
    with colp:
        st.number_input("Pitch (S0)", min_value=-50.0, max_value=130.0, value=st.session_state.pitch_val, step=1.0, key="pitch_num", on_change=update_joint, args=("pitch", "num"))
        st.slider("Pitch Slider", min_value=-50.0, max_value=130.0, value=st.session_state.pitch_val, step=1.0, key="pitch_slider", label_visibility="collapsed", on_change=update_joint, args=("pitch", "slider"))
        pitch = st.session_state.pitch_val
    with colr:
        st.number_input("Roll (S1)", min_value=0.0, max_value=160.0, value=st.session_state.roll_val, step=1.0, key="roll_num", on_change=update_joint, args=("roll", "num"))
        st.slider("Roll Slider", min_value=0.0, max_value=160.0, value=st.session_state.roll_val, step=1.0, key="roll_slider", label_visibility="collapsed", on_change=update_joint, args=("roll", "slider"))
        roll = st.session_state.roll_val
    with colw:
        st.number_input("Yaw (S2)", min_value=-30.0, max_value=150.0, value=st.session_state.yaw_val, step=1.0, key="yaw_num", on_change=update_joint, args=("yaw", "num"))
        st.slider("Yaw Slider", min_value=-30.0, max_value=150.0, value=st.session_state.yaw_val, step=1.0, key="yaw_slider", label_visibility="collapsed", on_change=update_joint, args=("yaw", "slider"))
        yaw = st.session_state.yaw_val
    with cole:
        st.number_input("Elbow (S3)", min_value=-55.0, max_value=125.0, value=st.session_state.elbow_val, step=1.0, key="elbow_num", on_change=update_joint, args=("elbow", "num"))
        st.slider("Elbow Slider", min_value=-55.0, max_value=125.0, value=st.session_state.elbow_val, step=1.0, key="elbow_slider", label_visibility="collapsed", on_change=update_joint, args=("elbow", "slider"))
        elbow = st.session_state.elbow_val
        
    if st.button("Set Joint Angles", type="primary"):
        send_command(f"q {pitch} {roll} {yaw} {elbow}")

with tab4:
    st.header("Stepper Motor Control")
    st.write("Control the separated stepper motor (A4988 driver).")
    
    col_step1, col_step2 = st.columns(2)
    with col_step1:
        if st.button("Move 120 deg", use_container_width=True):
            send_command("stepper 120")
    with col_step2:
        if st.button("Move 240 deg", use_container_width=True):
            send_command("stepper 240")

st.divider()
if st.button("Request Arm Status"):
    send_command("status")
    st.info("Status command sent to ESP32. (Check physical serial monitor for output, or expand this UI later to read bi-directional serial).")
