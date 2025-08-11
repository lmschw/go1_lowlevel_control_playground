from robot_interface import LowCmd, LowState, MotorCmd, UDP, LeggedType, RecvEnum
import time

motors = {'FR_0':0, 'FR_1':1, 'FR_2':2,
        'FL_0':3, 'FL_1':4, 'FL_2':5, 
        'RR_0':6, 'RR_1':7, 'RR_2':8, 
        'RL_0':9, 'RL_1':10, 'RL_2':11 }

# 1. Setup
robot_type = LeggedType.Go1
udp = UDP(8082, 8081, 0, True, RecvEnum.block, False)  # Set to True for debug output

# 2. Create command and state containers
cmd = LowCmd()
state = LowState()

# 3. Configure basic header
cmd.head[0] = 0xFE
cmd.head[1] = 0xEF
cmd.levelFlag = 0x00  # 0 = low-level

# 4. Enable motor mode and set torque to zero (or desired values)
for i in range(12):
    motor = MotorCmd()
    motor.mode = 0x0A        # Motor ON
    motor.q = 0.0            # Position (not used)
    motor.dq = 0.0           # Velocity (not used)
    motor.Kp = 0.0           # Position gain
    motor.Kd = 0.0           # Velocity gain
    motor.tau = 0.5 if i == 2 else 0.0  # Torque on FL_hip as example
    cmd.motorCmd[i] = motor

# 5. Control loop
print("Sending torque commands. Press Ctrl+C to stop.")
motor_label = 'FR_0'
motor_id = motors[motor_label]
try:
    while True:
        motor = MotorCmd()
        motor.mode = 0x0A        # Motor ON
        motor.q = 0.0            # Position (not used)
        motor.dq = 0.0           # Velocity (not used)
        motor.Kp = 0.0           # Position gain
        motor.Kd = 0.0           # Velocity gain
        motor.tau = 2.5
        cmd.motorCmd[motor_id] = motor
        udp.SetSend(cmd)    # Set the command first
        udp.Send()          # Then send it
        udp.Recv()
        udp.GetRecv(state)

        # for motor in motors.keys():
        #     motor_state = state.motorState[motors[motor]]
        #     print(f"{motor}: Pos={motor_state.q:.3f}, Vel={motor_state.dq:.3f}, Tau={motor_state.tauEst:.3f}")
        
        motor_state = state.motorState[motor_id]
        print(f"{motor_label}: Pos={motor_state.q:.3f}, Vel={motor_state.dq:.3f}, Tau={motor_state.tauEst:.3f}")

        time.sleep(0.002)
except KeyboardInterrupt:
    print("Exiting...")
    udp.Close()  # Make sure to close cleanly
