import time
import csv
import math
import sys
from collections import deque
sys.path.append('/home/lilly/robot_shoe/unitree_legged_sdk/lib/python/amd64/')
import robot_interface as sdk

# PARAMETERS - tune these conservatively
DT = 0.002                    # control loop target (s)
INTERP_SECONDS = 10.0         # very slow standup
Kp_START = 4.0
Kp_MAX = 22.0
KD = 1.0
MAX_STEP = 0.007              # radians per control step (very small)
IQ_LIMIT = 3.5                # motor current threshold (A) -> ADJUST to safe low value for Go1
LOOP_JITTER_LIMIT = 0.02      # if loop falls behind more than this (s), abort
ERR_ABORT = True              # abort if motorState[i].err != 0
LOG_FILENAME = "standup_telemetry.csv"
SIGNMAP =  [1, 1, -1, -1, 1, -1, 1, 1, -1, -1, -1, -1]
MOTOR_IDS = {'FR_0': 0, 'FR_1': 1, 'FR_2': 2,
		'FL_0': 3, 'FL_1': 4, 'FL_2': 5,
		'RR_0': 6, 'RR_1': 7, 'RR_2': 8,
		'RL_0': 9, 'RL_1': 10, 'RL_2': 11}

POS_STOP_F = math.pow(10, 9)
VEL_STOP_F = 16000.0

HIGHLEVEL = 0xee
LOWLEVEL = 0xff

def getJointAngles(csv_filename):
	with open(csv_filename, 'r') as csvfile:
		reader = csv.reader(csvfile)
		row = next(reader)  # 读取第一行数据

	joint_angles = [float(angle) for angle in row]
	print(f"joint angles: {joint_angles}")
	return joint_angles


def safe_get(attr_obj, name, default=0.0):
    return getattr(attr_obj, name, default)

def run_safe_stand(udp, cmd, state, joint_angles_target, sign_map):
    interp_steps = int(max(1, INTERP_SECONDS / DT))

    # Read initial state
    udp.Recv(); udp.GetRecv(state)
    qInit = [state.motorState[i].q for i in range(12)]

    # Seed commands to avoid initial catch-up
    for i in range(12):
        cmd.motorCmd[i].q = qInit[i]
        cmd.motorCmd[i].dq = 0.0
        cmd.motorCmd[i].Kp = Kp_START
        cmd.motorCmd[i].Kd = KD
        cmd.motorCmd[i].tau = 0.0
    for _ in range(20):
        udp.SetSend(cmd); udp.Send(); time.sleep(0.02)

    # CSV logging setup
    header = ["t","step","rate","loop_dt"]
    for i in range(12):
        header += [f"q_cmd_{i}", f"q_meas_{i}", f"iq_{i}", f"err_{i}"]
    f = open(LOG_FILENAME, "w", newline='')
    writer = csv.writer(f)
    writer.writerow(header)

    prev_q_cmd = [cmd.motorCmd[i].q for i in range(12)]
    next_time = time.perf_counter()

    time.sleep(2)

    try:
        for step in range(1, interp_steps + 1):
            next_time += DT
            now = time.perf_counter()
            sleep_t = next_time - now
            loop_dt = 0.0
            if sleep_t > 0:
                time.sleep(sleep_t)
                loop_dt = time.perf_counter() - now
            else:
                # we're behind schedule
                loop_dt = time.perf_counter() - now
                # If we are badly behind, abort
                if loop_dt > LOOP_JITTER_LIMIT:
                    print(f"[ABORT] Loop jitter too large: {loop_dt:.4f}s")
                    raise RuntimeError("loop_jitter_abort")

            # receive state
            udp.Recv(); udp.GetRecv(state)

            rate = float(step) / interp_steps
            Kp_now = Kp_START + (Kp_MAX - Kp_START) * rate

            # build commands with sign_map on delta and strong per-step limit
            for i in range(12):
                delta_desired = (joint_angles_target[i] - qInit[i])
                signed_delta = sign_map[i] * delta_desired
                target_q = qInit[i] + signed_delta * rate

                delta_cmd = target_q - prev_q_cmd[i]
                if abs(delta_cmd) > MAX_STEP:
                    delta_cmd = MAX_STEP * (1 if delta_cmd > 0 else -1)
                q_cmd = prev_q_cmd[i] + delta_cmd

                cmd.motorCmd[i].q = q_cmd
                cmd.motorCmd[i].dq = 0.0
                cmd.motorCmd[i].Kp = Kp_now
                cmd.motorCmd[i].Kd = KD
                cmd.motorCmd[i].tau = 0.0

                prev_q_cmd[i] = q_cmd

            udp.SetSend(cmd); udp.Send()

            # Read telemetry and check safety
            row = [time.time(), step, rate, loop_dt]
            for i in range(12):
                q_meas = state.motorState[i].q
                # try common names for motor current: iq, iq_meas, current
                iq = safe_get(state.motorState[i], "iq", None)
                if iq is None:
                    iq = safe_get(state.motorState[i], "i", 0.0)
                err = safe_get(state.motorState[i], "err", 0)
                row += [prev_q_cmd[i], q_meas, iq, err]

                # Abort conditions
                if iq is not None and abs(iq) > IQ_LIMIT:
                    print(f"[ABORT] High current joint {i}: iq={iq}")
                    raise RuntimeError("iq_limit")
                if ERR_ABORT and err != 0:
                    print(f"[ABORT] Motor error joint {i}: err={err}")
                    raise RuntimeError("motor_error")

            writer.writerow(row)

        # Final gentle hold - ramp to final q slowly with small clamps
        for _ in range(40):
            udp.Recv(); udp.GetRecv(state)
            for i in range(12):
                final_q = qInit[i] + sign_map[i] * (joint_angles_target[i] - qInit[i])
                delta_cmd = final_q - prev_q_cmd[i]
                if abs(delta_cmd) > MAX_STEP:
                    delta_cmd = MAX_STEP * (1 if delta_cmd > 0 else -1)
                q_cmd = prev_q_cmd[i] + delta_cmd
                cmd.motorCmd[i].q = q_cmd
                cmd.motorCmd[i].Kp = 8.0
                cmd.motorCmd[i].Kd = 0.9
                prev_q_cmd[i] = q_cmd
            udp.SetSend(cmd); udp.Send()
            time.sleep(0.02)

        print("Standup completed (logged). Now performing PowerProtect call.")
        # final safety call
        # safe = sdk.Safety(sdk.LeggedType.Go1)
        # safe.PowerProtect(cmd, state, 1)
        # print("PowerProtect called.")
    except Exception as e:
        print("Aborting motion due to:", e)
        # Gentle emergency hold on exception
        udp.Recv(); udp.GetRecv(state)
        q_now = [state.motorState[i].q for i in range(12)]
        for _ in range(50):
            for i in range(12):
                cmd.motorCmd[i].q = q_now[i]
                cmd.motorCmd[i].dq = 0.0
                cmd.motorCmd[i].Kp = 2.0
                cmd.motorCmd[i].Kd = 0.5
                cmd.motorCmd[i].tau = 0.0
            udp.SetSend(cmd); udp.Send()
            time.sleep(0.02)
        raise
    finally:
        f.close()
        print(f"Telemetry saved to {LOG_FILENAME}")


csv_filename = 'joint_angles_log.csv'
udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
cmd = sdk.LowCmd(); state = sdk.LowState(); udp.InitCmdData(cmd)
run_safe_stand(udp, cmd, state, joint_angles_target=getJointAngles(csv_filename), sign_map=SIGNMAP)