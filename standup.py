#!/usr/bin/python

import sys
import time
import math
import numpy as np
import csv

sys.path.append('/home/lilly/robot_shoe/unitree_legged_sdk/lib/python/amd64/')
import robot_interface as sdk

MOTOR_IDS = {'FR_0': 0, 'FR_1': 1, 'FR_2': 2,
		'FL_0': 3, 'FL_1': 4, 'FL_2': 5,
		'RR_0': 6, 'RR_1': 7, 'RR_2': 8,
		'RL_0': 9, 'RL_1': 10, 'RL_2': 11}

POS_STOP_F = math.pow(10, 9)
VEL_STOP_F = 16000.0

HIGHLEVEL = 0xee
LOWLEVEL = 0xff

SIGNMAP =  [1, 1, -1, -1, 1, -1, 1, 1, -1, -1, -1, -1]


def jointLinearInterpolation(initPos, targetPos, rate):
	rate = np.fmin(np.fmax(rate, 0.0), 1.0)
	p = initPos * (1 - rate) + targetPos * rate
	return p

def jointleft(desPos, targetPos):
	p = targetPos - desPos
	return p

def getJointAngles(csv_filename):
	with open(csv_filename, 'r') as csvfile:
		reader = csv.reader(csvfile)
		row = next(reader)  # 读取第一行数据

	joint_angles = [float(angle) for angle in row]
	print(f"joint angles: {joint_angles}")
	return joint_angles

def standup():
	# init
	dt = 0.002
	qInit = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	qDes = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	qleft = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	rate_count = 0
	Kp = 20
	Kd = 2

	udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
	safe = sdk.Safety(sdk.LeggedType.Go1)

	cmd = sdk.LowCmd()
	state = sdk.LowState()
	udp.InitCmdData(cmd)

	motiontime = 0

	# get target angles
	csv_filename = 'joint_angles_log.csv'
	joint_angles = getJointAngles(csv_filename)


	# sleep to give time to get hold of the dog
	time.sleep(5)

	while True:
		time.sleep(dt)
		#print("done sleeping")

		motiontime += 1

		# receive state
		udp.Recv()
		udp.GetRecv(state)

		if (motiontime >= 0):

			# first, get record initial position
			if (motiontime >= 0 and motiontime < 10):
				for i in range(12):
					qInit[i] = state.motorState[i].q

					# print(qInit)

			# second, move legs to the standing position
			if (motiontime >= 10):
				rate_count += 1
				rate = rate_count / 2000.0

				for i in range(12):
					qDes[i] = jointLinearInterpolation(qInit[i], joint_angles[i], rate)
					qleft[i] = jointleft(qDes[i], joint_angles[i])
				if motiontime % 1000 == 0:
					print("qleft:", qleft)
					print("qDes:", qDes)

				for i in range(12):
					cmd.motorCmd[i].q = qDes[i] * SIGNMAP[i]
					cmd.motorCmd[i].dq = 0
					cmd.motorCmd[i].Kp = Kp
					cmd.motorCmd[i].Kd = Kd
					cmd.motorCmd[i].tau = 0
					#print(motiontime, qDes[i])

		if (motiontime > 10):
			safe.PowerProtectsuggested(cmd, state, 1)

		# send commands
		udp.SetSend(cmd)
		udp.Send()


def standup_safe():
	import time
	dt = 0.002  # 2 ms control step
	qInit = [0.0] * 12
	qDes = [0.0] * 12
	qleft = [0.0] * 12

	rate_count = 0
	Kp_base = 25   # starting stiffness
	Kp_max = 40    # max stiffness after ramp-up
	Kd_val = 1.5   # low damping for smooth movement

	# --- UDP & Safety ---
	udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
	safe = sdk.Safety(sdk.LeggedType.Go1)
	cmd = sdk.LowCmd()
	state = sdk.LowState()
	udp.InitCmdData(cmd)

	motiontime = 0

    # --- Load target joint angles ---
	csv_filename = 'joint_angles_standup.csv'
	joint_angles = getJointAngles(csv_filename)  # should return list of 12 floats

    # --- Give yourself time to grab the robot ---
	print("Standing up in 5 seconds...")
	time.sleep(5)

	while True:
		time.sleep(dt)
		motiontime += 1

        # --- Receive state ---
		udp.Recv()
		udp.GetRecv(state)

		if motiontime < 10:
			# record initial joint positions
			for i in range(12):
				qInit[i] = state.motorState[i].q

			print(qInit)	

		else:
			rate_count += 1
			rate = min(rate_count / 2000.0, 1.0)  # ~4 seconds to reach stand
			Kp_now = Kp_base + (Kp_max - Kp_base) * rate  # gradual Kp ramp

			for i in range(12):
				qDes[i] = jointLinearInterpolation(qInit[i], joint_angles[i], rate)
				qleft[i] = jointleft(qDes[i], joint_angles[i])  # if needed

				cmd.motorCmd[i].q = qDes[i] * SIGNMAP[i]
				cmd.motorCmd[i].dq = 0.0
				cmd.motorCmd[i].Kp = Kp_now
				cmd.motorCmd[i].Kd = Kd_val
				cmd.motorCmd[i].tau = 0.0

			# Call safety after movement mostly complete
			if rate >= 0.95:
				safe.PowerProtect(cmd, state, 1)

        # --- Send commands ---
		udp.SetSend(cmd)
		udp.Send()

import time, math

def one_joint_direction_test(udp, cmd, state, step=0.05):
    """
    step: small radian offset to command (positive)
    Returns: list of observed directions: +1 moved positive, -1 moved negative, 0 no move
    """
    udp.Recv(); udp.GetRecv(state)
    qInit = [state.motorState[i].q for i in range(12)]

    # low stiffness safe hold
    baseKp = 10
    Kd = 1.0

    results = [0]*12

    for i in range(12):
        print(i)
        # prepare command - hold all motors at qInit
        for j in range(12):
            cmd.motorCmd[j].q = qInit[j]
            cmd.motorCmd[j].dq = 0.0
            cmd.motorCmd[j].Kp = baseKp
            cmd.motorCmd[j].Kd = Kd
            cmd.motorCmd[j].tau = 0.0

        # send hold
        udp.SetSend(cmd); udp.Send()
        time.sleep(0.2)

        # now command a small positive step for motor i
        cmd.motorCmd[i].q = qInit[i] + step
        udp.SetSend(cmd); udp.Send()
        time.sleep(0.4)  # let motor move

        # read back
        udp.Recv(); udp.GetRecv(state)
        qNew = state.motorState[i].q
        delta = qNew - qInit[i]

        if abs(delta) < 1e-4:
            results[i] = 0
        elif delta > 0:
            results[i] = +1
        else:
            results[i] = -1

        # return motor i to init
        cmd.motorCmd[i].q = qInit[i]
        udp.SetSend(cmd); udp.Send()
        time.sleep(0.2)

    return results

def run_sign_direction_test():
	udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
	cmd = sdk.LowCmd(); state = sdk.LowState(); udp.InitCmdData(cmd)
	dirs = one_joint_direction_test(udp, cmd, state, step=0.05)
	print("Observed directions (+1 means motor moved + when commanded +):", dirs)


def emergency_hold(udp, cmd, state, hold_cycles=50, kp_hold=2.0, kd_hold=0.5):
    # read current state once
    udp.Recv(); udp.GetRecv(state)
    q_now = [state.motorState[i].q for i in range(12)]

    # prepare a gentle hold command and send repeatedly
    for _ in range(hold_cycles):
        for i in range(12):
            cmd.motorCmd[i].q = q_now[i]
            cmd.motorCmd[i].dq = 0.0
            cmd.motorCmd[i].Kp = kp_hold   # very low stiffness
            cmd.motorCmd[i].Kd = kd_hold
            cmd.motorCmd[i].tau = 0.0
        udp.SetSend(cmd); udp.Send()
        time.sleep(0.02)  # 20 ms between sends

def run_emergency_hold():
	udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
	cmd = sdk.LowCmd(); state = sdk.LowState(); udp.InitCmdData(cmd)
	emergency_hold(udp, cmd, state)

import time
import json
import math

def joint_direction_calibration(udp, cmd, state, step=0.05, kp_test=10.0, kd_test=1.0):
    """
    Tests each joint direction at low stiffness.
    step: small offset in radians (+0.05 rad ≈ 2.8°)
    kp_test/kd_test: low stiffness to avoid torque spikes
    Returns: sign_map (list of +1/-1 per joint)
    """
    sign_map = []
    results = []

    # Get initial positions
    udp.Recv(); udp.GetRecv(state)
    qInit = [state.motorState[i].q for i in range(12)]

    print("\nStarting joint direction test...")
    print("Make sure robot is restrained and safe to move!\n")

    for i in range(12):
        print(f"\nTesting joint {i}...")

        # Hold all joints at current positions
        for j in range(12):
            cmd.motorCmd[j].q = qInit[j]
            cmd.motorCmd[j].dq = 0.0
            cmd.motorCmd[j].Kp = kp_test
            cmd.motorCmd[j].Kd = kd_test
            cmd.motorCmd[j].tau = 0.0
        udp.SetSend(cmd); udp.Send()
        time.sleep(0.2)

        # Command small positive step for this joint
        cmd.motorCmd[i].q = qInit[i] + step
        udp.SetSend(cmd); udp.Send()
        time.sleep(0.4)  # let it move

        # Read back state
        udp.Recv(); udp.GetRecv(state)
        qNew = state.motorState[i].q
        iq = getattr(state.motorState[i], 'dq', 0.0)  # change to 'iq' if SDK provides motor current

        delta = qNew - qInit[i]
        direction = 0
        if abs(delta) < 1e-4:
            direction = 0
        elif delta > 0:
            direction = +1
        else:
            direction = -1

        sign_map.append(direction if direction != 0 else 1)  # default to +1 if unmoved
        results.append({
            "joint": i,
            "q_init": qInit[i],
            "q_new": qNew,
            "delta": delta,
            "measured_direction": direction,
            "motor_current_or_dq": iq
        })

        print(f"  q_init: {qInit[i]:.4f}, q_new: {qNew:.4f}, delta: {delta:.4f}, dir: {direction}")

        # Return joint to init
        cmd.motorCmd[i].q = qInit[i]
        udp.SetSend(cmd); udp.Send()
        time.sleep(0.3)

    # Save to JSON
    data = {"sign_map": sign_map, "results": results}
    with open("joint_sign_map.json", "w") as f:
        json.dump(data, f, indent=2)

    print("\nCalibration complete!")
    print(f"Sign map: {sign_map}")
    print("Results saved to joint_sign_map.json")
    return sign_map

def run_joint_direction_calibration():
# Example usage (wired recommended):
	udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
	cmd = sdk.LowCmd(); state = sdk.LowState(); udp.InitCmdData(cmd)
	sign_map = joint_direction_calibration(udp, cmd, state)
	print(sign_map)

import time

import time
import math

def standup_safe_corrected(udp, cmd, state,
                           joint_angles_target,   # length-12, in same encoder frame as qInit
                           sign_map,
                           dt=0.002,
                           interp_seconds=8.0,
                           Kp_start=5.0,
                           Kp_max=30.0,
                           Kd_val=1.2,
                           max_step_per_cycle=0.01):
    """
    Safe standup with corrected sign handling and seeded initial commands.
    - joint_angles_target must be in the same frame as qInit (encoder frame).
    - sign_map flips the delta only: cmd_q = qInit + sign_map*(target - qInit)*rate
    """
    interp_steps = max(1, int(interp_seconds / dt))
    # --- read initial state ---
    udp.Recv(); udp.GetRecv(state)
    qInit = [state.motorState[i].q for i in range(12)]

    # --- Seed cmd with current positions to avoid large initial jump ---
    for i in range(12):
        cmd.motorCmd[i].q = qInit[i]
        cmd.motorCmd[i].dq = 0.0
        cmd.motorCmd[i].Kp = Kp_start
        cmd.motorCmd[i].Kd = Kd_val
        cmd.motorCmd[i].tau = 0.0

    # send a few gentle holds so motors are engaged smoothly
    for _ in range(20):
        udp.SetSend(cmd)
        udp.Send()
        time.sleep(0.02)   # 20 ms

    # slight pause before motion
    time.sleep(0.5)

    # prepare loop variables
    prev_q_cmd = [cmd.motorCmd[i].q for i in range(12)]
    step = 0
    Kp_now = Kp_start

    # precise timing
    next_time = time.perf_counter()

    while True:
        # maintain real-time loop at dt
        next_time += dt
        now = time.perf_counter()
        sleep_time = next_time - now
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            # if we are behind, do not accumulate prints; just run next cycle
            next_time = time.perf_counter()

        step += 1

        # receive latest state
        udp.Recv(); udp.GetRecv(state)

        if step <= interp_steps:
            rate = float(step) / interp_steps  # 0..1
            # ramp Kp slowly
            Kp_now = Kp_start + (Kp_max - Kp_start) * rate

            for i in range(12):
                # compute signed delta relative to qInit (don't flip qInit itself)
                delta_desired = (joint_angles_target[i] - qInit[i])
                signed_delta = sign_map[i] * delta_desired
                target_q = qInit[i] + signed_delta * rate

                # rate limiting based on previous commanded q (prev_q_cmd)
                delta_cmd = target_q - prev_q_cmd[i]
                if abs(delta_cmd) > max_step_per_cycle:
                    delta_cmd = max_step_per_cycle * (1 if delta_cmd > 0 else -1)
                q_cmd = prev_q_cmd[i] + delta_cmd

                # write to command
                cmd.motorCmd[i].q = q_cmd
                cmd.motorCmd[i].dq = 0.0
                cmd.motorCmd[i].Kp = Kp_now
                cmd.motorCmd[i].Kd = Kd_val
                cmd.motorCmd[i].tau = 0.0

                prev_q_cmd[i] = q_cmd

        else:
            # final hold: ensure exact target (signed) and gentle Kp
            for i in range(12):
                final_q = qInit[i] + sign_map[i] * (joint_angles_target[i] - qInit[i])
                # small safety step clamp for final arrival
                delta_cmd = final_q - prev_q_cmd[i]
                if abs(delta_cmd) > max_step_per_cycle:
                    delta_cmd = max_step_per_cycle * (1 if delta_cmd > 0 else -1)
                q_cmd = prev_q_cmd[i] + delta_cmd

                cmd.motorCmd[i].q = q_cmd
                cmd.motorCmd[i].dq = 0.0
                cmd.motorCmd[i].Kp = 18.0   # gentle hold
                cmd.motorCmd[i].Kd = 0.9
                cmd.motorCmd[i].tau = 0.0

                prev_q_cmd[i] = q_cmd

            # send final protects once steady
            udp.SetSend(cmd); udp.Send()
            # give a short settle period, then call power protect once
            time.sleep(0.5)
            safe = sdk.Safety(sdk.LeggedType.Go1)
            safe.PowerProtect(cmd, state, 1)
            return  # done

        # send
        udp.SetSend(cmd)
        udp.Send()


def run_standup_safe_with_signmap():
	csv_filename = 'joint_angles_log.csv'
	udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
	cmd = sdk.LowCmd(); state = sdk.LowState(); udp.InitCmdData(cmd)
	standup_safe_corrected(udp, cmd, state, joint_angles_target=getJointAngles(csv_filename), sign_map=SIGNMAP)


if __name__ == '__main__':
	run_standup_safe_with_signmap()

