#!/usr/bin/env python3
import motorcortex
from src.motor_driver.canmotorlib import CanMotorController
import numpy as np
import time
import threading
import os
import pathlib
from asyncio import Event

class Gripper():
    def __init__(self) -> None:
        self.__stop_event = Event()

        self.__motor_controller = self.__init_can_driver()
        self.__current_state = None

        self.__torque_min = -2
        self.__torque_max = 2
        self.__velocity = 10

        self.__open_drive_pose = -55
        self.__close_drive_pose = -10

        self.__close_drive_path = 70

        self.__start_open_gripper_th = threading.Thread(target=self.__open_gripper_block_exec, daemon=True)
        self.__start_close_gripper_th = threading.Thread(target=self.__close_gripper_block_exec, daemon=True)

        self.__calibrate_open_pose()

    def __setZeroPosition(self):
        pos, _, _ = self.__motor_controller.set_zero_position()
        while abs(np.rad2deg(pos)) > 0.5:
            pos, vel, curr = self.__motor_controller.set_zero_position()
            print("Position: {}, __Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel), curr))

    def __moveTo(self, start, end):
        vel = self.__velocity
        t_exec = abs(end-start)*3/2/vel

        def getTargetPose(time, vel_sign):
            
            if time <= t_exec/3:
                pos = start + vel_sign*3/2*vel/t_exec*time**2
            elif time <= t_exec * 2 / 3:
                pos = start + vel_sign*vel/6 * t_exec + vel_sign*vel*(time - t_exec/3)
            elif time <= t_exec:
                pos = start + vel_sign*vel*(time - 2*t_exec/3) - 3*vel_sign*vel/2/t_exec*(time - 2*t_exec/3)**2 + vel_sign*vel*t_exec/2
            else:
                pos = end

            return pos
        
        sign = abs(end - start) / (end - start)

        start_time = time.time()
        cur_time = start_time
        torque_lim_counter = 0
        while cur_time - start_time <= t_exec:
            if self.__stop_event.is_set():
                return
            cur_time = time.time()
            dt = cur_time - start_time
            pos = getTargetPose(dt, sign)
            try:
                c_pos, c_vel, c_curr = self.__motor_controller.send_deg_command(pos, 0, 50, 1, 0)
                if (not self.__torque_min is None):
                    if(self.__torque_min > c_curr or self.__torque_max < c_curr):
                        if (torque_lim_counter > 3 and dt > 1.5):
                            print("Torque limiter: {0}".format(c_curr))
                            return
                        else:
                            torque_lim_counter+=1
                    else:
                        torque_lim_counter = 0
                # print("Position: {}, __Velocity: {}, Torque: {}".format(c_pos, c_vel,
                #                                                     c_curr))
            except:
                pass

        c_pos, c_vel, c_curr = self.__motor_controller.send_deg_command(end, 0, 50, 1, 0)
        print("Position: {}, Velocity: {}, Torque: {}".format(c_pos, c_vel,
                                                            c_curr))
        return

    def __open_gripper_block_exec(self):
        pos, vel, curr = self.__motor_controller.enable_motor()
        c_pos = np.rad2deg(pos)
        c_pos = self.__moveTo(c_pos, self.__open_drive_pose)

    def __close_gripper_block_exec(self):
        pos, vel, curr = self.__motor_controller.enable_motor()
        c_pos = np.rad2deg(pos)
        c_pos = self.__moveTo(c_pos, self.__close_drive_pose)

    def __init_can_driver(self):
        motor_id = 0x01
        motor_controller = CanMotorController('/dev/ttyACM0', motor_id)

        while True:
            try:
                pos, vel, curr = motor_controller.enable_motor()
                break
            except:
                print("Error connect to motor, refresh")

        print("Initial Position: {}, __Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel),
                                                                        curr))
        time.sleep(1)
        return motor_controller

    def __calibrate_open_pose(self):
        c_pos, vel, curr = self.__motor_controller.enable_motor()
        c_pos = np.rad2deg(c_pos)
        print("Start calibration")
        time_start = time.time()
        cur_time = time.time()
        dt = cur_time - time_start
        while True:
            if (dt>1):
               print(curr)
               if (curr > self.__torque_max + 0.5 or curr < self.__torque_min - 0.5):
                break
            cur_time = time.time()
            dt = cur_time - time_start
            cmd = c_pos - 0.5
            c_pos, c_vel, curr = self.__motor_controller.send_deg_command(cmd, 0, 30, 2, 0)
            time.sleep(0.04)
        print("Stot calibration")

        self.__open_drive_pose = c_pos
        self.__close_drive_pose = c_pos + self.__close_drive_path

    def open_gripper(self):
        if (self.__current_state is None or self.__current_state == "close"):
            if (self.__start_close_gripper_th.is_alive()):
                self.__stop_event.clear()
                self.__stop_event.set()
                self.__start_close_gripper_th.join()
                self.__stop_event.clear()
            
            if (self.__start_open_gripper_th.is_alive()):
                self.__stop_event.clear()
                self.__stop_event.set()
                self.__start_open_gripper_th.join()
                self.__stop_event.clear()

            self.__start_open_gripper_th = threading.Thread(target=self.__open_gripper_block_exec, daemon=True)
            self.__start_open_gripper_th.start()
            self.__current_state = "open"

    def close_gripper(self):
        if (self.__current_state is None or self.__current_state == "open"):
            if (self.__start_close_gripper_th.is_alive()):
                self.__stop_event.clear()
                self.__stop_event.set()
                self.__start_close_gripper_th.join()
                self.__stop_event.clear()
            
            if (self.__start_open_gripper_th.is_alive()):
                self.__stop_event.clear()
                self.__stop_event.set()
                self.__start_open_gripper_th.join()
                self.__stop_event.clear()

            self.__start_close_gripper_th = threading.Thread(target=self.__close_gripper_block_exec, daemon=True)
            self.__start_close_gripper_th.start()
            self.__current_state = "close"

if __name__=="__main__":
    time.sleep(5)
    parameter_tree = motorcortex.ParameterTree()
    motorcortex_types = motorcortex.MessageTypes()

    try:
        lic_path = os.path.dirname(__file__) + "/mcx.cert.pem"
        print(os.path.dirname(__file__))
        print(lic_path)
        lic_path = "/home/admin/motorcortex-can-module/mcx.cert.pem"
        req, sub = motorcortex.connect('wss://127.0.0.1:5568:5567', motorcortex_types, parameter_tree,
                                        timeout_ms=1000, certificate=lic_path,
                                        login="admin", password="vectioneer")

        print("Request connection is etablished")
    except Exception as e:
        print(f"Failed to establish connection: {e}")
        exit()

    subscription = sub.subscribe(["root/Control/dummyDouble"], 'group1', 5)
    subscription.get()

    gripper = Gripper()
    params = subscription.read()

    while True:
        try:    
            params = subscription.read()
            cmd = params[0].value[0]
            if (cmd == 0):
                gripper.open_gripper()
            elif (cmd == 1):
                gripper.close_gripper()
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
