#!/usr/bin/env python3
from __future__ import print_function
from math import e
import sys
import time
import lcm
from SyncBagCtrl import *
from TimeSync import *
from PlayerStats import *
import keyboard  # using module keyboard
import readchar
from pynput.keyboard import Key, Listener
import threading
import math
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning) 

PLAY_DELAY = 0.1
class SyncCtrl:
    def __init__(self, rate=1.0, t_start = 0, duration=1000000, start_delay =PLAY_DELAY):
        self.rate = rate
        self.start_t = t_start
        self.duration = duration
        self.is_paused = False
        self.pause_start = None
        self.t_sys_start = time.time()
        self.start_delay = start_delay

        self.t_p_sys = {}
        self.t_p_bag = {}
        self.error_p = {}
        self.t_bag = {}

        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
        self.sync_sub = self.lc.subscribe("PLAYER_STATS", self.time_sync_handle)
        self.is_terminated = False
        self.t = threading.Thread(target = self.lcm_thread)
        self.t.start()
        
    def time_sync_handle(self, channel, msg):
        msg = PlayerStats.decode(msg)
        self.t_p_sys[msg.drone_id] = msg.played_time_sys
        self.t_p_bag[msg.drone_id] = msg.played_time_bag
        r = msg.rate
        self.error_p[msg.drone_id] = msg.played_time_sys*r-msg.played_time_bag
        self.t_bag[msg.drone_id] = msg.bag_time_abs
        sys.stdout.flush()

    def work(self):
        self.start()
        while not self.is_terminated:
            try:
                k = readchar.readkey()
                if k == "q" or k =="\x03":
                    self.stop()
                elif k == " ":
                    self.pause()
                else:
                    print(k)
            except Exception as e:
                self.stop()
                raise e

    def statistics(self):
        status = "Playing"
        if self.is_paused:
            status = "Paused"
        _num_drone = len(self.t_p_sys)
        t_p_sys = 0
        t_p_bag = 0
        error_p = 0
        max_t_bag = 0
        min_t_bag = float('inf')

        for name in self.t_p_sys:
            t_p_sys += self.t_p_sys[name]
            t_p_bag += self.t_p_bag[name]
            error_p += math.fabs(self.error_p[name])
            if min_t_bag > self.t_bag[name]:
                min_t_bag = self.t_bag[name]
            if max_t_bag < self.t_bag[name]:
                max_t_bag = self.t_bag[name]

        if _num_drone > 0:
            t_p_sys/=_num_drone
            t_p_bag/=_num_drone
            error_p/=_num_drone

        err_bags = (max_t_bag-min_t_bag)*1000

        print(f"[SyncCtrl][{status}] num {_num_drone} t_played_bag {t_p_bag:.1f} error {err_bags:.2f}ms error_sys {error_p*1000:3.1f}ms", end="\r")
            
    def lcm_thread(self):
        while not self.is_terminated:
            self.statistics()
            self.lc.handle_timeout(10)

    def start(self):
        ctrl = SyncBagCtrl()
        ctrl.cmd = 1
        self.t_sys_start = ctrl.system_time = time.time() + self.start_delay
        ctrl.duration = self.duration
        ctrl.rate = self.rate
        ctrl.start_t = self.start_t
        self.lc.publish("CTRL_PLAYER", ctrl.encode())
        print(f"[SyncCtrl] Start bag play rate {ctrl.rate:.1f}s start {ctrl.start_t:.1f}s duration {ctrl.duration:.1f}s. Ctrl-C or q to exit. Space to pause.")
        

    def stop(self):
        ctrl = SyncBagCtrl()
        ctrl.cmd = -1
        self.lc.publish("CTRL_PLAYER", ctrl.encode())
        self.is_terminated = True
        time.sleep(0.1)
        print("[SyncCtrl] Stop bag play. Exit.")


    def pause(self):
        if not self.is_paused:
            ctrl = SyncBagCtrl()
            ctrl.cmd = 0
            self.lc.publish("CTRL_PLAYER", ctrl.encode())
            self.pause_start = time.time()
            self.is_paused = True
        else:
            ctrl = SyncBagCtrl()
            ctrl.cmd = 2
            ctrl.system_time = self.t_sys_start + PLAY_DELAY + time.time() - self.pause_start
            self.lc.publish("CTRL_PLAYER", ctrl.encode())
            self.is_paused = False
        
if __name__ == "__main__":
    sync_ctrl = SyncCtrl()
    sync_ctrl.work()