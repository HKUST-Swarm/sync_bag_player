#!/usr/bin/env python3
from __future__ import print_function
import sys
import time
import lcm
from SyncBagCtrl import *
from TimeSync import *
import keyboard  # using module keyboard
import readchar
from pynput.keyboard import Key, Listener
import threading

class SyncCtrl:
    def __init__(self, rate=1.0, start_t = 0, duration=1000000):
        self.rate = rate
        self.start_t = start_t
        self.duration = duration
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
        self.sync_sub = self.lc.subscribe("PLAYERS_SYNC", self.time_sync_handle)
        self.is_terminated = False
        self.t = threading.Thread(target = self.lcm_thread)
        self.t.start()

    def time_sync_handle(self, channel, msg):
        msg = TimeSync.decode(msg)
        t_p_sys = msg.played_time_sys
        t_p_bag = msg.played_time_bag
        r = msg.rate
        error_p = t_p_sys*r-t_p_bag
        print(f"[SyncCtrl] time_sync_handle t_played_sys {t_p_sys:.1f} t_played_bag {t_p_bag:.1f} error {error_p*1000:3.1f}ms", end="\r")
        sys.stdout.flush()

    def work(self):
        self.start()
        while not self.is_terminated:
            try:
                k = readchar.readkey()
                if k == "q":
                    self.stop()
                elif k == " ":
                    self.pause()
                else:
                    print(k)
            except KeyboardInterrupt:
                self.stop()
                
    def lcm_thread(self):
        while not self.is_terminated:
            self.lc.handle_timeout(10)

    def start(self):
        ctrl = SyncBagCtrl()
        ctrl.cmd = 1
        ctrl.system_time = time.time()
        ctrl.duration = self.duration
        ctrl.rate = self.rate
        ctrl.start_t = self.start_t
        self.lc.publish("CTRL_PLAYER", ctrl.encode())
        print("[SyncCtrl] Start bag play")
        

    def stop(self):
        ctrl = SyncBagCtrl()
        ctrl.cmd = -1
        self.lc.publish("CTRL_PLAYER", ctrl.encode())
        self.is_terminated = True
        time.sleep(0.1)

    def pause(self):
        ctrl = SyncBagCtrl()
        ctrl.cmd = 0
        self.lc.publish("CTRL_PLAYER", ctrl.encode())
        print("[SyncCtrl] Pause bag play")

        
    def on_release(self, key):
        print('{0} release'.format(
            key))

    def on_press(self, key):
        print('{0} pressed'.format(
            key))
        if key == Key.esc or key == "q":
            print('Stoping the bag play')
            self.stop()
        if key == Key.space:
            print('Pausing the bag play')
            self.pause()
    
    def on_release(self, key):
        print('{0} release'.format(
            key))


if __name__ == "__main__":
    sync_ctrl = SyncCtrl()
    sync_ctrl.work()