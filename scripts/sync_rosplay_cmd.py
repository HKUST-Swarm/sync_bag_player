#!/usr/bin/env python2
from __future__ import print_function
import sys
import time
import lcm
from SyncBagCtrl import *
import keyboard  # using module keyboard
from pynput.keyboard import Key, Listener
import readchar

class SyncCtrl:
    def __init__(self):
        self.lc = lcm.LCM()

    def work(self):
        print("Cmd is ready. wait for your control")

        while True:
            k = readchar.readkey()
            if k == "q":
                self.stop()

    def stop(self):
        ctrl = SyncBagCtrl()
        ctrl.cmd = -1
        self.lc.publish("sync_bag_ctrl", ctrl.encode())
        time.sleep(1)
        #exit()

    def pause(self):
        pass
        
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