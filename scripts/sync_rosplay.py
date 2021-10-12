#!/usr/bin/env python3
from __future__ import print_function
import rospy
import rosbag
import sys
from rosgraph_msgs.msg import Clock
import time
import lcm
from SyncBagCtrl import *
from TimeSync import *
import threading

class SyncBagPlayer:
    def __init__(self, bag_path, play_t0_sys=None, start = 0, rate=1.0, autostart=False):
        self.bag = rosbag.Bag(bag_path)
        self.prepare_publishers()
        self.play_t0_sys = play_t0_sys
        self.bag_t0 = None
        self.r = rospy.Rate(10000)
        self.start_bag_duration = start
        self.total_time = self.bag.get_end_time() - self.bag.get_start_time()
        self.rate = rate
        self.is_bag_playing = autostart
        self.is_terminated = False

        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
        self.ctrl_sub = self.lc.subscribe("CTRL_PLAYER", self.sync_bag_ctrl_handle)
        self.t = threading.Thread(target = self.lcm_thread)
        self.t.start()
        
    
    def lcm_thread(self):
        while not self.is_terminated:
            self.lc.handle_timeout(10)
    
    def sync_bag_ctrl_handle(self, channel, msg):
        msg = SyncBagCtrl.decode(msg)
        if msg.cmd == 1:
            self.is_bag_playing = True
            self.play_t0_sys = msg.system_time
        elif msg.cmd < 0:
            print("[SyncBagPlayer] Player cancelled by LCM, exiting.")
            self.is_bag_playing = False
            self.is_terminated = True
            sys.stdout.flush()
        elif msg.cmd == 0:
            print("[SyncBagPlayer] Player paused by LCM.")
            self.is_bag_playing = False
        
    
    def prepare_publishers(self):
        bag = self.bag
        topics = bag.get_type_and_topic_info()[1]
        publishers = {}
        #Prepare publishers for bag
        for topic, msg, t in bag.read_messages():
            if topic not in publishers:
                publishers[topic] = rospy.Publisher(topic, type(msg), queue_size=10)
            if len(topics) == len(publishers):
                break
        print("[SyncBagPlayer] Topics", publishers.keys())
        self.publishers = publishers
        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=10)
    
    def send_time_sync(self, tsys, tplayedsys, tplayedbag):
        t_sync = TimeSync()
        t_sync.system_time = tsys
        t_sync.played_time_sys = tplayedsys
        t_sync.played_time_bag = tplayedbag
        t_sync.rate = self.rate
        self.lc.publish("PLAYERS_SYNC", t_sync.encode())

    def play(self):
        count = 0
        all_msgs = self.bag.read_messages()
        print("[SyncBagPlayer] Ready.")
        while not rospy.is_shutdown() and not self.is_terminated:
            if not self.is_bag_playing:
                time.sleep(0.001)
                continue
            try:
                t_system = time.time()
                topic, msg, t_bag  = next(all_msgs)
               
                if self.play_t0_sys is None:
                    self.play_t0_sys = t_system
                if self.bag_t0 is None:
                    self.bag_t0 = t_bag
                    print("[SyncBagPlayer] SYS_T0", self.play_t0_sys, "BAG_T0", t_bag.to_sec(), "Now", t_system , "Start bag", self.start_bag_duration)

                t_played_time_sys = t_system - self.play_t0_sys
                t_bag_played = (t_bag - self.bag_t0).to_sec() - self.start_bag_duration

                if t_bag_played < 0:
                    continue

                while t_played_time_sys*self.rate < t_bag_played:
                    t_system = time.time()
                    t_played_time_sys = t_system - self.play_t0_sys
                    time.sleep(0.0001)

                self.send_time_sync(t_system, t_played_time_sys, t_bag_played)

                self.publishers[topic].publish(msg)
                sim_clock = Clock()
                sim_clock.clock = t_bag
                self.clock_pub.publish(sim_clock)
                if count % 100 == 0:
                    t_ = (time.time()- self.play_t0_sys)*self.rate
                    progress = t_ / self.total_time * 100
                    print("Time {:5.2f}/{:5.2f} Progress {:3.2f}% Count {}".format(t_, self.total_time, progress, count), end="\r")
                    sys.stdout.flush()
                count += 1
            except Exception:
                print("[SyncBagPlayer] Exiting.")
                break

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Play bags synchronously from dockers')
    
    parser.add_argument('--path', type=str,
                    help='bagpath')

    parser.add_argument('--syst', type=float,
                    default=time.time() + 1,
                    help='start time')
    
    parser.add_argument('--rate', type=float,
                    default=1.0,
                    help='play rate')
        
    parser.add_argument('--start', type=float,
                    default=0.0,
                    help='start')

    parser.add_argument('--autostart', type=bool,
                    default=False,
                    help='automatics start play at spec system time')

    args = parser.parse_args()
    rospy.init_node("player")
    player = SyncBagPlayer(args.path, args.syst, args.start, args.rate, args.autostart)
    player.play()