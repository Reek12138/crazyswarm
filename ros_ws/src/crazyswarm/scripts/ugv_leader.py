from pycrazyswarm import Crazyswarm
import threading
from enum import Enum
import numpy as np

import argparse
import vrpn
import time
import math
import datetime
from scipy.spatial.transform import Rotation as R

from config import LandingConfig

HEIGHT_Z = 1.0
FORMATION_DIS = 0.5
interrupt_trajectory = False

class State(Enum):
    INIT = 0
    TAKEOFF = 1
    FOLLOW = 2
    LAND = 3



class ugv_tracker:
    def __init__(self):
        self.args = LandingConfig().parser.parse_args()
        self.args.ugv_number = 1
        self.targetx = None
        self.targety = None
        self.state = State.INIT

        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.allcfs = self.swarm.allcfs
        self.cfs = self.allcfs.crazyflies

        self.ugv_tkr = vrpn.receiver.Tracker('ugv' + str(self.args.ugv_number) + self.args.host)
        self.ugv_tkr.register_change_handler(None,self.ugv_pos_update,"position")

        self.ugv_msg = {"position": [None, None, None], "orientation": [None, None, None],
                "velocity": [None, None, None], "heading": [None, None, None]}
        
        self.threadcfs = []
        
    def check_for_interrupt():
        global interrupt_trajectory
        while True:
            userInput = input("输入 '0' 中断飞行: ")
            if userInput == '0':
                interrupt_trajectory = True
                print("飞行将被中断...")
                break

    def take_off(self):
        Z = 1.5
        self.allcfs.takeoff(targetHeight=Z, duration=3.0+Z)
        self.timeHelper.sleep(3 + Z)

    def land(self):
        print("降落中....")
        for cf in self.cfs:
            cf.land(targetHeight=-0.5, duration=4.0)
        self.timeHelper.sleep(4.0)
        self.allcfs.stop()
    
    def ugv_msg_get(self, tkr, flag):
        while flag:
            tkr.mainloop()

    # def print_tracker_position(userdata, t):
    #     print("小车位置更新: Sensor {} is at ({:.3f},{:.3f},{:.3f}) at time {}".format(
    #         t['sensor'],
    #         t['position'][0], t['position'][1], t['position'][2],
    #         t['time'])
    #     )

    def ugv_pos_update(self,userdata,t):
            self.ugv_msg["position"][0], self.ugv_msg["position"][1], self.ugv_msg["position"][2], self.vel_t_now= t['position'][0], \
                    t['position'][1], t['position'][2],t['time'].second
            self.targetx, self.targety = self.ugv_msg["position"][0], self.ugv_msg["position"][1]
            self.relative_distance_isupdate = True

    def drone_control(self, cf, ugv_pos, z, i):
        # global interrupt_trajectory
        # if interrupt_trajectory:
        #     print(f"中断无人机{cf.id}的飞行...")
        #     cf.land(targetHeight=-0.5, duration=2)
        #     return
        
        target_x = ugv_pos["position"][0] + i*FORMATION_DIS
        target_y = ugv_pos["position"][1] + i*FORMATION_DIS

        target_pos = np.array([target_x, target_y, z])

        cf.cmdPosition(target_pos.tolist(),0)
        self.timeHelper.sleepForRate(18)

    def thread_start(self):
        for thread in self.threadcfs:
            thread.start()


    def run(self):

        global interrupt_trajectory

        if self.state == State.INIT:
            if self.ugv_msg["position"][0] is not None:
                print(
                        '车辆当前坐标为：\n x:{:.3f},y:,{:.3f},z:{:.3f}'.format(
                            self.ugv_msg["position"][0], self.ugv_msg["position"][1], self.ugv_msg["position"][2]))
                takeoff_flag = True
                for cf in self.cfs:
                    if cf.position()[0] is not None:
                        print(' 无人机当前坐标为：\n x:{:.3f},y:,{:.3f},z:{:.3f}'.format(
                        cf.position()[0],cf.position()[1], cf.position()[2]))
                    
                    else:
                        print("无人机位置丢失")
                        takeoff_flag = False

                if takeoff_flag == True:
                    self.state = State.TAKEOFF
            else:
                print("无人车位置丢失\n")
                print(self.cfs[0].position(),self.ugv_msg["position"])
                self.state = State.INIT
                
        if self.state == State.TAKEOFF:

            self.take_off()

            # 开启避碰
            xy_radius = 0.125
            radii = 1.0 * xy_radius * np.array([1.0, 1.0, 3.0])
            for i,cf in enumerate(self.cfs):
                others = self.cfs[:i] + self.cfs[(i+1):]
                cf.enableCollisionAvoidance(others, radii)

            self.state = State.FOLLOW
        
        if self.state == State.FOLLOW:
            
            for cf,i in enumerate (self.cfs):
                thread = threading.Thread(target=self.drone_control, args=(cf, self.ugv_msg, HEIGHT_Z, i))
                self.threadcfs.append(thread)
                self.thread_start()
            
        if interrupt_trajectory == True:
            self.state = State.LAND
            for thread in self.threadcfs:
                thread.join()
            self.land()

    def main(self):
        self.run()
        self.timeHelper.sleepForRate(18)
      
        return
if __name__ == "__main__":
    tracking = ugv_tracker()
    
    threads = []
    ugv_track = threading.Thread(target=tracking.ugv_msg_get, args=(tracking.ugv_tkr, True))
    ugv_track.start()

    tracking.main()

    check_land = threading.Thread(target=ugv_tracker.check_for_interrupt)
    check_land.start()

    

    
    


    
