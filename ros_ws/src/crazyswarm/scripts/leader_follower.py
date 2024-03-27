from pycrazyswarm import Crazyswarm
import threading
from enum import Enum
import numpy as np

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
# 队形参数
FORMATION_DISTANCE = 0.5  # 跟随者与领航者的水平距离
FORMATION_TYPE = None  # 队形类型，由用户输入决定

swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs
cfs = allcfs.crazyflies

interrupt_trajectory = False

class FlightState(Enum):
    IDLE = 0
    TAKEOFF = 1
    FORMATION_FLIGHT = 2
    LANDING = 3

class FORMATION(Enum):
    LINE = 1
    TRIANGLE = 2

def take_off():
    Z = 1.5
    allcfs.takeoff(targetHeight=Z, duration=3.0+Z)
    timeHelper.sleep(3 + Z)

def land():
    print("降落中....")
    for cf in cfs:
        cf.land(targetHeight=-0.5, duration=4.0)
    timeHelper.sleep(4.0)
    allcfs.stop()

def read_leader_pose(filename):
    with open("./leader_path"+filename,'r') as file:
        path = []
        for line in file:
            x,y,z = map(float,line.strip().split())
            path.append([x,y,z])
    return path

def calculate_follower_positions(leader_pos, formation_type):
    """根据队形类型计算跟随者的位置"""
    if formation_type == "line":
        # 一字型队形
        follower1_pos = (leader_pos[0], leader_pos[1] - FORMATION_DISTANCE, leader_pos[2])
        follower2_pos = (leader_pos[0], leader_pos[1] + FORMATION_DISTANCE, leader_pos[2])
    elif formation_type == "triangle":
        # 三角型队形
        follower1_pos = (leader_pos[0] - FORMATION_DISTANCE, leader_pos[1] - FORMATION_DISTANCE, leader_pos[2])
        follower2_pos = (leader_pos[0] - FORMATION_DISTANCE, leader_pos[1] + FORMATION_DISTANCE, leader_pos[2])
    else:
        raise ValueError("Unknown formation type")
    return follower1_pos, follower2_pos

def flight_formation(leader_cf, follower1_cf, follower2_cf, leader_path, formation_type):
    """根据给定的队形和路径执行飞行"""
    global interrupt_trajectory
    for pos in leader_path:
        if interrupt_trajectory:
            print("中断飞行....")
            return True
        leader_pos = np.array(pos)
        f1_pos, f2_pos = calculate_follower_positions(leader_pos, formation_type)
        
        # 发送位置命令
        leader_cf.cmdPosition(leader_pos, 0)
        follower1_cf.cmdPosition(f1_pos, 0)
        follower2_cf.cmdPosition(f2_pos, 0)
        
        # 等待一定时间以保持队形移动
        timeHelper.sleepForRate(1.0)
    return False
    
def check_for_interrupt():
    global interrupt_trajectory
    while True:
        userInput = input("输入 '0' 中断飞行: ")
        if userInput == '0':
            interrupt_trajectory = True
            print("飞行将被中断...")
            break

def main():
    global FORMATION_TYPE
    
    print("队形选择：1 - 一字型(Line), 2 - 三角型(Triangle)")
    formation_input = input("请输入队形类型：")
    if formation_input == '1':
        FORMATION_TYPE = "line"
    elif formation_input == '2':
        FORMATION_TYPE ="triangle"
    else:
        print("无效的输入,退出程序")
        return

    leader_path = read_leader_pose("leader.txt")

    leader_cf = cfs[0]
    follower1_cf = cfs[1]
    follower2_cf = cfs[2]

    # 状态机
    current_state = FlightState.IDLE
    while True:
        if current_state ==FlightState.IDLE:
            take_off()
            current_state = FlightState.TAKEOFF

        elif current_state == FlightState.TAKEOFF:
            interrupted = flight_formation(leader_cf, follower1_cf, follower2_cf, leader_path, FORMATION_TYPE)
            if interrupted:
                current_state = FlightState.LANDING
            else:
                print("完成编队飞行，准备降落")
                current_state = FlightState.LANDING

        elif current_state == FlightState.LANDING:
            land()
            print("飞行任务完成，所有无人机降落")
            break

if __name__ == "__main__":
    main()

