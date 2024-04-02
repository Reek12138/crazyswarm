from pycrazyswarm import Crazyswarm
import time
import threading
TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0

swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs
cfs = allcfs.crazyflies
allcfs = swarm.allcfs


def run_sequence(scf, flie_name):
    coords = read_pose(flie_name)
    for coord in coords:
        print((coord))
        scf.cmdPosition([coord[0], coord[1], coord[2]])

        timeHelper.sleepForRate(15)

    scf.land(targetHeight=-0.1, duration=2.5)
    timeHelper.sleep(2.5)


def read_pose(filename):
    with open("./track/"+filename, 'r') as f:
        coords = []
        for line in f:
            parts = line.split()
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            coords.append([x, y, z])
    return coords


def main():
    cfs[0].setGroupMask(1)
    Z = 0.5
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z, groupMask=1)
    timeHelper.sleep(2 + Z)
    allcfs.goTo(goal=[0.5, 0, 0], yaw=0, duration=2.0, groupMask=1)
    timeHelper.sleep(2.5)
    allcfs.goTo(goal=[0, 0.5, 0], yaw=0, duration=2.0, groupMask=1)
    timeHelper.sleep(2.5)
    allcfs.goTo(goal=[0, 0, 0.5], yaw=0, duration=2.0, groupMask=1)
    timeHelper.sleep(2.5)
    allcfs.land(targetHeight=0.1, duration=2.5, groupMask=1)


if __name__ == "__main__":
    main()
