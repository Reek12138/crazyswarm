from pycrazyswarm import Crazyswarm
import time
import threading
TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
lock = threading.Lock()
swarm = Crazyswarm()
timeHelper = swarm.timeHelper
# cf = swarm.allcfs.crazyflies[0]
allcfs = swarm.allcfs
cfs =  allcfs.crazyflies
# swarm.allcfs
allcfs = swarm.allcfs

def run_sequence(scf, flie_name):
    coords = read_pose(flie_name);  
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
    Z = 1.0
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(60*10 + Z)

    # threads = []
    # for i in range(len(cfs)):
    #     thread = threading.Thread(target=run_sequence, args=(cfs[i], f"{i+1}.txt"))
    #     threads.append(thread)

    # for thread in threads:
    #     thread.start()
    
    # allcfs.land(targetHeight=-0.1, duration=2.5)
    # timeHelper.sleep(2.5)
if __name__ == "__main__":
    main()