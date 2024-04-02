from pycrazyswarm import Crazyswarm
import threading

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0

swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs
cfs = allcfs.crazyflies

interrupt_trajectory = False


def run_sequence(scf, file_name):
    global interrupt_trajectory
    coords = read_pose(file_name)
    for coord in coords:
        if interrupt_trajectory:
            break
        scf.cmdPosition([coord[0], coord[1], coord[2]], 0)
        timeHelper.sleepForRate(18)

    scf.land(targetHeight=-0.05, duration=coord[2] * 2+2)
    timeHelper.sleep(coord[2]*2 + 2)


def read_pose(filename):
    with open("./xiezhuanquan/" + filename, 'r') as f:
        coords = []
        for line in f:
            parts = line.split()
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            coords.append([x, y, z])
    return coords


def takeoff():
    Z = 1.5
    allcfs.takeoff(targetHeight=Z, duration=3.0 + Z)
    timeHelper.sleep(3 + Z)


def land():
    print("降落中...")
    for cf in cfs:
        cf.land(targetHeight=-0.05, duration=4.0)
    timeHelper.sleep(4.0)
    allcfs.stop()


def main():
    global interrupt_trajectory
    while True:
        user_input = input("输入1起飞并执行轨迹,输入其他数字(2-9)退出程序: ")

        if user_input == '1':
            takeoff()

            threads = []
            interrupt_trajectory = False  # 重置中断标志
            for i in range(len(cfs)):
                thread = threading.Thread(
                    target=run_sequence, args=(cfs[i], f"{i + 1}.txt"))
                threads.append(thread)

            for thread in threads:
                thread.start()

            user_input_land = input("输入0中断轨迹执行并降落,输入其他数字(2-9)退出程序: ")
            if user_input_land == '0':
                interrupt_trajectory = True  # 设置中断标志
                for thread in threads:
                    thread.join()  # 等待轨迹跟踪线程结束
            else:
                print("退出程序。")
                break
        elif user_input == '0':
            land()
        else:
            print("退出程序。")
            break


if __name__ == "__main__":
    main()
