from pynput import keyboard

from kuavoRobotSDK import kuavo
import time

robot_instance = kuavo("3_7_kuavo")


def setEndEffector(robot_l ,robot_r):

    if robot_l == 0  and robot_r == 0:
        robot_instance.set_robot_joller_position(0, 0)
    elif robot_l == 0 and robot_r == 1:
        robot_instance.set_robot_joller_position(0, 255)
    elif robot_l == 1 and robot_r == 1:
        robot_instance.set_robot_joller_position(255, 255)
    elif robot_l == 1 and robot_r == 0:
        robot_instance.set_robot_joller_position(255, 0)
    else:
        print("输入范围有误")
    

def on_press(key):

    try:
        print("检测到了键盘输入，开始进行夹住的规划")
            
        # if key.char == "w":

        robot_l = int(input("请输入左夹爪的开合值,1代表的关,0代表开"))
        # time.sleep(5)
        robot_r = int(input("请输入右夹爪的开合值,1代表的关,0代表开"))

        setEndEffector(robot_l=robot_l, robot_r=robot_r)
        print("+++++++++++++++++++++++")

    except AttributeError:
        pass

listen = keyboard.Listener(on_press=on_press)

listen.start()




while(1):
    pass



