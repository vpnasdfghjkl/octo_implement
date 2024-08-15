import matplotlib.pyplot as plt
import re
import os
import numpy as np

src_path = "/home/rebot801/LIuXin/Dataset/"

# src_path = "/home/rebot801/LIuXin/Dataset_old/Dataset111/"

"""功能函数"""
def my_match(txt_line):
    """用于匹配字符串"""

    Info_txt = []
    Debug_txt = []

    for line in txt_line:

        match = re.search(r'[INFO]', line)

        match_Debug = re.search(r'[DEBUG]', line)

        if match:
            if "程序正常启动" in line or "当前是初始化函数" in line:
                continue

            index = line.find(":[")

            # Info_txt.append(line[index + 1:].replace(",", " ").replace("[", " ").replace("]"," "))
            Info_txt.append(np.array(line[index + 1:].replace("[", "").replace("]", "").replace("\n", "").split(",")))

        elif match_Debug:
            if "程序正常启动" in line or "当前是初始化函数" in line:
                continue

            index = line.find(":[")
            # Debug_txt.append(line[index + 1:])
            Debug_txt.append(np.array(line[index + 1:].replace("[", "").replace("]", "").replace("\n", "").split(",")))

    return Info_txt, Debug_txt


def Get_x(txt_line:list, count: int, num = None):

    if num is None:

        sum_x = []

        sum_y_values = [[] for _ in range(count)]

        i = 0

        for line in txt_line:

            i = i + 1
            
            y_values = [float(num) for num in line[7:15]]
   
            sum_x.append(float(i))

            for sum_list, y in zip(sum_y_values, y_values):

                sum_list.append(float(y))

        return sum_x, sum_y_values
    
    else:
        
        sum_y_values = [[] for _ in range(count)]

        for line in txt_line:

            y_values = [float(num) for num in line[7:14]]

            for sum_list, y in zip(sum_y_values, y_values):
                sum_list.append(float(y))

        return sum_y_values

def Get_txt(txt_path: str):

    """从当前的log里边获取相关的信息, 主要有两个, 一个是Debug, 一个是Info"""

    sum_x = []

    sum_y = [[] for _ in range(14)]

    with open(txt_path, "r", encoding="utf-8") as files:
        txt_line = files.readlines()

    command, state = my_match(txt_line)

    command_x, command_y = Get_x(txt_line=command,count=8)

    state_y = Get_x(txt_line=state, count=7, num=1)

    state_y.append(command_y[7])

    return command_x, command_y, state_y


def my_plt(command_x, command_y, state_y):

    fig, axs = plt.subplots(3, 3)

    for i, (ax, y_values, z_values) in enumerate(zip(axs.flatten()[:8], state_y, command_y)):

        ax.plot(command_x, y_values, label=f'inference')
        ax.plot(command_x, z_values, label=f'command')
        ax.set_title(f'motor {i+1} state')
        ax.legend()

    axs.flatten()[-1].axis('off')

    plt.subplots_adjust(wspace=0.4, hspace=0.4)

    # plt.savefig(output_file)

    plt.show()

if __name__ == "__main__":


    txt_path = "/home/rebot801/LIuXin/ICCUB_ws/Dataset/时间戳/1721095144.3137858_log"

    command_x, command_y, state_y = Get_txt(txt_path)
    my_plt(command_x, command_y, state_y)

