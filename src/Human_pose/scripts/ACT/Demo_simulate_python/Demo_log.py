"""
 @LiuXin - 2023.03.12
    日志记录的源程序，通过日志程序调用，记录程序的相关信息
"""

import logging
import time
import os


# 创建一个日志类
class DemoLog:

    def __init__(self) -> None:
        pass

    def log(self):

        # 创建一个日志器
        logger = logging.getLogger("logger")

        # 设置日志的最低输出等级
        logger.setLevel(logging.DEBUG)

        # 创建一个处理器
        sh = logging.StreamHandler()  # 创建一个处理器处理流数据

        # log_path = f'/home/rebot801/LIuXin/ICCUB_ws/src/Human_pose/scripts/ACT/Demo_simulate_python/log/{time.time()}_log'

        # if not os.path.isdir(log_path):
        #     os.makedirs(log_path)

        # fh = logging.FileHandler(filename=, encoding="utf-8")  # 创建一个处理器保存到文本数据中

        fh = logging.FileHandler(filename="/home/rebot801/LIuXin/ICCUB_ws/src/Human_pose/scripts/ACT/Demo_simulate_python/ACT_log/{}_log".format(time.time()),encoding="utf-8")  #创建一个处理器保存到文本数据中

        # 创建一个格式器
        formator = logging.Formatter(fmt='[%(levelname)s] [%(filename)s] [%(asctime)s] [%(message)s]')
        # 将格式器添加到处理器中
        sh.setFormatter(formator)
        fh.setFormatter(formator)

        logger.addHandler(sh)
        logger.addHandler(fh)

        return logger
