import cv2
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
for i in range(10):
    print(i)
    img=cv2.imread("/home/lab/hx/ICCUB_ws/src/octo/examples/last_pic/pick_up_something_2024-08-10-21-29-43_last_img.png")
    img=Image.open("/home/lab/hx/ICCUB_ws/src/octo/examples/last_pic/pick_up_something_2024-08-10-21-29-43_last_img.png")
    img=img.resize((55,55))
    img=cv2.resize(np.array(img),(55,55))
    # save pic use pil
    # import os
    # cur_dir=os.path.dirname(os.path.abspath(__file__))
    # img.save(f"{cur_dir}/test.png")
    plt.imshow(img)
    plt.show()
    plt.pause(1)