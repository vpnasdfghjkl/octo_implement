"""
This script shows how we evaluated a finetuned Octo model on a real WidowX robot. While the exact specifics may not
be applicable to your use case, this script serves as a didactic example of how to use Octo in a real-world setting.

If you wish, you may reproduce these results by [reproducing the robot setup](https://rail-berkeley.github.io/bridgedata/)
and installing [the robot controller](https://github.com/rail-berkeley/bridge_data_robot)
"""

from datetime import datetime
from functools import partial
import os
import time

from absl import app, flags, logging
import click
import cv2
# from envs.widowx_env import convert_obs, state_to_eep, wait_for_obs, WidowXGym
import imageio
import jax
import jax.numpy as jnp
import numpy as np
# from widowx_envs.widowx_env_service import WidowXClient, WidowXConfigs, WidowXStatus

from octo.model.octo_model import OctoModel
from octo.utils.gym_wrappers import HistoryWrapper, TemporalEnsembleWrapper
from octo.utils.train_callbacks import supply_rng

#============================================================add kuavo lib
from kuavoRobotSDK import kuavo
from dynamic_biped.msg import robotArmInfo,recordArmHandPose
from dynamic_biped.srv import controlEndHand, controlEndHandRequest
from sensor_msgs.msg import JointState
from cam import camera_init,get_rgb

import math
import rospy
from typing import List
import pyrealsense2 as rs
from collections import deque
from dynamic_biped.msg import robotHandPosition        # 机械臂关节的角度和信息
from std_msgs.msg import Float32MultiArray
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
#============================================================
'''
python octo/examples/04_eval_finetuned_on_kuavo.py --checkpoint_weights_path=/media/lab/PortableSSD/0808_evening 
'''
np.set_printoptions(suppress=True)

logging.set_verbosity(logging.WARNING)

FLAGS = flags.FLAGS

flags.DEFINE_string("checkpoint_weights_path", None, "Path to checkpoint", required=True)
flags.DEFINE_integer("checkpoint_step", None, "Checkpoint step", required=False)
flags.DEFINE_string("start_traj_txt_path", None, "start_traj_txt_path", required=False)

flags.DEFINE_integer("im01_size", 256, "Image size", required=False)
flags.DEFINE_integer("im02_size", 128, "Image size", required=False)

flags.DEFINE_bool("proprio", False, "is there primary", required=False)
flags.DEFINE_bool("primary", True, "is there primary", required=False)
flags.DEFINE_bool("secondary", False, "is there secondary", required=False)

# flags.DEFINE_string("video_save_path", None, "Path to save video",required=False)
flags.DEFINE_integer("num_timesteps", 20000, "num timesteps",required=False)
flags.DEFINE_integer("window_size", 2, "Observation history length",required=False)
flags.DEFINE_integer(
    "action_horizon", 8, "Length of action sequence to execute/ensemble"
)


# show image flag
flags.DEFINE_bool("show_image", False, "Show image")
#python  --checkpoint_weights_path="/home/robot/work/yu_code/ckpt" --start_traj_txt_path="/home/robot/work/yu_code/traj_added.txt" 
##############################################################################

STEP_DURATION_MESSAGE = """
Bridge data was collected with non-blocking control and a step duration of 0.2s.
However, we relabel the actions to make it look like the data was collected with
blocking control and we evaluate with blocking control.
Be sure to use a step duration of 0.2 if evaluating with non-blocking control.
"""
STEP_DURATION = 0.1
STICKY_GRIPPER_NUM_STEPS = 1
WORKSPACE_BOUNDS = [[-180, -5, -90, -90, -90, -90, -90, -180, -135, -90, -90, -90, -90, -90], 
                    [30, 135, 90, 0, 90, 90, 90,  30, 5,  90, 0, 90, 90, 90]]
ARM_TOPICS = [{"name": "/robot_arm_q_v_tau"}]
ENV_PARAMS = {
    "robot_arm_q_v_tau": ARM_TOPICS,
    "override_workspace_boundaries": WORKSPACE_BOUNDS,
    "move_duration": STEP_DURATION,
}
GRIPPER_OPEN_STATE = "[0, 30, 0, 0, 0, 0, 0, 30, 0, 0, 0, 0]"
GRIPPER_CLOSE_STATE = "[30, 30, 90, 90, 90, 90, 30, 30, 90, 90, 90, 90]"

##############################################################################
state=None
gripper=None
cmd_eef=None
state_eef=None

def rad_to_angle(rad_list: list) -> list:
    """弧度转变为角度"""
    angle_list = [0 for _ in range(len(rad_list))]
    for i, rad in enumerate(rad_list):
        angle_list[i] = rad / math.pi * 180.0
    return angle_list   


def get_gripper_callback(msg):
    global gripper
    left_hand_position = msg.left_hand_position
    right_hand_position = msg.right_hand_position
    hand_position = left_hand_position + right_hand_position
    gripper=str(list(hand_position))
    
def get_cmd_eef_callback(msg):
    global cmd_eef
    xyz=np.array(msg.left_pose.pos_xyz)
    xyzw=np.array(msg.left_pose.quat_xyzw)
    rotation = R.from_quat(xyzw)
    euler_angles = rotation.as_euler('xyz')
    cmd_eef=np.concatenate((xyz,euler_angles))
    # print("cmd_eef=",cmd_eef)

def get_state_eef_callback(msg):
    global state_eef
    xyz=np.array(msg.left_pose.pos_xyz)
    xyzw=np.array(msg.left_pose.quat_xyzw)
    rotation = R.from_quat(xyzw)
    euler_angles = rotation.as_euler('xyz')
    state_eef=np.concatenate((xyz,euler_angles))
    # print("state_eef=",state_eef)

def call_control_end_hand_service(left_hand_position: List[float], right_hand_position: List[float]):
    """控制爪子开合的服务"""
    hand_positions = controlEndHandRequest()
    hand_positions.left_hand_position = left_hand_position  # 左手位置
    hand_positions.right_hand_position = right_hand_position  # 右手位置
    try:
        rospy.wait_for_service('/control_end_hand')
        control_end_hand = rospy.ServiceProxy('/control_end_hand', controlEndHand)
        resp = control_end_hand(hand_positions)
        return resp.result
    except rospy.ROSException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False


def go_to_start(robot_instance):
    start_traj_txt_path=FLAGS.start_traj_txt_path
    with open(start_traj_txt_path, 'r') as f:
        lines = f.readlines()
        # example line:[1721453954.4682379] [-0.011260911328125012, -0.032774646324176294, 0.010934900999376768, -0.010906881191067398, -3.92324055631966, 0.6438061314323509, -4.359844068453743, -9.344588960334962, -14.195518408431509, -41.38598418663083, -43.33137265347529, 4.645285919214266, -33.10302521671483, 3.5093105509978906]
        start_traj = []
        for line in lines:
            import re
            pattern=r'\[(.*?)\]'
            matchs=re.findall(pattern,line)
            array_str=matchs[1]
            line = array_str[1:-1]
            line = line.split(", ")
            start_traj.append([float(x) for x in line])

    for step in start_traj:
        # all_joint= [0] * 14
        # all_joint[7:14]=step[:7]
        # print("***")
        robot_instance.set_arm_traj_position(step)
        # print("###")

        time.sleep(0.015)

def normalize(data, metadata):
    mask = metadata.get("mask", np.ones_like(metadata["mean"], dtype=bool))
    return np.where(
        mask,
        (data - np.array(metadata["mean"])) / (np.array(metadata["std"]) + 1e-8),
        data,
    )

def get_info(history_dict,ts,action_proprio_metadata):
    global gripper
    obs=dict()
    if "proprio" in history_dict.keys():
        if not TEST_MODE:
            state=robot_instance.latest_RobotstatePosition
        else:
            state=np.random.rand(14)
        print("#################",gripper)
        gripper01=10

        if gripper==GRIPPER_OPEN_STATE:
            gripper01=0
        elif gripper==GRIPPER_CLOSE_STATE:
            gripper01=1

        one_hand = state[0:7]
        state = np.append(one_hand, gripper01)
        # normolize state
        state =normalize(state,action_proprio_metadata)
        
        if FLAGS.window_size==2:
            if ts!=0:
                all_proprio = list(history_dict['proprio'])+[state] 
            else:
                all_proprio = list([state]+[state])
        elif FLAGS.window_size==1:
            all_proprio = list([state])
        obs['proprio'] = np.stack(all_proprio)

    if "image_primary" in history_dict.keys():
        frames01=pipeline1.wait_for_frames()
        color_frame01 = frames01.get_color_frame()
        color_image01 = np.asanyarray(color_frame01.get_data())

        if FLAGS.show_image:
            # plt.imshow(color_image01)
            # plt.savefig("check_plt.png")
            # plt.show()
            # plt.pause(1)
            bgr_image = cv2.cvtColor(color_image01, cv2.COLOR_RGB2BGR)
            cv2.imshow('BGR Image', bgr_image)
            cv2.waitKey(1)

        current_image01=cv2.resize(np.array(color_image01),(FLAGS.im01_size,FLAGS.im01_size))
        if FLAGS.window_size==2:
            if ts!=0:
                all_images01 = list(history_dict['image_primary'])+[current_image01]
                obs['timestep_pad_mask']=np.array([True,True])
            else:
                all_images01 = list([current_image01]+[current_image01])
                obs['timestep_pad_mask']=np.array([False,True])
        elif FLAGS.window_size==1:
            all_images01=list([current_image01])
            obs['timestep_pad_mask']=np.array([True])

        obs['image_primary'] = np.stack(all_images01)
        # obs['timestep_pad_mask']=np.full((obs['image_primary'].shape[0]), True, dtype=bool)

    if "image_secondary" in history_dict.keys():
        frames02 = pipeline2.wait_for_frames()
        color_frame02 = frames02.get_color_frame()
        color_image02 = np.asanyarray(color_frame02.get_data())
        cv2.imshow("image_secondary", color_image02)
        cv2.waitKey(1)

        current_image02=cv2.resize(np.array(color_image02),(FLAGS.im01_size,FLAGS.im01_size))
        if FLAGS.window_size==2:
            if ts!=0:
                all_images02 = list(history_dict['image_secondary'])+[current_image02]
                obs['timestep_pad_mask']=np.array([True,True])
            else:
                all_images02 = list([current_image02]+[current_image02])    
                obs['timestep_pad_mask']=np.array([False,True])
        elif FLAGS.window_size==1:
            all_images02=list([current_image02])
            obs['timestep_pad_mask']=np.array([True])
        obs['image_secondary'] = np.stack(all_images02)
        # obs['timestep_pad_mask']=np.full((obs['image_secondary'].shape[0]), True, dtype=bool)
        assert obs['image_secondary'].shape[0]==obs['image_primary'].shape[0] 
    return obs

def main(_):
    global cmd_eef,state_eef
    # load models
    print(f"loading model {FLAGS.checkpoint_step}......")
    model = OctoModel.load_pretrained(
        FLAGS.checkpoint_weights_path,
        FLAGS.checkpoint_step,
    )
    print("#############################")
    # wrap the robot environment    
    if not TEST_MODE:
        # go_to_start(robot_instance)
        # robot_instance.set_arm_traj_position(rad_to_angle([-0.20478183873824504, 0.35038397443912594, -0.08983954441583523, -1.0378029064231027, 0.3076629727825504, -0.4968868614783442, -0.06390367491100628, -0.009941041141309736, -0.06084704514836867, 0.046350946082915206, -0.04596946889977211, -0.010105169683585763, -0.08030962587205101, -0.002484736483884915]))
        # robot_instance.set_arm_traj_position(rad_to_angle([0.07654063876612716, 0.3896756013744171, 0.23861315834852517, -1.2239652308027662, -1.1400463740847107, -0.3259918692717774, -0.3725070651525045, -0.012679085784232396, -0.23289101999893283, 0.016595147937879157, 0.01201667800747805, 0.01316203147402272, -0.09366555498369236, 0.012013296334962993]))
        # robot_instance.set_arm_traj_position(rad_to_angle([-0.12972190316347956, 0.2481504023530134, 0.3267348231911655, -0.9626532166944664, -1.344521948527243, -0.39998260979532796, -0.4594819781554815, -0.012665699156471774, -0.23288873404360166, 0.01659332160836448, 0.01201692164983737, 0.013545224215471703, -0.09326857302262562, 0.011636985085326798]))
        # robot_instance.set_arm_traj_position(rad_to_angle([-0.10185799358730473, 0.2565439918630544, 0.30727905668020195, -0.9988939184810046, -1.344509005358521, -0.3992203308928522, -0.46026802882319895, -0.012665024364732836, -0.23289365504503357, 0.016595872466927827, 0.01201740357167308, 0.013163451868615138, -0.09288226737091004, 0.012028623554987085])) # grab orange 200 epidodes
        robot_instance.set_arm_traj_position(rad_to_angle( [0.02299491847040674, 0.7356761380119029, -0.2058058888967117, -1.3384070847451883, -0.738727460861821, -0.21610934770334628, -0.3385586497193961, -0.028488898797332865, -0.039101431080773394, -0.07381675320897092, 0.0005727149417722595, 0.12035521374958336, -0.13065499446613835, 0.1066232488068308]))
        call_control_end_hand_service(left_hand_position=list(map(int, GRIPPER_OPEN_STATE[1:-1].split(", ")))[:6], right_hand_position=[0, 0, 0, 0, 0, 0])
    # create policy functions
    def sample_actions(
        pretrained_model: OctoModel,
        observations,
        tasks,
        rng,
    ):
        # add batch dim to observations
        observations = jax.tree_map(lambda x: x[None], observations)
        actions = pretrained_model.sample_actions(
            observations,
            tasks,
            rng=rng,
            unnormalization_statistics=pretrained_model.dataset_statistics["action"],
        )
        # remove batch dim
        return actions[0]

    policy_fn = supply_rng(
        partial(
            sample_actions,
            model,
            # argmax=FLAGS.deterministic,
            # temperature=FLAGS.temperature,
        )
    )

    # goal sampling loop
    while True:
        #====================
        #
        # create task
        #
        #====================
        from PIL import Image
        # img=Image.open("/home/lab/hx/ICCUB_ws/src/octo/examples/last_pic/pick_up_something_2024-08-10-21-29-43_last_img.png") # fruit last img
        img=Image.open("/home/lab/hx/ICCUB_ws/src/octo/examples/last_pic/pick_up_something_2024-08-13-20-19-56_last_img.png") # juice last img
        # goal_image01=img.resize((256,256))
        
        goal_image01=cv2.resize(np.array(img),(256,256))
        plt.imshow(goal_image01)
        plt.savefig("./goal_image01.png")
        # text="Pick up the orange and place it in the yellow plate."
        text="Put Orange into Juicer Container."
        # task = model.create_tasks(texts=[text])
        task = model.create_tasks(texts=[text],goals={"image_primary": goal_image01[None]})


        
        #====================
        #
        # create history window deque
        #
        #====================
        ts = 0
        history_dict = {}
        if FLAGS.proprio:
            history_dict["proprio"] = deque(maxlen=FLAGS.window_size-1)
        if FLAGS.primary:
            history_dict["image_primary"] = deque(maxlen=FLAGS.window_size-1)
        if FLAGS.secondary:
            history_dict["image_secondary"] = deque(maxlen=FLAGS.window_size-1)
        

        #====================
        #
        # do rollout
        #
        #====================
        action_proprio_metadata=model.dataset_statistics["proprio"]
        last_tstep = time.time()
        while ts < FLAGS.num_timesteps:
            print("diff_time===========>",time.time()-last_tstep)
            if time.time() > last_tstep + STEP_DURATION:
                last_tstep = time.time()

                obs=get_info(history_dict,ts,action_proprio_metadata)

                # rec history 
                if "proprio" in history_dict.keys():
                    history_dict["proprio"].append(obs["proprio"][0])
                    print("cur_joint",obs["proprio"][0])
                if "image_primary" in history_dict.keys():
                    history_dict["image_primary"].append(obs["image_primary"][0])
                if "image_secondary" in history_dict.keys():
                    history_dict["image_secondary"].append(obs["image_secondary"][0])

                # get action
                forward_pass_time = time.time()
                action = np.array(policy_fn(obs, task), dtype=np.float32)
                print("forward pass time: ", time.time() - forward_pass_time) #0.04s/0.07s
                
                loot_time = time.time()
                for select_index in range(8):
                    msg = Float32MultiArray()
                    l_arm_delte_eef,hand_state = action[select_index][:6],action[select_index][6]

                    cur_cmd_eef=None
                    if TEST_MODE:
                        cur_cmd_eef=np.array([0.5,0.5,0.5,0.5,0.5,0.5])
                    if not TEST_MODE and ts==0:
                        cur_cmd_eef=state_eef
                    else:
                        cur_cmd_eef=cmd_eef

                    data=cur_cmd_eef+l_arm_delte_eef
                    print("cur_cmd_eef=",cmd_eef,"\ndelta_eef",l_arm_delte_eef,"\nhand_state=",hand_state)
                    print("data=",data)
                    msg.data = data.tolist()
                    if not TEST_MODE:
                        # perform environment step
                        start_time = time.time()
                        eef_pub.publish(msg)
                        rospy.loginfo("Publishing: %s", msg.data)
                        if hand_state>0.8:
                            call_control_end_hand_service(left_hand_position=list(map(int, GRIPPER_CLOSE_STATE[1:-1].split(", ")))[:6], right_hand_position=[0, 0, 0, 0, 0, 0])
                        else:
                            call_control_end_hand_service(left_hand_position=list(map(int, GRIPPER_OPEN_STATE[1:-1].split(", ")))[:6], right_hand_position=[0, 0, 0, 0, 0, 0])
                        print("step time: ", time.time() - start_time)
                    
                        print(ts,"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n")
                        ts += 1
                print("inner loop_time: ", time.time() - loot_time)
                    
        print("done")
        break

if __name__ == "__main__":
    TEST_MODE = 0
    rospy.init_node('demo_test')
    robot_instance = kuavo("3_7_kuavo")
    rospy.Subscriber("/robot_hand_position", robotHandPosition, get_gripper_callback)
    rospy.Subscriber("/drake_ik/cmd_arm_hand_pose",recordArmHandPose,get_cmd_eef_callback)
    rospy.Subscriber("/drake_ik/real_arm_hand_pose",recordArmHandPose,get_state_eef_callback)
    eef_pub = rospy.Publisher('/drake_ik/target_LHandEef', Float32MultiArray, queue_size=10)
    
    pipeline1=camera_init()
    app.run(main)
