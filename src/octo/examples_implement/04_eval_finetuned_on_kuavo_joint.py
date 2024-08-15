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
from dynamic_biped.msg import robotArmInfo
from dynamic_biped.srv import controlEndHand, controlEndHandRequest
from sensor_msgs.msg import JointState
from cam import camera_init,get_rgb

import math
import rospy
from typing import List
import pyrealsense2 as rs
from collections import deque
from dynamic_biped.msg import robotHandPosition        # 机械臂关节的角度和信息
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
flags.DEFINE_integer("num_timesteps", 200, "num timesteps",required=False)
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
        # cv2.imshow("image_primary", color_image01)
        # cv2.waitKey(1)
        cv2.imwrite("check.png",color_image01)
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
        robot_instance.set_arm_traj_position(rad_to_angle([-0.3261288947090802, 0.01811530772712316, 0.2115283417884367, -0.586518085244565, -0.2103761430854162, -0.6746457384525815, 0.12379741046503845, -0.012431532204051734, -0.05588632419706121, 0.047875332916695726, -0.05817196796461878, -0.0093349926801437, -0.08298794124956278, -0.0032379768505814486]))
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
        img=Image.open("/home/lab/hx/ICCUB_ws/src/octo/examples/last_pic/pick_up_something_2024-08-10-21-29-43_last_img.png")
        # goal_image02=cv2.imread("/home/rebot801/LIuXin/ICCUB_ws/src/octo/examples/goal_secondary.png",cv2.IMREAD_COLOR)
        goal_image01=img.resize((256,256))
        # goal_image02=cv2.resize(np.array(goal_image02),(128,128))

        text="Pick up the bottle and place it next to it."
        # task = model.create_tasks(texts=[text])
        task = model.create_tasks(texts=[text],goals={"image_primary": goal_image01[None]})
        # task = model.create_tasks(texts=[text],goals={"image_primary": goal_image01[None],"image_secondary":goal_image02[None]})


        
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
        last_tstep = time.time()
        already_grab=0
        while ts < FLAGS.num_timesteps:
            if time.time() > last_tstep + STEP_DURATION:
                last_tstep = time.time()
                
                action_proprio_metadata=model.dataset_statistics["proprio"]
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
                action = np.array(policy_fn(obs, task), dtype=np.float64)
                
                # all_actions=action[::5]
                import random
                # select_index=random.randint(20,30)
                # select_index=random.randint(0,0)
                for select_index in range(2):
                    hand_state=action[select_index][7]
                    print("hand_state",hand_state)

                    select_action=rad_to_angle(action[select_index])
                    print("forward pass time: ", time.time() - forward_pass_time) #0.04s/0.07s
                
                    print(select_action)

                    all_joint= [0] * 14
                    all_joint[0:7]=select_action[:7]

                    # perform environment step
                    start_time = time.time()
                    #=========================================================
                    if not TEST_MODE:
                        robot_instance.set_arm_traj_position(all_joint)
                        if already_grab==1:
                            call_control_end_hand_service(left_hand_position=[0, 30, 80, 80, 80, 80], right_hand_position=[0, 0, 0, 0, 0, 0])
                        else:
                            grap=hand_state
                            if grap>0.5:
                                already_grab=1
                                call_control_end_hand_service(left_hand_position=[0, 30, 80, 80, 80, 80], right_hand_position=[0, 0, 0, 0, 0, 0])
                            else:
                                call_control_end_hand_service(left_hand_position=[0, 70, 20, 20, 20, 20], right_hand_position=[0, 0, 0, 0, 0, 0])
                    #=========================================================
                    print("step time: ", time.time() - start_time)
                    time.sleep(0.5)
                    print(ts,"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n")
                    ts += 1

        print("done")
        break

if __name__ == "__main__":
    TEST_MODE = 0
    if not TEST_MODE:
        rospy.init_node('demo_test')
        robot_instance = kuavo("3_7_kuavo")
        rospy.Subscriber("/robot_hand_position", robotHandPosition, get_gripper_callback)
    pipeline1=camera_init()
    # app.run(main)
    
    from std_msgs.msg import Float32MultiArray
    eef_pub = rospy.Publisher('/drake_ik/target_LHandEef', Float32MultiArray, queue_size=10)
    msg = Float32MultiArray()
    xyz=[0.13783123712250098, 0.38179042783097505, 0.11421566243091555]
    xyzw=[0.2820704950269744, -0.33308091937969525, -0.6005782088951896, 0.669924736056716]
    from scipy.spatial.transform import Rotation as R
    rotation = R.from_quat(xyzw)
    # 转换为欧拉角 (默认是 'xyz' 顺序，单位是弧度)
    euler_angles = rotation.as_euler('xyz')
    xyzrpy=np.concatenate((xyz,euler_angles))
    data=np.array(xyzrpy, dtype=np.float32)       
    msg.data = data.tolist()
    
    while 1 :
        eef_pub.publish(msg)
        rospy.loginfo("Publishing: %s", msg.data)