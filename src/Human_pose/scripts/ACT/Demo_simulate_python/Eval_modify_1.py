import cv2
import torch
import numpy as np
import os
import pickle
import argparse
from einops import rearrange
from utils import set_seed
from utils import Log
from datetime import datetime
from policy_R import ACTPolicy, CNNMLPPolicy
import IPython

e = IPython.embed
# camera_names = ['camera1','camera2']
camera_names = ['camera1']


def get_configs(ckpt_path: str, task_name: str, batch_Size=None, policy_class=None):
    if batch_Size is None:
        batch_Size = 1
    if policy_class is None:
        policy_class = "ACT"

    parser = argparse.ArgumentParser()
    parser.add_argument('--eval', default="eval")
    parser.add_argument('--onscreen_render', action='store_true')

    parser.add_argument('--ckpt_dir', default=ckpt_path)
    parser.add_argument('--policy_class', default=policy_class)

    parser.add_argument('--task_name', default=task_name)
    parser.add_argument('--batch_size', default=batch_Size)
    parser.add_argument('--seed', default=1)
    parser.add_argument('--num_epochs', default=50)
    parser.add_argument('--lr', default=1e-5)

    parser.add_argument('--kl_weight', default=10)
    parser.add_argument('--chunk_size', default=20)
    parser.add_argument('--hidden_dim', default=512)
    parser.add_argument('--dim_feedforward', default=3200)
    parser.add_argument('--temporal_agg', action='store_true', default='temporal_agg')

    Human_config = vars(parser.parse_args())

    return Human_config


# 确认使用哪一个网络，ACT | CNNMLP
def make_policy(policy_class, policy_config, ckpt_dir):
    if policy_class == 'ACT':
        policy = ACTPolicy(policy_config, ckpt_dir)
    elif policy_class == 'CNNMLP':
        policy = CNNMLPPolicy(policy_config)
    else:
        raise NotImplementedError
    return policy


def make_optimizer(policy_class, policy):
    if policy_class == 'ACT':
        optimizer = policy.configure_optimizers()
    elif policy_class == 'CNNMLP':
        optimizer = policy.configure_optimizers()
    else:
        raise NotImplementedError
    return optimizer


def get_image_from_arm(cam):
    # 获取图片--经过修改以后的获取801图片的程序
    # cam--两个，表示图片的获取渠道
    # fix--固定摄像头
    # followed--跟随摄像头

    curr_images = []
    for cam_name in cam:
        curr_image = rearrange(cam_name, 'h w c -> c h w')
        curr_images.append(curr_image)
    curr_image = np.stack(curr_images, axis=0)
    curr_image = torch.from_numpy(curr_image / 255.0).float().cuda().unsqueeze(0)
    return curr_image


def eval_bc(config, ckpt_name, save_episode=True, End_status=None):
    set_seed(1)
    ckpt_dir = config['ckpt_dir']
    state_dim = config['state_dim']
    policy_class = config['policy_class']
    policy_config = config['policy_config']
    temporal_agg = config['temporal_agg']
    ckpt_path = os.path.join(ckpt_dir, ckpt_name)
    policy = make_policy(policy_class, policy_config, ckpt_dir)  # 将detr中的ckpt设置成根据上层ckpt进行变化
    loading_status = policy.load_state_dict(torch.load(ckpt_path))

    policy.cuda()
    policy.eval()

    stats_path = os.path.join(ckpt_dir, f'dataset_stats.pkl')
    with open(stats_path, 'rb') as f:
        stats = pickle.load(f)
    #
    pre_process = lambda s_qpos: (s_qpos - stats['qpos_mean'][0:7]) / stats['qpos_std'][0:7]
    post_process = lambda a: a * stats['action_std'][0:7] + stats['action_mean'][0:7]

    # pre_process = lambda s_qpos: (s_qpos - stats['qpos_mean']) / stats['qpos_std']
    # post_process = lambda a: a * stats['action_std'] + stats['action_mean']

    # 初始化操作，从config中获取需要的变量
    query_frequency = policy_config['num_queries']
    if temporal_agg:
        query_frequency = 1
        num_queries = policy_config['num_queries']

    max_timesteps = int(1)  # may increase for real-world tasks

    num_rollouts = 1

    cam_followed = config[camera_names[0]]

    cam = [cam_followed]

    if (len(camera_names) == 2):
        cam_fixed = config[camera_names[1]]

        cam = [cam_followed, cam_fixed]

    current_joints = config['current_joints']

    # 开始进行inference
    for rollout_id in range(num_rollouts):
        rollout_id += 0

        if temporal_agg:
            all_time_actions = torch.zeros([max_timesteps, max_timesteps + num_queries, state_dim]).cuda()

        qpos_history = torch.zeros((1, max_timesteps, state_dim)).cuda()

        with torch.inference_mode():
            for t in range(max_timesteps):  # chuck 10 max_timestep 100  time=7 current position

                qpos = np.zeros(8)
                target_qpos = np.zeros(8)

                qpos_numpy = current_joints

                """以下两行设置只针对做归一化的函数"""
                qpos[0:7] = pre_process(qpos_numpy[0:7])
                qpos[7] = qpos_numpy[7]
                # ################################################

                # qpos = pre_process(qpos_numpy)

                qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)

                qpos_history[:, t] = qpos

                curr_image = get_image_from_arm(cam)

                # query policy
                if config['policy_class'] == "ACT":
                    if t % query_frequency == 0:
                        all_actions = policy(qpos, curr_image)

                    if temporal_agg:

                        all_time_actions[[t], t:t + num_queries] = all_actions
                        actions_for_curr_step = all_time_actions[:, t]
                        actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
                        actions_for_curr_step = actions_for_curr_step[actions_populated]
                        k = 0.01
                        exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
                        exp_weights = exp_weights / exp_weights.sum()
                        exp_weights = torch.from_numpy(exp_weights).cuda().unsqueeze(dim=1)

                        raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)

                    else:
                        # raw_action = all_actions[:, t % query_frequency]
                        raw_action = all_actions

                elif config['policy_class'] == "CNNMLP":
                    raw_action = policy(qpos, curr_image)
                else:
                    raise NotImplementedError

                """对夹爪单独做，相当于轨迹用时间集成，而夹爪直接用当前预测出来的最大值"""

                raw_action = raw_action.squeeze(0).cpu().numpy()
                all_actions = all_actions.squeeze(0).cpu().numpy()

                arg_max_index = np.argmax(all_actions[:, 7])
                arg_min_index = np.argmin(all_actions[:, 7])

                if End_status is False:

                    action = post_process(all_actions[17, :][0:7])
                    target_qpos[7] = abs(all_actions[arg_max_index, :][7])

                elif End_status is True:

                    action = post_process(all_actions[19, :][0:7])
                    target_qpos[7] = abs(all_actions[arg_min_index, :][7])

                    print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")

                else:
                    action = post_process(raw_action[0:7])

                    target_qpos[7] = abs(all_actions[arg_max_index, :][7])

                    print("********************************")

                target_qpos[0:7] = action

    return target_qpos


def main_ACT(cam_followed, current_joints, cam_fixed=None, End_status=None):

    # args = get_configs(ckpt_path='/home/rebot801/LIuXin/ckpt/ckpt-2024.07.28/ckpt_batch筛选', task_name="801")  # 目前来看最稳定

    args = get_configs(ckpt_path= '/home/rebot801/LIuXin/ckpt/ckpt_2024.07.31/ckpt', task_name="801")

    set_seed(1)
    # command line parameters
    is_eval = args['eval']
    ckpt_dir = args['ckpt_dir']
    policy_class = args['policy_class']
    onscreen_render = args['onscreen_render']
    task_name = args['task_name']
    num_epochs = args['num_epochs']
    # camera_names = ['camera1', 'camera2']
    camera_names = ['camera1']

    # fixed parameters
    state_dim = 8
    lr_backbone = 1e-5
    backbone = 'resnet18'
    if policy_class == 'ACT':
        enc_layers = 4
        dec_layers = 7
        nheads = 8
        policy_config = {'lr': args['lr'],
                         # ATTENTION_HERE
                         'num_queries': args['chunk_size'],
                         'kl_weight': args['kl_weight'],
                         'hidden_dim': args['hidden_dim'],
                         'dim_feedforward': args['dim_feedforward'],
                         'lr_backbone': lr_backbone,
                         'backbone': backbone,
                         'enc_layers': enc_layers,
                         'dec_layers': dec_layers,
                         'nheads': nheads,
                         'camera_names': camera_names,
                         }
    elif policy_class == 'CNNMLP':
        policy_config = {'lr': args['lr'], 'lr_backbone': lr_backbone, 'backbone': backbone, 'num_queries': 1,
                         'camera_names': camera_names, }
    else:
        raise NotImplementedError

    config = {
        'num_epochs': num_epochs,
        'ckpt_dir': ckpt_dir,
        'state_dim': state_dim,
        'lr': args['lr'],
        'policy_class': policy_class,
        'onscreen_render': onscreen_render,
        'policy_config': policy_config,
        'task_name': task_name,
        'seed': args['seed'],
        'temporal_agg': args['temporal_agg'],
        'camera_names': camera_names,
        'camera1': cam_followed,
        'camera2': cam_fixed,
        'current_joints': current_joints

    }

    # Log("here")
    if is_eval:
        ckpt_names = [f'policy_best.ckpt']
        results = []

        for ckpt_name in ckpt_names:
            target_pose = eval_bc(config=config, ckpt_name=ckpt_name, save_episode=True, End_status=End_status)
            results.append([ckpt_name, target_pose])

    return target_pose


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--eval', action='store_true')
    parser.add_argument('--onscreen_render', action='store_true')
    parser.add_argument('--ckpt_dir', default='/home/rebot801/LIuXin/ckpt')
    parser.add_argument('--policy_class', default="ACT")
    parser.add_argument('--task_name', default="801")
    parser.add_argument('--batch_size', default=32)
    parser.add_argument('--seed', default=0)
    parser.add_argument('--num_epochs', default=50)
    parser.add_argument('--lr', default=1e-5)

    parser.add_argument('--kl_weight', default=10)
    parser.add_argument('--chunk_size', default=10)
    parser.add_argument('--hidden_dim', default=512)
    parser.add_argument('--dim_feedforward', default=3200)
    parser.add_argument('--temporal_agg', action='store_true')
    _801_config = vars(parser.parse_args())
    Log(_801_config)
    cam_followed = cv2.imread(
        "/home/rebot801/wangwei/数据集/dest/1719386659.1461983_Data/camera/1719386667.7267902.png")

    current_joints = [8.611633315157817, -10.786409219932523, -68.07353859109551, -61.297710035021495,
                      11.833523627540732, -18.982591290446493, 4.360230263035246, 0]

    cc = main_ACT(cam_followed, current_joints)

## python3 imitate_episodes_801_eval_server_modify_socket.py --task_name 801 --ckpt_dir /media/smj/新加卷/LiuXin_aloha/801_single_fix/ckpt_current/ --policy_class ACT --kl_weight 10 --chunk_size 10 --hidden_dim 512 --batch_size 1 --dim_feedforward 3200 --num_epochs 500  --lr 1e-5 --seed 0 --eval --temporal_agg
