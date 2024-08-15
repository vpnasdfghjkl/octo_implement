#!/bin/bash

# SSH登录脚本

# 定义远程服务器的域名、端口、用户名和密码
HOST="tencent.wangwanyou.xyz"
PORT="10002"
USERNAME="hit"
PASSWORD="cT9*cI8/nY7[fK9*"
USERNAME="octo"
PASSWORD="HIT-J1513"
# 使用sshpass工具进行密码认证
# 确保sshpass已经安装在你的系统上
sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=20 -p $PORT $USERNAME@$HOST

# root
# sY0>uR9%bP7-mF4!