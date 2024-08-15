import matplotlib.pyplot as plt
import re

with open('/home/rebot801/LIuXin/ICCUB_ws/Dataset/1720016821.595169_Data/state/1720016821.595169_state.txt', 'r', encoding='utf-8') as file:
    lines = file.readlines()

sum_x = []
sum_y_values = [[] for _ in range(14)]

for line in lines:
    if "当前是state" in line or line.strip() == '':
        continue

    match = re.search(r'\[(.*?)\] \[(.*?)\]', line)
    if match is None:
        print(f'line: {line} is not matched')
        continue

    x = match.group(1)
    y_values = match.group(2).split(',')

    sum_x.append(float(x))
    for sum_list, y in zip(sum_y_values, y_values):

        sum_list.append(float(y))

print("*****************************************************")

# 下面这段代码，把注释解开之后，文件名改一下，就可以在一个曲线图上画两条线，一个是kuavo_arm_traj，一个是robot_amr_q_v_tau
# ********************************************************************************************************************

with open('/home/rebot801/LIuXin/ICCUB_ws/Dataset/1720016821.595169_Data/command/1720016821.595169_command.txt', 'r', encoding='utf-8') as file_2:
    lines_2 = file_2.readlines()

sum_x_2 = []
sum_z_values = [[] for _ in range(14)]

for line_2 in lines_2:
    if "当前是command" in line_2:
        continue

    match = re.search(r'\[(.*?)\] \[(.*?)\]', line_2)


    x = match.group(1)
    y_values = match.group(2).split(',')

    sum_x_2.append(float(x))
    for sum_list, y in zip(sum_z_values, y_values):
        sum_list.append(float(y))

print("***************************************************************************************************")

# ********************************************************************************************************************

all_values = sum_x + sum_x_2
min_x = min(all_values)
max_x = max(all_values)

sum_x = [(x - min_x) / (max_x - min_x) * 1000 for x in sum_x]
sum_x_2 = [(x - min_x) / (max_x - min_x) * 1000 for x in sum_x_2]

fig, axs = plt.subplots(3, 5)
# 下面这段代码，把注释解开之后，就可以在一个曲线图上画两条线，一个是kuavo_arm_traj，一个是robot_amr_q_v_tau
# for i, (ax, y_values) in enumerate(zip(axs.flatten()[:14], sum_y_values)):
for i, (ax, y_values, z_values) in enumerate(zip(axs.flatten()[:14], sum_y_values, sum_z_values)):
    ax.plot(sum_x, y_values, label=f'state')
    ax.plot(sum_x_2, z_values, label=f'command')
    ax.set_title(f'motor {i+1} state')
    ax.legend()

axs.flatten()[-1].axis('off')

plt.subplots_adjust(wspace=0.4, hspace=0.4)
# plt.figure(figsize=(16, 16))

plt.show()