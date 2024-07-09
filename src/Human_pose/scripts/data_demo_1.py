import matplotlib.pyplot as plt
import re
import os

src_path = "/home/rebot801/LIuXin/ICCUB_ws/Dataset788"

def get_txt(txt_path: str):

    sum_x = []

    sum_y = [[] for _ in range(14)]

    with open(txt_path, 'r', encoding='utf-8') as files:

        txt_line = files.readlines()

    for line in txt_line:

        if "当前是state" in line or "当前是command" in line or line.strip() == '':
            continue

        match = re.search(r'\[(.*?)\] \[(.*?)\]', line)

        if match is None:
            print(f'line: {line} is not matched')
            continue

        # 匹配X和Y，并将其复制到sum里边

        X = match.group(1)
        sum_x.append(float(X))

        Y = match.group(2).split(',')

        for sum_list, y in zip(sum_y, Y):

            sum_list.append(float(y))
    return sum_x, sum_y


for file_path in os.listdir(src_path):
    try:

        data_time = os.path.basename(file_path).split('_')[0]

        command_path = f'{src_path}/{file_path}/command/{data_time}_command.txt'

        stand_path = f'{src_path}/{file_path}/state/{data_time}_state.txt'

        X_command, Y_command = get_txt(command_path)

        X_state, Y_state = get_txt(stand_path)

        # 将两个值拼接起来，取最大值，并做归一化
        all_values = X_command + X_state
        min_x = min(all_values)
        max_x = max(all_values)

        x_command = [(x - min_x) / (max_x - min_x) * 1000 for x in X_command]
        x_state = [(x - min_x) / (max_x - min_x) * 1000 for x in X_state]

        fig, axs = plt.subplots(3, 5)


        for i, (ax, y_values, z_values) in enumerate(zip(axs.flatten()[:14], Y_command, Y_state)):

            ax.plot(x_command, y_values, label=f'state')
            ax.plot(x_state, z_values, label=f'command')
            ax.set_title(f'motor {i + 1} state')
            ax.legend()

        axs.flatten()[-1].axis('off')

        plt.subplots_adjust(wspace=0.4, hspace=0.4)

        plt.show()
        plt.rcParams["figure.figsize"] = [20, 10]

        plt.savefig(f'{file_path}.png')
        print(file_path)

    except Exception as error:

        print("当前的错误文件夹如下", file_path)
