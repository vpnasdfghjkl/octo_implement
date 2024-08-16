import subprocess
result = subprocess.run(['rosbag', 'play', '/home/lab/hx/rosbag_WRC_juice3/juice_3_begin.bag'], capture_output=True, text=True)
