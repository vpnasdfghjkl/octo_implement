import sys
import time

num =1

while 1:

    if num > 10:
        print("***************")
        sys.exit()

    print("^^^^^^^^^^^^^^^^^^")
    time.sleep(1)
    print(num)

    num = num + 1

