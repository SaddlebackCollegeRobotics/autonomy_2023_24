
import numpy as np
import math
from time import sleep

def main():

    current_pos = np.array([0,0]) 
    target_pos = np.array([10,-10])

    current_rot = 0

    direction_vector = target_pos - current_pos
    target_rot = math.degrees(math.atan2(direction_vector[1], direction_vector[0]))

    print(target_rot)

    while (True):

        rot_dir = np.sign([target_rot - current_rot])[0]

        print(current_rot)
        
        current_rot += rot_dir
        sleep(0.1)
        



if __name__ == '__main__':
    main()
