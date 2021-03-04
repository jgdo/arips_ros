import glob
import os
import shutil

from annot_utils import *

all_folder = '/home/jgdo/catkin_ws/src/arips_ros/door_handle_detection/data/all/'
handle_folder = '/home/jgdo/catkin_ws/src/arips_ros/door_handle_detection/data/with_handle/'
no_folder = '/home/jgdo/catkin_ws/src/arips_ros/door_handle_detection/data/no_handle/'

jpgs = list(sorted(glob.glob(all_folder + "*.jpg")))

for img in jpgs:
    annot = getAnnotationName(img)
    if not os.path.exists(annot):
        print("Skipping unannotated {}".format(img))
        continue

    print("Copying {}".format(img))
    data = loadAnnotation(annot)
    target_folder = handle_folder if data[2] > 0 else no_folder
    shutil.copy(img, os.path.join(target_folder, os.path.basename(img)))
    shutil.copy(annot, os.path.join(target_folder, os.path.basename(annot)))
