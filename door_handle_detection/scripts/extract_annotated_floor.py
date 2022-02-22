import glob
import os
import shutil
import cv2

from annot_utils import *

all_folder = '/home/jgdo/catkin_ws/src/arips_data/datasets/floor/all/'
pos_folder = '/home/jgdo/catkin_ws/src/arips_data/datasets/floor/annotated_scaled_pos/'
neg_folder = '/home/jgdo/catkin_ws/src/arips_data/datasets/floor/annotated_scaled_neg/'

jpgs = list(sorted(glob.glob(all_folder + "*.jpg")))

for img_path in jpgs:
    annot = getAnnotationName(img_path)
    if not os.path.exists(annot):
        print("Skipping unannotated {}".format(img_path))
        continue

    print("Copying {}".format(img_path))

    # shutil.copy(img, os.path.join(target_folder, os.path.basename(img)))
    data = loadAnnotation(annot)
    target_folder = pos_folder if data[2] > 0 else neg_folder

    img = cv2.imread(img_path)
    h_factor = 288 / img.shape[0]
    w_factor = 384 / img.shape[1]
    img = cv2.resize(img, (384, 288), interpolation=cv2.INTER_LINEAR)
    cv2.imwrite(os.path.join(target_folder, os.path.basename(img_path)), img)


    if data[2] > 0:
        data = [(int(data[0][0]*w_factor), int(data[0][1]*h_factor)), (int(data[1][0]*w_factor), int(data[1][1]*h_factor)), data[2]]
    saveAnnotation(os.path.join(target_folder, os.path.basename(annot)), data)

