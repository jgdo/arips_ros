import glob
import os

import cv2


def add_gripper_mask(img, gripper_img, gripper_mask):
    # https://stackoverflow.com/questions/10469235/opencv-apply-mask-to-a-color-image
    bg = cv2.bitwise_and(gripper_img, gripper_img, mask=gripper_mask)

    gripper_mask = cv2.bitwise_not(gripper_mask)
    fg = cv2.bitwise_and(img, img, mask=gripper_mask)
    final = cv2.bitwise_or(fg, bg)
    return final


def main():
    gripper_img = cv2.imread("../data/gripper_image.png")
    gripper_mask = cv2.imread("../data/gripper_mask.png", cv2.IMREAD_GRAYSCALE)

    folder_base = "/home/jgdo/catkin_ws/src/arips_ros/door_handle_detection/data/no_gripper/"
    folder_base_target = "/home/jgdo/catkin_ws/src/arips_ros/door_handle_detection/data/"
    subfolders = ["no_handle/", "with_handle/"]

    for subfolder in subfolders:
        source_folder = folder_base+subfolder
        jpgs = glob.glob(source_folder + "*.jpg")
        target_folder = folder_base_target + subfolder

        for jpg in jpgs:
            img = cv2.imread(jpg)
            new_image = add_gripper_mask(img, gripper_img, gripper_mask)
            target_img_path = target_folder + os.path.basename(jpg)
            cv2.imwrite(target_img_path, new_image)


if __name__ == "__main__":
    main()