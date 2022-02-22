import numpy as np

from pytorch.dataset import load_all_data, floor_step_conv_func


def compose_floor_step_dataset():
    out_path = "/home/jgdo/catkin_ws/src/arips_data/datasets/floor/dataset.npz"
    all_images, all_labels = load_all_data(["/home/jgdo/catkin_ws/src/arips_data/datasets/floor/annotated_scaled_pos/",
                                            "/home/jgdo/catkin_ws/src/arips_data/datasets/floor/annotated_scaled_neg/"],
                                           floor_step_conv_func)

    np.savez_compressed(out_path, all_images=all_images, all_labels=all_labels)


if __name__=="__main__":
    compose_floor_step_dataset()