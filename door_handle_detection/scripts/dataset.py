import glob
import os
import random

import cv2

import numpy as np
from tensorflow.python.keras.utils.data_utils import Sequence
import matplotlib.pyplot as plt
from annot_utils import *


def normalizeCoords(x, size):
    n = (x / size) * 2.0 - 1.0
    return n


def denormalizeCoords(n, size):
    x = int((n + 1) / 2 * size)
    return x


# https://stackoverflow.com/questions/4601373/better-way-to-shuffle-two-numpy-arrays-in-unison
def unison_shuffled_copies(a, b):
    assert len(a) == len(b)
    p = np.random.permutation(len(a))
    return a[p], b[p]


def loadDataFromFolder(folder, coord_labels = False):
    jpgs = glob.glob(folder + "*.jpg")

    imgs = []
    labels = []

    if coord_labels:
        x = np.linspace(-1, 1, 320)
        y = np.linspace(-1, 1, 240)
        xv, yv = np.meshgrid(x, y)
        xv = np.expand_dims(xv, axis=2)
        yv = np.expand_dims(yv, axis=2)

    for img_path in jpgs:
        annot = getAnnotationName(img_path)

        if not os.path.exists(annot):
            # print("Skipping unannotated {}".format(img_path))
            continue

        data = loadAnnotation(annot)

        img = cv2.imread(img_path) / 255.0

        if coord_labels:
            img = np.concatenate([img
                                     , xv, yv
                                  ], axis=2)
            label = [normalizeCoords(data[0][0], 320),
                     normalizeCoords(data[0][1], 240),
                     normalizeCoords(data[1][0], 320),
                     normalizeCoords(data[1][1], 240),
                     1.0 if data[2] > 0.5 else 0.0]
        else:
            label = np.zeros((240, 320, 2), dtype=np.float32)
            if data[2] > 0.5:
                label[data[0][1], data[0][0], 0] = 1
                label[data[1][1], data[1][0], 1] = 1
                ksize = 17
                label[:,:,0] = cv2.GaussianBlur(label[:,:,0], (ksize, ksize), 0)
                label[:, :, 0] /= label[:,:,0].max()
                label[:, :, 1] = cv2.GaussianBlur(label[:, :, 1], (ksize, ksize), 0)
                label[:, :, 1] /= label[:, :, 1].max()

                #kernel = np.ones((7, 7), np.uint8)
                #label[:,:,0] = cv2.dilate(label[:,:,0], kernel, iterations=1)
                #label[:, :, 1] = cv2.dilate(label[:, :, 1], kernel, iterations=1)

        imgs.append(img)
        labels.append(np.asarray(label))

    imgs = np.stack(imgs)
    labels = np.stack(labels)

    return imgs, labels


def loadAllData(coord_labels = False):
    all_images = []
    all_labels = []

    all_folders = [
        "/home/jgdo/catkin_ws/src/arips_ros/door_handle_detection/data/with_handle/",
        "/home/jgdo/catkin_ws/src/arips_ros/door_handle_detection/data/no_handle/",
    ]

    for folder in all_folders:
        images, labels = loadDataFromFolder(folder, coord_labels)
        all_images.append(images)
        all_labels.append(labels)

    all_images = np.concatenate(all_images)
    all_labels = np.concatenate(all_labels)
    all_images, all_labels = unison_shuffled_copies(all_images, all_labels)

    return all_images, all_labels


class DoorDataGenerator(Sequence):
    def __init__(self, all_images, all_labels, train, batch_size=32, shuffle=True, coord_labels = True):
        assert len(all_images) == len(all_labels)

        train_ratio = 0.8
        split_index = int(len(all_labels) * train_ratio)

        if train:
            self.images = all_images[0:split_index]
            self.labels = all_labels[0:split_index]
        else:
            self.images = all_images[split_index:-1]
            self.labels = all_labels[split_index:-1]

        self.batch_size = batch_size
        self.shuffle = shuffle

        self.get_with_indices = False
        self.coord_labels = coord_labels

        self.on_epoch_end()

    def __len__(self):
        return int(np.floor(len(self.images) / self.batch_size))

    def on_epoch_end(self):
        'Updates indexes after each epoch'
        self.indexes = np.arange(len(self.images))
        if self.shuffle == True:
            np.random.shuffle(self.indexes)

    def arugment_brightness(self, img, label):
        brightness_center = random.uniform(0.0, 1.0)
        brightness_factor = random.uniform(0.8, 1.3)

        img -= brightness_center
        img *= brightness_factor
        img += brightness_center

        np.clip(img, 0.0, 1.0, out=img)

        return img, label

    def augment_pos(self, img, label):
        width, height = img.shape[1], img.shape[0]

        center_x = random.randrange(width)
        center_y = random.randrange(height)
        scale_factor = random.uniform(1.0, 1.3)

        # ignore if label will be outside image
        if label[4] > 0:
            ncenter_x = normalizeCoords(center_x, width)
            ncenter_y = normalizeCoords(center_y, height)
            sx = (label[0] - ncenter_x) * scale_factor + ncenter_x
            sy = (label[1] - ncenter_y) * scale_factor + ncenter_y
            ex = (label[2] - ncenter_x) * scale_factor + ncenter_x
            ey = (label[3] - ncenter_y) * scale_factor + ncenter_y

            def in_range(x):
                return -1.0 <= x <= 1.0

            # return unchanged if out of range
            for x in [sx, sy, ex, ey]:
                if not in_range(x):
                    return img, label

            label = [sx, sy, ex, ey, label[4]]

        # else: ignore label, since no door handle present

        img_resized = cv2.resize(img, None, fx=scale_factor, fy=scale_factor, interpolation = cv2.INTER_LINEAR)
        start_x = round((center_x) * scale_factor - center_x)
        start_y = round((center_y) * scale_factor - center_y)

        img = img_resized[start_y:start_y+height, start_x:start_x+width]

        return img, label

    def augment_noise(self, img, label):
        mean = 0
        sigma = random.uniform(0.0, 0.05)
        gauss = np.random.normal(mean, sigma, img.shape)
        img += gauss
        np.clip(img, 0, 1, out=img)
        return img, label

    def augment(self, batch_images, batch_labels, funcs):
        assert len(batch_images) == len(batch_labels)

        for i in range(len(batch_images)):
            img, label = batch_images[i, :, :, 0:3], batch_labels[i]
            for func in funcs:
                img, label = func(img, label)

            batch_images[i, :, :, 0:3], batch_labels[i] = img, label

        return batch_images, batch_labels

    def __getitem__(self, index):
        'Generate one batch of data'
        # Generate indexes of the batch
        indices = self.indexes[index * self.batch_size: (index + 1) * self.batch_size]

        batch_images, batch_labels = self.images[indices], self.labels[indices]

        batch_images, batch_labels = self.augment(batch_images, batch_labels,
                                                  [self.arugment_brightness,
                                                   # self.augment_pos,
                                                   self.augment_noise,])

        if self.get_with_indices:
            return batch_images, batch_labels, indices
        return batch_images, batch_labels


def showImageLabels(img_float_5, labels, coord_labels):
    img = (img_float_5[:, :, 0:3] * 255).astype(np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    if coord_labels:
        if labels[4] > 0.5:
            cv2.circle(img, (denormalizeCoords(labels[0], 320), (denormalizeCoords(labels[1], 240))), 3, (255, 0, 0),
                       -1)
            cv2.circle(img, (denormalizeCoords(labels[2], 320), (denormalizeCoords(labels[3], 240))), 3, (0, 255, 0),
                       -1)
        else:
            cv2.circle(img, (img.shape[1] // 2, img.shape[0] // 2), 30, (255, 255, 0), 1)
    else:
        color1 = np.stack([labels[:,:,0]*255, labels[:,:,0]*0, labels[:,:,0]*0], axis=2).astype(np.uint8)
        color2 = np.stack([labels[:, :, 1] * 0, labels[:, :, 1] * 255, labels[:, :, 1] * 0], axis=2).astype(np.uint8)
        img = np.maximum(img, color1)
        img = np.maximum(img, color2)

    plt.imshow(img)
    plt.show()

def test_dataset():
    coord_labels = False

    all_images, all_labels = loadAllData(coord_labels=coord_labels)
    dataset = DoorDataGenerator(all_images, all_labels, train=True, coord_labels=coord_labels)

    batch_images, batch_labels = dataset[0]

    for i in range(10):
        showImageLabels(batch_images[i], batch_labels[i], coord_labels)


if __name__ == "__main__":
    test_dataset()
