import glob
import os
import cv2

import numpy as np
import tensorflow as tf
from tensorflow.keras import datasets, layers, models, activations, optimizers
import matplotlib.pyplot as plt
from tensorflow.keras.utils import get_custom_objects
from annot_utils import *

def normalizeCoords(x, size):
    n = (x / size) * 2.0 - 1.0
    return n

def denormalizeCoords(n, size):
    x = int((n+1)/2 * size)
    return x

# https://stackoverflow.com/questions/4601373/better-way-to-shuffle-two-numpy-arrays-in-unison
def unison_shuffled_copies(a, b):
    assert len(a) == len(b)
    p = np.random.permutation(len(a))
    return a[p], b[p]

def loadDataFromFolder(folder):
    jpgs = glob.glob(folder + "*.jpg")

    imgs = []
    labels = []

    x = np.linspace(-1, 1, 320)
    y = np.linspace(-1, 1, 240)
    xv, yv = np.meshgrid(x, y)
    xv = np.expand_dims(xv, axis=2)
    yv = np.expand_dims(yv, axis=2)

    for img_path in jpgs:
        annot = getAnnotationName(img_path)

        if not os.path.exists(annot):
            print("Skipping unannotated {}".format(img_path))
            continue

        data = loadAnnotation(annot)
        label = [normalizeCoords(data[0][0], 320),
                 normalizeCoords(data[0][1], 240),
                 normalizeCoords(data[1][0], 320),
                 normalizeCoords(data[1][1], 240),
                 1.0 if data[2] > 0.5 else 0.0]

        img = cv2.imread(img_path)
        img = np.concatenate([img / 255.0, xv, yv], axis=2)

        imgs.append(img)
        labels.append(np.asarray(label))

    imgs = np.stack(imgs)
    labels = np.stack(labels)

    return imgs, labels

def loadAllData():
    all_images = []
    all_labels = []

    all_folders = [
        "/home/jgdo/catkin_ws/src/arips_ros/door_handle_detection/data/with_handle/",
        "/home/jgdo/catkin_ws/src/arips_ros/door_handle_detection/data/no_handle/",
        ]

    for folder in all_folders:
        images, labels = loadDataFromFolder(folder)
        all_images.append(images)
        all_labels.append(labels)

    all_images = np.concatenate(all_images)
    all_labels = np.concatenate(all_labels)
    all_images, all_labels = unison_shuffled_copies(all_images, all_labels)
    
    return all_images, all_labels

all_images, all_labels = loadAllData()

assert len(all_images) == len(all_labels)

split_index = int(len(all_labels) * 0.8)
train_images, train_labels = all_images[0:split_index], all_labels[0:split_index]
test_images, test_labels = all_images[split_index:-1], all_labels[split_index:-1]

print("Got {} training and {} validation images.".format(len(train_images), len(test_images)))

'''

(train_images, train_labels), (test_images, test_labels) = datasets.cifar10.load_data()

train_images, test_images = train_images / 255.0, test_images / 255.0

'''
model_path = "mymodel"

# TODO load dataset
model = models.Sequential()
model.add(layers.Conv2D(20, (5, 5), use_bias=True, input_shape=(240, 320, 5)))
model.add(layers.BatchNormalization())
model.add(layers.LeakyReLU())
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Conv2D(32, (5, 5), use_bias=True))
model.add(layers.BatchNormalization())
model.add(layers.LeakyReLU())
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Conv2D(64, (3, 3), use_bias=True))
model.add(layers.BatchNormalization())
model.add(layers.LeakyReLU())
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Conv2D(64, (3, 3), use_bias=True))
model.add(layers.BatchNormalization())
model.add(layers.LeakyReLU())
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Conv2D(64, (3, 3), use_bias=True))
model.add(layers.BatchNormalization())
model.add(layers.LeakyReLU())
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Conv2D(64, (3, 3), use_bias=True))
model.add(layers.BatchNormalization())
model.add(layers.LeakyReLU())
model.add(layers.Flatten())
model.add(layers.Dense(64, activation='tanh'))
model.add(layers.Dense(5, activation='tanh'))

# model.summary()

def my_loss_function(y_true, y_pred):
    squared_difference = tf.square(y_true[:, 0:4] - y_pred[:, 0:4])
    # print("squared_difference: ", squared_difference)
    position_loss = tf.reduce_mean(squared_difference, axis=-1) * y_true[:, 4]
    #print("position_loss: ", position_loss)
    loss = position_loss + tf.square((y_true[:, 4]*2-1) - y_pred[:, 4]) * 0.25
    #print("loss: ", loss)
    return loss

get_custom_objects().update({"my_loss_function": my_loss_function})

model.compile(optimizer=optimizers.Adam(learning_rate=0.0001),
              loss=my_loss_function,
              metrics=['mean_squared_error'])


#model = models.load_model(model_path)

if 1:
    history = model.fit(train_images, train_labels, epochs=300,
                        validation_data=(test_images, test_labels))

    plt.plot(history.history['loss'], label='loss')
    plt.plot(history.history['val_loss'], label='val_loss')
    plt.xlabel('Epoch')
    plt.ylabel('Accuracy')
    plt.ylim([0.0, 1])
    plt.legend(loc='lower right')
    plt.show()

model.save(model_path)


for test_index in range(0,20):
    labels = model.predict(train_images[test_index:test_index+1])[0]
    img = train_images[test_index, :, :, 0:3].copy()

    img = (img[:,:,::-1] * 255).astype(np.uint8)

    if labels[4] > 0.5:
        cv2.circle(img, (denormalizeCoords(labels[0], 320), (denormalizeCoords(labels[1], 240))), 3, (255, 0, 0), -1)
        cv2.circle(img, (denormalizeCoords(labels[2], 320), (denormalizeCoords(labels[3], 240))), 3, (0, 255, 0), -1)
    else:
        cv2.circle(img, (img.shape[1]//2, img.shape[0]//2), 30, (255, 255, 0), 1)

    plt.imshow(img)
    plt.show()
