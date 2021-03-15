import glob
import os
import cv2

import numpy as np
import tensorflow as tf
from tensorflow.keras import datasets, layers, models, activations, optimizers
import matplotlib.pyplot as plt
from tensorflow.keras.utils import get_custom_objects
from annot_utils import *
import dataset
from dataset import loadAllData, denormalizeCoords
import heatmap_model

all_images, all_labels = loadAllData()
train_gen = dataset.DoorDataGenerator(all_images, all_labels, train=True)
test_gen = dataset.DoorDataGenerator(all_images, all_labels, train=False, shuffle=False)

model_path = "mymodel"

"""
# my_activation = layers.LeakyReLU
my_activation = layers.ReLU

# TODO load dataset
model = models.Sequential()
model.add(layers.Conv2D(32, (5, 5), use_bias=True, input_shape=(240, 320, 5)))
model.add(layers.BatchNormalization())
model.add(my_activation())
#model.add(layers.Dropout(0.05))
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Conv2D(64, (3, 3), use_bias=True))
model.add(layers.BatchNormalization())
model.add(my_activation())
#model.add(layers.Dropout(0.1))
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Conv2D(128, (3, 3), use_bias=True))
model.add(layers.BatchNormalization())
model.add(my_activation())
#model.add(layers.Dropout(0.15))
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Conv2D(256, (3, 3), use_bias=True))
model.add(layers.BatchNormalization())
model.add(my_activation())
model.add(layers.Conv2D(128, (3, 3), use_bias=True))
model.add(layers.BatchNormalization())
model.add(my_activation())
model.add(layers.Conv2D(64, (3, 3), use_bias=True))
model.add(layers.BatchNormalization())
model.add(my_activation())
model.add(layers.Conv2D(32, (3, 3), use_bias=True))
model.add(layers.BatchNormalization())
model.add(my_activation())
model.add(layers.Flatten())
model.add(layers.Dense(64, activation='relu'))
model.add(layers.Dense(5, activation=None))


def my_loss_function(y_true, y_pred):
    squared_difference = tf.square(y_true[:, 0:4] - y_pred[:, 0:4])
    # print("squared_difference: ", squared_difference)
    position_loss = tf.reduce_mean(squared_difference, axis=-1) * y_true[:, 4]
    #print("position_loss: ", position_loss)
    loss = position_loss + tf.square((y_true[:, 4]*2-1) - y_pred[:, 4]) * 0.25
    #print("loss: ", loss)
    return loss


get_custom_objects().update({"my_loss_function": my_loss_function})
"""

model = heatmap_model.DoorHandleHeatmapModel()
model.build((1, 240, 320, 3))
model.summary()

model.compile(optimizer=optimizers.Adam(learning_rate=0.0005),
              loss='mse')


if 0:
    model = models.load_model(model_path)

if 1:
    try:
        my_callbacks = [
            tf.keras.callbacks.EarlyStopping(patience=20),
        ]

        history = model.fit(train_gen, epochs=200,
                            validation_data=test_gen, callbacks=my_callbacks)

        plt.plot(history.history['loss'], label='loss')
        plt.plot(history.history['val_loss'], label='val_loss')
        plt.xlabel('Epoch')
        plt.ylabel('Accuracy')
        # plt.ylim([0.0, 1])
        plt.legend(loc='lower right')
        plt.show()

    except KeyboardInterrupt:
        pass

    # model.save(model_path, ca)


def predictImage(path):
    x = np.linspace(-1, 1, 320)
    y = np.linspace(-1, 1, 240)
    xv, yv = np.meshgrid(x, y)
    xv = np.expand_dims(xv, axis=2)
    yv = np.expand_dims(yv, axis=2)

    img = cv2.imread(path)
    img_input = np.concatenate([img.astype(np.float32) / 255.0, xv, yv], axis=2)
    img_input = np.expand_dims(img_input, axis=0)

    labels = model.predict(img_input)[0]

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    if labels[4] > 0.5:
        cv2.circle(img, (denormalizeCoords(labels[0], 320), (denormalizeCoords(labels[1], 240))), 3, (255, 0, 0), -1)
        cv2.circle(img, (denormalizeCoords(labels[2], 320), (denormalizeCoords(labels[3], 240))), 3, (0, 255, 0), -1)
    else:
        cv2.circle(img, (img.shape[1] // 2, img.shape[0] // 2), 30, (255, 255, 0), 1)

    plt.imshow(img)
    plt.show()

# predictImage('/home/jgdo/frame0000.jpg')

"""

test_gen.get_with_indices = True
best_images = np.ndarray((0, 240, 320, 5), dtype=np.float32)
best_indices = []
best_losses = []
for i_b in range(len(test_gen)):
    images, gt_labels, indices = test_gen[i_b]
    pred_labels = model.predict(images)
    loss = my_loss_function(gt_labels, pred_labels).numpy()

    best_images = np.append(best_images, images, axis=0)
    best_indices += indices.tolist()
    best_losses = np.append(best_losses, loss, axis=0)

assert len(best_images) == len(best_indices)
assert len(best_losses) == len(best_indices)

best_indices.sort(key=lambda idx: best_losses[idx])
best_indices.reverse()


for best_index in range(20):
    idx = best_indices[best_index]
    labels = model.predict(best_images[idx:idx + 1])[0]
    img = best_images[idx, :, :, 0:3].copy()

    img = (img[:, :, ::-1] * 255).astype(np.uint8)

    if labels[4] > 0.5:
        cv2.circle(img, (denormalizeCoords(labels[0], 320), (denormalizeCoords(labels[1], 240))), 3, (255, 0, 0), -1)
        cv2.circle(img, (denormalizeCoords(labels[2], 320), (denormalizeCoords(labels[3], 240))), 3, (0, 255, 0), -1)
    else:
        cv2.circle(img, (img.shape[1] // 2, img.shape[0] // 2), 30, (255, 255, 0), 1)

    plt.imshow(img)
    plt.show()

"""

if 1:
    test_batch_images, test_batch_labels = test_gen[0]

    for test_index in range(len(test_batch_images)):
        labels = model.predict(test_batch_images[test_index:test_index+1])[0]

        dataset.showImageLabels(test_batch_images[test_index], labels, False)