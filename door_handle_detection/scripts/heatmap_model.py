import glob
import os
import cv2

import numpy as np
import tensorflow as tf
from tensorflow.keras import datasets, layers, models, activations, optimizers
import matplotlib.pyplot as plt
from tensorflow.keras.utils import get_custom_objects


class DoorHandleHeatmapModel(tf.keras.Model):
    def __init__(self, conv_chan = [16, 32, 64, 128]):
        super(DoorHandleHeatmapModel, self).__init__(name='')

        self.num = len(conv_chan)

        self.pool = layers.MaxPooling2D((2, 2))

        self.c = []
        for lay in conv_chan:
            self.c.append(layers.Conv2D(lay, (5, 5), padding='same'))


        # self.b1 = tf.keras.layers.BatchNormalization()

        self.d = []
        for lay in conv_chan[0:-1]:
            self.d.append(layers.Conv2DTranspose(lay, (3, 3), strides=(2,2), padding='same'))

        self.d_last = layers.Conv2DTranspose(2, (2, 3), strides=(2,2), padding='same')

        self.comb = []
        for lay in conv_chan[0:-1]:
            self.comb.append([layers.Conv2D(lay, (5, 5), padding='same'), layers.Conv2D(lay, (3, 3), padding='same')])


    def call(self, x, training=False):
        xn = []

        for i in range(self.num):
            x = self.c[i](x)
            x = tf.nn.relu(x)
            x = self.pool(x)

            if i != self.num-1:
                xn.append(x)

        for i in range(self.num-2, -1, -1):
            x = self.d[i](x)
            # x += xn[i]
            x = tf.concat([x, xn[i]], axis=3)
            x = self.comb[i][0](x)
            x = tf.nn.relu(x)
            x = self.comb[i][1](x)
            x = tf.nn.relu(x)

        x = self.d_last(x)
        x = tf.nn.relu(x)
        return x

def test_heatmap_model():
    tf.executing_eagerly()

    model = DoorHandleHeatmapModel()

    input = np.ndarray((3, 240, 320, 1))
    output = model(input)

    model.build(input.shape)
    model.summary()

if __name__ == "__main__":
    test_heatmap_model()
