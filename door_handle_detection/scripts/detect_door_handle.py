import numpy as np
import tensorflow as tf
from tensorflow.keras import models
from tensorflow.keras.utils import get_custom_objects
import cv2
import roslib
import rospy

from sensor_msgs.msg import CompressedImage
import geometry_msgs.msg

def denormalizeCoords(n, size):
    x = int((n+1)/2 * size)
    return x

def my_loss_function(y_true, y_pred):
    squared_difference = tf.square(y_true[:, 0:4] - y_pred[:, 0:4])
    # print("squared_difference: ", squared_difference)
    position_loss = tf.reduce_mean(squared_difference, axis=-1) * y_true[:, 4]
    #print("position_loss: ", position_loss)
    loss = position_loss + tf.square((y_true[:, 4]*2-1) - y_pred[:, 4]) * 0.25
    #print("loss: ", loss)
    return loss

class DoorHandleDetector:
    def __init__(self):
        self.win = "DoorHandleDetector"

        get_custom_objects().update({"my_loss_function": my_loss_function})
        self.model = models.load_model("mymodel")

        # Load the TFLite model and allocate tensors.
        '''
        self.interpreter = tf.lite.Interpreter(model_path="exported_model/model.tflite")
        self.interpreter.allocate_tensors()

        # Get input and output tensors.
        input_details = self.interpreter.get_input_details()
        output_details = self.interpreter.get_output_details()

        input_shape = input_details[0]['shape']
        output_shape = output_details[0]['shape']
        assert np.array_equal(input_shape, [1, 240, 320, 5])
        assert np.array_equal(output_shape, [1, 5])

        self.tf_input_index = input_details[0]['index']
        self.tf_output_index = output_details[0]['index']
        '''

        x = np.linspace(-1, 1, 320)
        y = np.linspace(-1, 1, 240)
        xv, yv = np.meshgrid(x, y)
        self.xx = np.expand_dims(xv, axis=2).astype(np.float32)
        self.yy = np.expand_dims(yv, axis=2).astype(np.float32)

        # cv2.namedWindow(self.win)

        self.image_pub = rospy.Publisher("/kinect/rgb/door_handle_detection/compressed",
                                         CompressedImage, queue_size=1)

        self.coords_pub = rospy.Publisher("/door_handle/poses", geometry_msgs.msg.PoseArray, queue_size=1)

        self.subscriber = rospy.Subscriber("/kinect/rgb/image_color/compressed",
                                           CompressedImage, self.image_callback, queue_size=1)

    def image_callback(self, ros_image):
        np_arr = np.frombuffer(ros_image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img = image_np.copy()

        # image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)

        assert image_np.shape == (240, 320, 3)
        assert image_np.dtype == np.uint8

        # cv2.imshow(self.win, image_np)
        # cv2.waitKey(2)

        ## image_np = image_np[:,:,::-1]
        image_np = np.concatenate([image_np.astype(np.float32) / 255.0, self.xx, self.yy], axis=2)
        image_np = np.expand_dims(image_np, axis=0)

        '''
        self.interpreter.set_tensor(self.tf_input_index, image_np)
        self.interpreter.invoke()

        labels = self.interpreter.get_tensor(self.tf_output_index)[0]
        '''
        labels = self.model.predict(image_np)[0]

        coords = geometry_msgs.msg.PoseArray()
        coords.header = ros_image.header
        if labels[4] > 0.2: # TODO fix threshold
            coords.poses.append(geometry_msgs.msg.Pose())
            coords.poses.append(geometry_msgs.msg.Pose())
            coords.poses[0].position.x = labels[0]
            coords.poses[0].position.y = labels[1]
            coords.poses[1].position.x = labels[2]
            coords.poses[1].position.y = labels[3]
        # else: leave coords empty if no handle detected
        self.coords_pub.publish(coords)

        if self.image_pub.get_num_connections() > 0:
            # print(output_data)

            if labels[4] > 0:
                cv2.circle(img, (denormalizeCoords(labels[0], 320), (denormalizeCoords(labels[1], 240))), 3, (0, 0, 255),
                           -1)
                cv2.circle(img, (denormalizeCoords(labels[2], 320), (denormalizeCoords(labels[3], 240))), 3, (0, 255, 0),
                           -1)
            else:
                cv2.circle(img, (img.shape[1] // 2, img.shape[0] // 2), 30, (0, 255, 255), 1)

            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', img)[1]).tobytes()
            # Publish new image
            self.image_pub.publish(msg)

def main():
    rospy.init_node('door_handle_detector')
    detector = DoorHandleDetector()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
