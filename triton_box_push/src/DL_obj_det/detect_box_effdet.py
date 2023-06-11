#!/usr/bin/env python3

#from tensorflow.python.compiler.tensorrt import trt_convert as trt
import tensorflow as tf
#import tensorflow_hub as hub
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
import numpy as np
#import tf2onnx


bridge = CvBridge()
img = None

"""
# Instantiate the TF-TRT converter
params = trt.DEFAULT_TRT_CONVERSION_PARAMS._replace(precision_mode=trt.TrtPrecisionMode.FP16)
converter = trt.TrtGraphConverterV2(
   input_saved_model_dir=model_path,
   conversion_params=params
)

#tf2onnx.convert.from_saved_model(model_path, output_path="converted_model.onnx")
print('okay')

#print("num GPUs available: ", len(tf.config.list_physical_devices('GPU')))
exit()
"""

#converter = tf.lite.TFLiteConverter.from_saved_model(model_path)
#converter.experimental_new_converter = True
#converter._experimental_lower_tensor_list_ops = False
#tflite_model = converter.convert()

#with open("effdet/model.tflite", "wb") as f:
#    f.write(tflite_model)
   

"""
start = time.time()
detector = hub.load("https://tfhub.dev/tensorflow/efficientdet/lite1/detection/1")
print(type(detector))
end = time.time()
print(end-start)
"""


def preprocess():
    global img
    #img_path = "./bottle.jpg"
    #img = tf.io.read_file(img_path)
    #img = tf.image.decode_jpeg(img, channels=3)
    img = tf.image.resize(img, (384,384))
    img = img/255.0
    print(img.dtype)
    img = tf.expand_dims(img,axis=0)
    img = tf.cast(img, dtype=tf.uint8)
    #img.set_shape([None, 320,320,3])
    print(img.dtype)
    return img

def predict(img, model):  
    start = time.time()
    boxes, scores, classes, num_detections = model(img)
    end = time.time()
    print("time2: ", end-start)
    return boxes, scores, classes, num_detections

#subscriber function
def camera_callback(data):
	global img
	img = bridge.imgmsg_to_cv2(data, "rgb8")


if __name__ == '__main__':
    try:
        rospy.init_node('bottle_det', anonymous=True)
        cam_sub = rospy.Subscriber("/camera/color/image_raw",Image, camera_callback)
        rospy.sleep(0.6)	
        
        model_path = "./effdet"
        start = time.time()
        model = tf.saved_model.load(model_path)
        #inter = tf.lite.Interpreter(model_path=model_path)
        #inter.allocate_tensors()
        end = time.time()
        print("time1: ",end-start)

        img = preprocess()
        boxes, scores, classes, num_detections = predict(img, model)
        print(f"classes: {classes}, num_detections: {num_detections}")
        print(f"max score: {np.max(scores)}")
        max_coords = boxes[0][0]
        print(max_coords)
        #[ymin,xmin,ymax,xmax]
        center_x = (max_coords[1] + max_coords[3])/2
        center_y = (max_coords[0] + max_coords[2])/2
        print("center of bounding box: ",(center_x, center_y))

    except rospy.ROSInterruptException:
        print("ROS Interrupted!")
