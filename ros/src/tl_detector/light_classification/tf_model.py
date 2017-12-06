from styx_msgs.msg import TrafficLight
import numpy as np
import os
import tensorflow as tf
from scipy.misc import imresize

cwd = os.path.dirname(os.path.realpath(__file__))


class TrafficLightModel():
    """A simple baseline traffic light classifier.
    :param thresh: Minimum amount of pixel within a given color.
    :param max_color: Pixel color value to exceed to get counted.
    """
    def __init__(self):
        os.chdir(cwd)

        MODEL_NAME = 'models/ssd_mobilenet.pb'
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(MODEL_NAME, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.classification_graph = tf.Graph()
        with self.classification_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile("models/classification.pb", 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

    def predict(self, image):
        light_states = [TrafficLight.GREEN, TrafficLight.RED, TrafficLight.YELLOW]
        light_strings = ['GREEN', 'RED', 'YELLOW']
        classification_imgs = []

        image = np.asarray(image, dtype="uint8")
        image = image[..., [2, 0, 1]]

        print(image.mean())
        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

                image_np = np.copy(image)
                img_height = image_np.shape[0]
                img_width = image_np.shape[1]
                image_np_expanded = np.expand_dims(image_np, axis=0)
                (boxes, scores, classes, num) = sess.run(
                    [detection_boxes, detection_scores, detection_classes, num_detections],
                    feed_dict={image_tensor: image_np_expanded})

                tl_idxs = np.where(classes == 10)
                scores = scores[tl_idxs]
                boxes = boxes[tl_idxs]
                top_score = np.where(scores > 0.1)
                boxes = boxes[top_score]

                for box in boxes:
                    ymin = int(box[0] * img_height)
                    ymax = int(box[2] * img_height)
                    xmin = int(box[1] * img_width)
                    xmax = int(box[3] * img_width)
                    traffic_light = image_np[ymin:ymax, xmin:xmax]
                    classification_imgs.append(traffic_light)

        if len(classification_imgs) > 0:
            print("traffic light detected")
            with self.classification_graph.as_default():
                with tf.Session(graph=self.classification_graph) as sess:
                    image_tensor = self.classification_graph.get_tensor_by_name('conv2d_13_input_6:0')
                    classification_tensor = self.classification_graph.get_tensor_by_name('out_0:0')

                    results = []
                    for img in classification_imgs:
                        img = imresize(img, (32, 32)).astype("float16")
                        image_np_expanded = np.expand_dims(img, axis=0)
                        (classes) = sess.run(
                            [classification_tensor],
                            feed_dict={image_tensor: image_np_expanded})
                        results.append(classes)
                    results = np.asarray(results).mean(axis=0)
                    return light_states[np.argmax(results)]

        return TrafficLight.UNKNOWN
