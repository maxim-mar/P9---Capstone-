from styx_msgs.msg import TrafficLight
import rospy
import os
import numpy as np
import tensorflow as tf

TL_THREHOLD = 0.5


NUM_CLASSES = 4

class TLClassifier(object):

    def __init__(self):
        
        PATH_TO_GRAPH = 'light_classification/model/frozen_inference_graph_sim3.pb'

        item_green = {'id': 1, 'name': 'Green'}
        item_red = {'id': 2, 'name': 'Red'}
        item_yellow = {'id': 3, 'name': 'Yellow'}

        self.label_dict = {1: item_green, 2: item_red, 3: item_yellow}

        self.build_model_graph(PATH_TO_GRAPH)

        print("Classifier is ready")


    def detect_traffic_light(self, scores, biggest_score_idx,
                             classes,
                             detected_light):
        if scores[biggest_score_idx] > TL_THREHOLD:
            rospy.logwarn("Current traffic light is: {}"
                          .format(self.label_dict[classes[biggest_score_idx]]['name']))
            if classes[biggest_score_idx] == 1:
                detected_light = TrafficLight.GREEN
            elif classes[biggest_score_idx] == 2:
                detected_light = TrafficLight.RED
            elif classes[biggest_score_idx] == 3:
                detected_light = TrafficLight.YELLOW
        else:
            rospy.logwarn("Not defined")
        return detected_light


    def build_model_graph(self, model_path):
        self.model_graph = tf.Graph()
        with self.model_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as fid:
                saved_graph = fid.read()
                od_graph_def.ParseFromString(saved_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.model_graph)

        self.image_tensor = self.model_graph.get_tensor_by_name('image_tensor:0')
        self.scores = self.model_graph.get_tensor_by_name('detection_scores:0')
        self.classes = self.model_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.model_graph.get_tensor_by_name('num_detections:0')    
        self.boxes = self.model_graph.get_tensor_by_name('detection_boxes:0')

    def classify(self, image):

        detected_light = TrafficLight.UNKNOWN

        image_expanded = np.expand_dims(image, axis=0)
        with self.model_graph.as_default():
            (boxes, scores, classes, num) = self.sess.run([self.boxes, self.scores, self.classes, self.num_detections], 
                                                          feed_dict={self.image_tensor: image_expanded})

        scores = np.squeeze(scores)

        return self.detect_traffic_light(scores, scores.argmax(),
                                         np.squeeze(classes).astype(np.int32),
                                         detected_light)
