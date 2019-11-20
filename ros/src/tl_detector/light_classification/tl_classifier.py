from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import cv2
import rospy
import matplotlib.image as mpimg
import datetime

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.mode = input ("Press '1' for Simulator Mode \nPress '2' for Real-Life Mode\n")

        if self.mode == 1:
            PATH_TO_GRAPH = 'light_classification/model/frozen_inference_graph_sim3.pb'
        elif self.mode == 2:
            PATH_TO_GRAPH = 'light_classification/model/frozen_inference_graph_real4.pb'
        self.graph = tf.Graph()
        self.threshold = .5

        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_GRAPH, 'rb') as fid:
                od_graph_def.ParseFromString(fid.read())
                tf.import_graph_def(od_graph_def, name='')

            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name(
                'num_detections:0')
        self.img_file = 'ND'
        self.file_nr = 0


        self.sess = tf.Session(graph=self.graph)


    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        with self.graph.as_default():
            img_expand = np.expand_dims(image, axis=0)
            start = datetime.datetime.now()
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: img_expand})
            end = datetime.datetime.now()
            c = end - start



        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)
        Y1 = int(image.shape[0]*boxes[0,0])
        X1 = int(image.shape[1]*boxes[0,1])
        Y2 = int(image.shape[0] * boxes[0, 2])
        X2 = int(image.shape[1] * boxes[0, 3])

        cv2.rectangle(image, (X1, Y1), (X2, Y2), (0, 0, 255),2)

        print('SCORES:', scores[0])
        print('CLASSES:', classes[0])

        self.file_nr += 1
        self.img_file = 'ND'

        if self.mode == 1:
            if scores[0] > self.threshold:

                if classes[0] == 1:
                    self.img_file = 'GREEN'
                    print('Current light state is: GREEN')
                    mpimg.imsave('../images/' + self.img_file + str(self.file_nr) + '.jpg',
                                 cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
                    return TrafficLight.GREEN
                elif classes[0] == 2:
                    self.img_file = 'RED'
                    print('Current light state is: RED')
                    mpimg.imsave('../images/' + self.img_file + str(self.file_nr) + '.jpg',
                                 cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
                    return TrafficLight.RED
                elif classes[0] == 3:
                    self.img_file = 'YELLOW'
                    print('Current light state is: YELLOW')
                    mpimg.imsave('../images/' + self.img_file + str(self.file_nr) + '.jpg',
                                 cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
                    return TrafficLight.YELLOW

        elif self.mode == 2:
            if scores[0] > self.threshold:

                if classes[0] == 1:
                    self.img_file = 'GREEN'
                    print('Current light state is: GREEN')
                    mpimg.imsave('../images/' + self.img_file + str(self.file_nr) + '.jpg',
                                 cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
                    return TrafficLight.GREEN
                elif classes[0] == 2:
                    self.img_file = 'YELLOW'
                    print('Current light state is: YELLOW')
                    mpimg.imsave('../images/' + self.img_file + str(self.file_nr) + '.jpg',
                                 cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
                    return TrafficLight.YELLOW
                elif classes[0] == 3:
                    self.img_file = 'RED'
                    print('Current light state is: RED')
                    mpimg.imsave('../images/' + self.img_file + str(self.file_nr) + '.jpg',
                                 cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
                    return TrafficLight.RED



        mpimg.imsave('../images/'+self.img_file + str(self.file_nr)+'.jpg', cv2.cvtColor(image, cv2.COLOR_BGR2RGB))



        return TrafficLight.UNKNOWN
