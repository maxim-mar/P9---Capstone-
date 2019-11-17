#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Header
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
import math
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import yaml


OBSERV_DIST = 100

STATE_COUNT_THRESHOLD = 2

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.light_positions = self.config['stop_line_positions']
        self.light_classifier = TLClassifier()
        self.camera_image = None
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.bridge = CvBridge()
        self.state_count = 0
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights


    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED \
                                   or state == TrafficLight.YELLOW else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))

        self.state_count = self.state_count + 1




    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        waypoint = 0
        waypoints_list = self.waypoints.waypoints
        dist = float('inf')
        for i in range(len(waypoints_list)):
            new_dist = self.calc_distance_points_3D(pose.position,
                                                    waypoints_list[i].pose.pose.position)
            if  dist > new_dist:
                dist = new_dist
                waypoint = i
        return waypoint



    def get_light_state(self):
        """ Get current color of the traffic light """
        if self.camera_image is None:
            return False
        else:
            self.camera_image.encoding = "rgb8"
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
            #Perform classification here
            state = self.light_classifier.classify(cv_image)
            # Use last state if classifier is not sure
            if state == TrafficLight.UNKNOWN and self.last_state:
                state = self.last_state
            return state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        light = None

        if hasattr(self, 'waypoints') and hasattr(self, 'pose'):

            min_dist = float('inf')

            car_wp = self.get_closest_waypoint(self.pose.pose)

            light_positions = self.light_positions

            i = 0

            for light_pos in light_positions:

                light_wp, tl_candid, tl_dist = self.calclulate_distance_to_traffic_light(car_wp, light_pos)

                if (tl_dist < min_dist) \
                        and (car_wp % len(self.waypoints.waypoints)) < (light_wp % len(self.waypoints.waypoints)) \
                        and (tl_dist < OBSERV_DIST) :
                    closest_light_wp = light_wp
                    min_dist = tl_dist
                    light = tl_candid

                i += 1

            state = TrafficLight.UNKNOWN
            light_wp = -1

            if light:
                state = self.get_light_state()
                light_wp = closest_light_wp

        else:
            light_wp = -1
            state = TrafficLight.RED

        return light_wp, state




    def calclulate_distance_to_traffic_light(self, car_waypoint, light_position):
        tl_candid = self.create_tl(0.0, TrafficLight.UNKNOWN, light_position[0], light_position[1], 0.0)
        light_waypoint = self.get_closest_waypoint(tl_candid.pose.pose)
        tl_dist = self.calc_distance_coords_2D(self.waypoints.waypoints[car_waypoint].pose.pose.position.x,
                                               self.waypoints.waypoints[car_waypoint].pose.pose.position.y,
                                               self.waypoints.waypoints[light_waypoint].pose.pose.position.x,
                                               self.waypoints.waypoints[light_waypoint].pose.pose.position.y)
        return light_waypoint, tl_candid, tl_dist


    def calc_distance_coords_2D(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    def calc_distance_points_3D(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

    def create_tl(self, yaw, state, x, y, z):
        traffic_light = TrafficLight()

        traffic_light.header = Header()
        traffic_light.pose.header = Header()
        traffic_light.pose = PoseStamped()


        traffic_light.state = state

        traffic_light.pose.pose.position.x = x
        traffic_light.pose.pose.position.y = y
        traffic_light.pose.pose.position.z = z


        traffic_light.pose.header.stamp = rospy.Time.now()
        traffic_light.pose.header.frame_id = 'world'

        traffic_light.header.stamp = rospy.Time.now()
        traffic_light.header.frame_id = 'world'

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi * yaw / 180.0)
        traffic_light.pose.pose.orientation = Quaternion(*q)

        return traffic_light


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
