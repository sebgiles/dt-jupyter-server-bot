#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import yaml

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped


class BraitenbergNode(DTROS):
    """Braitenberg Behaviour

    This node implements Braitenberg vehicle behavior on a Duckiebot.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node
            that ROS will use

    Configuration:
        ~gain (:obj:`float`): scaling factor applied to the desired
            velocity, taken from the robot-specific kinematics
            calibration
        ~trim (:obj:`float`): trimming factor that is typically used
            to offset differences in the behaviour of the left and
            right motors, it is recommended to use a value that results
            in the robot moving in a straight line when forward command
            is given, taken from the robot-specific kinematics calibration
        ~baseline (:obj:`float`): the distance between the two wheels
            of the robot, taken from the robot-specific kinematics
            calibration
        ~radius (:obj:`float`): radius of the wheel, taken from the
            robot-specific kinematics calibration
        ~k (:obj:`float`): motor constant, assumed equal for both
            motors, taken from the robot-specific kinematics calibration
        ~limit (:obj:`float`): limits the final commands sent to the
            motors, taken from the robot-specific kinematics calibration

    Subscriber:
        ~image/compressed (:obj:`CompressedImage`): The acquired camera
            images

    Publisher:
        ~wheels_cmd (:obj:`duckietown_msgs.msg.WheelsCmdStamped`): The
            wheel commands that the motors will execute

    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(BraitenbergNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")

        # Use the kinematics calibration for the gain and trim
        self.parameters['~gain'] = None
        self.parameters['~trim'] = None
        self.parameters['~baseline'] = None
        self.parameters['~radius'] = None
        self.parameters['~k'] = None
        self.parameters['~limit'] = None

        # Set parameters using a robot-specific yaml file if such exists
        self.readParamFromFile()
        self.updateParameters()

        # Wait for the automatic gain control
        # of the camera to settle, before we stop it
        rospy.sleep(2.0)
        rospy.set_param('/%s/camera_node/exposure_mode'%self.veh_name, 'off')
        rospy.set_param('/%s/camera_node/res_h'%self.veh_name, 240)
        rospy.set_param('/%s/camera_node/res_w'%self.veh_name, 320)

        image_topic = "/%s/camera_node/image/compressed"%self.veh_name
        self.image_sub = rospy.Subscriber(image_topic, CompressedImage, self.image_cb);
        cmd_topic = "/%s/wheels_driver_node/wheels_cmd"%self.veh_name
        self.cmd_pub = rospy.Publisher(cmd_topic, WheelsCmdStamped, queue_size=1);

        self.log("Initialized")

    # import numpy as np
    # import rospy
    # from sensor_msgs.msg import CompressedImage
    # import cv2
    # from threading import Thread
    # rospy.init_node("test1")
    # x = Thread(target=rospy.spin)
    # x.start()

    # it kinda makes sense that everything should happen in here
    def image_cb(self, data):
        #data = rospy.wait_for_message("/theducknight/camera_node/image/compressed",CompressedImage)
        np_arr = np.fromstring(data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        blurred = cv2.GaussianBlur(hsv,(15,15),0)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(hsv[:,:,2])
        x = maxLoc[0]
        y = maxLoc[1]
        w = np.shape(blurred)[1]
        gain = 0
        if maxVal > 100:
            gain = 5

        if hsv[y,x,0] > 40 and hsv[y,x,0] < 80 and hsv[y,x,1] > 100:
            #green
            gain = -gain

        #slit = np.mean(value, axis=(0))
        #center_of_mass = np.average(range(len(slit)),weights=np.square(slit));
        #peak = np.argmax(slit)
        gain = -5
        offset = gain*2*(x/w-0.5)
        #print("CoM = %f\r"%(center_of_mass))
        fwd_speed = 1.0

        l_cmd, r_cmd = self.speedToCmd((fwd_speed+offset), (fwd_speed-offset))
        now = rospy.Time.now()
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp.secs = now.secs
        msg_wheels_cmd.header.stamp.nsecs = now.nsecs
        msg_wheels_cmd.vel_right = r_cmd
        msg_wheels_cmd.vel_left = l_cmd
        self.cmd_pub.publish(msg_wheels_cmd)


    def speedToCmd(self, speed_l, speed_r):
        """Applies the robot-specific gain and trim to the
        output velocities

        Applies the motor constant k to convert the deisred wheel speeds
        to wheel commands. Additionally, applies the gain and trim from
        the robot-specific kinematics configuration.

        Args:
            speed_l (:obj:`float`): Desired speed for the left
                wheel (e.g between 0 and 1)
            speed_r (:obj:`float`): Desired speed for the right
                wheel (e.g between 0 and 1)

        Returns:
            The respective left and right wheel commands that need to be
                packed in a `WheelsCmdStamped` message

        """

        # assuming same motor constants k for both motors
        k_r = self.parameters['~k']
        k_l = self.parameters['~k']

        # adjusting k by gain and trim
        k_r_inv = (self.parameters['~gain'] + self.parameters['~trim'])\
                  / k_r
        k_l_inv = (self.parameters['~gain'] - self.parameters['~trim'])\
                  / k_l

        # conversion from motor rotation rate to duty cycle
        u_r = speed_r * k_r_inv
        u_l = speed_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = self.trim(u_r,
                                -self.parameters['~limit'],
                                self.parameters['~limit'])
        u_l_limited = self.trim(u_l,
                                -self.parameters['~limit'],
                                self.parameters['~limit'])

        return u_l_limited, u_r_limited

    def readParamFromFile(self):
        """
        Reads the saved parameters from
        `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml` or
        uses the default values if the file doesn't exist. Adjsuts
        the ROS paramaters for the node with the new values.

        """
        # Check file existence
        fname = self.getFilePath(self.veh_name)
        # Use the default values from the config folder if a
        # robot-specific file does not exist.
        if not os.path.isfile(fname):
            self.log("Kinematics calibration file %s does not "
                     "exist! Using the default file." % fname, type='warn')
            fname = self.getFilePath('default')

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        for param_name in ["gain", "trim", "baseline", "k", "radius", "limit"]:
            param_value = yaml_dict.get(param_name)
            if param_name is not None:
                rospy.set_param("~"+param_name, param_value)
            else:
                # Skip if not defined, use default value instead.
                pass

    def getFilePath(self, name):
        """
        Returns the path to the robot-specific configuration file,
        i.e. `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`.

        Args:
            name (:obj:`str`): the Duckiebot name

        Returns:
            :obj:`str`: the full path to the robot-specific
                calibration file

        """
        cali_file_folder = '/data/config/calibrations/kinematics/'
        cali_file = cali_file_folder + name + ".yaml"
        return cali_file

    def trim(self, value, low, high):
        """
        Trims a value to be between some bounds.

        Args:
            value: the value to be trimmed
            low: the minimum bound
            high: the maximum bound

        Returns:
            the trimmed value
        """

        return max(min(value, high), low)

    def onShutdown(self):
        """Shutdown procedure.

        Publishes a zero velocity command at shutdown."""
        # MAKE SURE THAT THE LAST WHEEL COMMAND YOU PUBLISH IS ZERO,
        # OTHERWISE YOUR DUCKIEBOT WILL CONTINUE MOVING AFTER
        # THE NODE IS STOPPED

        self.image_sub.unregister()

        now = rospy.Time.now()
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp.secs = now.secs
        msg_wheels_cmd.header.stamp.nsecs = now.nsecs
        msg_wheels_cmd.vel_right = 0
        msg_wheels_cmd.vel_left = 0
        self.cmd_pub.publish(msg_wheels_cmd)

        super(BraitenbergNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    camera_node = BraitenbergNode(node_name='braitenberg')
    # Keep it spinning to keep the node alive
    rospy.spin()
