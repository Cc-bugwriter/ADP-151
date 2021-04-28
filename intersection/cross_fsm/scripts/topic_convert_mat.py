#!/usr/bin/env python

import os
import rospy
import rospkg
import numpy as np
import message_filters
import scipy.io.matlab as matlab_io

from vaafo_msgs.msg import *
from operator import itemgetter
from collections import defaultdict

def usage():
        print " -----------------------------------  USAGE:  -------------------------------------------------"
        print " rosrun coordinate_transformation topic_convert_mat.py"
        print ""
        print " This code will extract all supported message types from the specified rosnode. "
        print ""
        print "  "
        print "  Currently supported message types are:"
        print "                   - vaafo_msgs/State "
        print "                   - vaafo_msgs/CarInfo "
        print "                   - vaafo_msgs/TrafficLightResult "
        print ""
        print "  Rostopic Convert Script v1 - Yi Cui - 12th Feb 2021"
        print ""
        print " ---------------------------------------------------------------------------------------------"


class TopicToMat:
    """
    Node class for collecting radar and vehicle data from CAN and publishing single radar objects.
    """

    def __init__(self):
        # Get directory for output file from ROS parameter server
        rospack = rospkg.RosPack()
        self.data_path = rospy.get_param(
            "/topic_convert_mat/output_path", rospack.get_path('cross_fsm') + "/data")
        self.save_to_mat = rospy.get_param(
            "/topic_convert_mat/save_to_mat", True)

        # Count files in output folder
        if not os.path.isdir(self.data_path):
            os.mkdir(self.data_path)

        self.files_in_output_dir = [f for f in os.listdir(self.data_path)
                                    if os.path.isfile(os.path.join(self.data_path, f))]
        self.num_output_files = int(len(self.files_in_output_dir)/2)

        # subscribers for all objects from sensors
        traffic_light_info = message_filters.Subscriber("/traffic_light", TrafficLightResult)
        state_info = message_filters.Subscriber("/behavior_state", State)
        car_info = message_filters.Subscriber("/VechInfo", CarInfo)

        sync= message_filters.TimeSynchronizer([car_info, state_info, traffic_light_info], 1)
        sync.registerCallback(self.callback_obj)

        # Initialize all variables
        self._vech = CarInfo()
        self._traffic = TrafficLightResult()
        self._state = State()
        self._target = CarInfo()
        self._output_vech = []
        self._output_traffic = []
        self._output_target = []

        self._output_vech_timestep = []
        self._output_vech_speed_x = []
        self._output_vech_speed_y = []
        self._output_vech_speed_z = []
        self._output_vech_angular_x = []
        self._output_vech_angular_y = []
        self._output_vech_angular_z = []
        self._output_vech_pose_x = []
        self._output_vech_pose_y = []
        self._output_vech_pose_z = []
        self._output_vech_quaternion_x = []
        self._output_vech_quaternion_y = []
        self._output_vech_quaternion_z = []
        self._output_vech_quaternion_w = []

        self._output_traffic_timestep = []
        self._output_traffic_light_id = []
        self._output_traffic_recognition_result = []
        self._output_traffic_reamining_time = []
        self._output_traffic_light_name = []
        self._output_traffic_pose_x = []
        self._output_traffic_pose_y = []
        self._output_traffic_pose_z = []
        #---------------------
        self._output_target_timestep = []
        self._output_target_speed_x = []
        self._output_target_speed_y = []
        self._output_target_speed_z = []
        self._output_target_angular_x = []
        self._output_target_angular_y = []
        self._output_target_angular_z = []
        self._output_target_pose_x = []
        self._output_target_pose_y = []
        self._output_target_pose_z = []
        self._output_target_quaternion_x = []
        self._output_target_quaternion_y = []
        self._output_target_quaternion_z = []
        self._output_target_quaternion_w = []
        self._output_target_frenet_s = []
        self._output_target_frenet_d = []

    # --------------------------- Callbacks ---------------------------------------------------
    def callback_obj(self, car_info, state_info, traffic_light_info):
        self._vech = car_info
        self._output_vech_timestep.append(car_info.header.stamp.to_sec())
        self._output_vech_speed_x.append(car_info.twist.linear.x)
        self._output_vech_speed_y.append(car_info.twist.linear.y)
        self._output_vech_speed_z.append(car_info.twist.linear.z)
        self._output_vech_angular_x.append(car_info.twist.angular.x)
        self._output_vech_angular_y.append(car_info.twist.angular.y)
        self._output_vech_angular_z.append(car_info.twist.angular.z)
        self._output_vech_pose_x.append(car_info.pose.position.x)
        self._output_vech_pose_y.append(car_info.pose.position.y)
        self._output_vech_pose_z.append(car_info.pose.position.z)
        self._output_vech_quaternion_x.append(car_info.pose.orientation.x)
        self._output_vech_quaternion_y.append(car_info.pose.orientation.y)
        self._output_vech_quaternion_z.append(car_info.pose.orientation.z)
        self._output_vech_quaternion_w.append(car_info.pose.orientation.w)

        self._state = state_info
        self._target = state_info.target
        self._output_target_timestep.append(state_info.target.header.stamp.to_sec())
        self._output_target_speed_x.append(state_info.target.twist.linear.x)
        self._output_target_speed_y.append(state_info.target.twist.linear.y)
        self._output_target_speed_z.append(state_info.target.twist.linear.z)
        self._output_target_angular_x.append(state_info.target.twist.angular.x)
        self._output_target_angular_y.append(state_info.target.twist.angular.y)
        self._output_target_angular_z.append(state_info.target.twist.angular.z)
        self._output_target_pose_x.append(state_info.target.pose.position.x)
        self._output_target_pose_y.append(state_info.target.pose.position.y)
        self._output_target_pose_z.append(state_info.target.pose.position.z)
        self._output_target_quaternion_x.append(state_info.target.pose.orientation.x)
        self._output_target_quaternion_y.append(state_info.target.pose.orientation.y)
        self._output_target_quaternion_z.append(state_info.target.pose.orientation.z)
        self._output_target_quaternion_w.append(state_info.target.pose.orientation.w)
        self._output_target_frenet_s.append(state_info.target.pose.s_d.x)
        self._output_target_frenet_d.append(state_info.target.pose.s_d.y)

        self._traffic = traffic_light_info
        self._output_traffic_timestep.append(traffic_light_info.header.stamp.to_sec())
        self._output_traffic_light_id.append(traffic_light_info.light_id)
        self._output_traffic_recognition_result.append(traffic_light_info.recognition_result)
        self._output_traffic_reamining_time.append(traffic_light_info.reamining_time)
        self._output_traffic_light_name.append(traffic_light_info.light_name_)
        self._output_traffic_pose_x.append(traffic_light_info.pose.position.x)
        self._output_traffic_pose_y.append(traffic_light_info.pose.position.x)
        self._output_traffic_pose_z.append(traffic_light_info.pose.position.x)

    # --------------------------- Functions ---------------------------------------------------
    def find_msg_type(self, msg):
        """
        find the datatype of input msg.

        Returns
        -------
        (str)
            datatype name.
        """
        msgType = str(type(msg))
        for c in msgType:
            if c =='.':
                msgType = msgType[msgType.index('.')+1:]
        msgType = msgType[:msgType.index('\'')]
        return msgType

    def FeedbackData(self):
        """
        return msg data and msg type.

        Returns
        -------
        (list, list, list, str, str, str)
            datatype, datatype, datatype, name, name, name
        """
        data_vech = []
        data_target = []
        data_traffic = []

        data_vech = [self._output_vech_timestep,self._output_vech_speed_x,self._output_vech_speed_y,self._output_vech_speed_z,
                            self._output_vech_angular_x,self._output_vech_angular_y,self._output_vech_angular_z,
                            self._output_vech_pose_x,self._output_vech_pose_y,self._output_vech_pose_z,
                            self._output_vech_quaternion_x,self._output_vech_quaternion_y,self._output_vech_quaternion_z,
                            self._output_vech_quaternion_w]

        data_target = [self._output_target_timestep,self._output_target_speed_x,self._output_target_speed_y,self._output_target_speed_z,
                            self._output_target_angular_x,self._output_target_angular_y,self._output_target_angular_z,
                            self._output_target_pose_x,self._output_target_pose_y,self._output_target_pose_z,
                            self._output_target_quaternion_x,self._output_target_quaternion_y,self._output_target_quaternion_z,
                            self._output_target_quaternion_w,self._output_target_frenet_s, self._output_target_frenet_d]

        data_traffic = [self._output_traffic_timestep, self._output_traffic_light_id, self._output_traffic_recognition_result,
                            self._output_traffic_reamining_time, self._output_traffic_light_name, self._output_traffic_pose_x,
                            self._output_traffic_pose_y, self._output_traffic_pose_z]

        type_vech = self.find_msg_type(self._vech)
        type_target = self.find_msg_type(self._state)
        type_traffic = self.find_msg_type(self._traffic)

        return data_vech, data_target, data_traffic, type_vech, type_target, type_traffic

    def postprocess_data(self, msgType, Input_daten):
        """
        Gives the correct file name if the current file name already exists.

        Returns
        -------
        (list, str)
            Sorted data and correct file name for saving the data.
        """
        sorted_data = Input_daten
        # Save data without overwriting pre-made files in 'data' directory
        save_file_name = "output_{}_{}".format(msgType, self.num_output_files)

        while save_file_name + ".npy" in self.files_in_output_dir:
            save_file_name = "output_{}_{}".format(msgType, self.num_output_files + 1)

        final_data = sorted_data
        return final_data, save_file_name

    def save_mat(self, msgType, data, path_name, file_name):
        """
        Saves the collected data as a Matlab file at the specified location.

        Parameters
        ----------
        data : [np.ndarray, Iterable]
            Collected data.

        path_name : str
            Path to the directory in which to save the file.

        file_name : str
            Filename.
        """
        if (msgType == 'TrafficLightResult'):
            names = ["timestep_traffic", "light_id", "recognition_result", "reamining_time", "light_name_", "pose_x_traffic", "pose_y_traffic", "pose_z_traffic"]
        elif (msgType == 'CarInfo'):
            names = ["timestep_car","speed_x_car", "speed_y_car", "speed_z_car", "angular_x_car", "angular_y_car", "angular_z_car",
                    "pose_x_car", "pose_y_car", "pose_z_car", "quat_x_car", "quat_y_car", "quat_z_car", "quat_w_car"]
        elif (msgType == 'State'):
            names = ["timestep_target","speed_x_target", "speed_y_target", "speed_z_target", "angular_x_target", "angular_y_target", "angular_z_target",
                    "pose_x_target", "pose_y_target", "pose_z_target", "quat_x_target", "quat_y_target", "quat_z_target", "quat_w_target", "frenet_s_target", "frenet_d_target"]

        data2 = defaultdict(list)
        for i in range(len(data)):
            data2[names[i]].append(data[i])

        matlab_io.savemat("{}/{}".format(path_name, file_name), data2)


def main():
    """
    Main entry point of the node.
    """
    rospy.init_node("topic_convert_node")
    topic_transmitter = TopicToMat()
    rospy.loginfo("Convert Topic to Mat Node started!")
    rate = rospy.Rate(1000) #100hz
    while not rospy.is_shutdown():
        data_vech, data_obj, data_obj_raw, type_vech, type_obj, type_obj_raw = topic_transmitter.FeedbackData()
    # Terminate program if user does not want to save data to a Matlab file.
    if not topic_transmitter.save_to_mat:
        return
    # Postprocess data
    final_data_vech, save_file_name_vech = topic_transmitter.postprocess_data(type_vech, data_vech)
    final_data_obj, save_file_name_obj = topic_transmitter.postprocess_data(type_obj, data_obj)
    final_data_obj_raw, save_file_name_obj_raw = topic_transmitter.postprocess_data(type_obj_raw, data_obj_raw)

    # Create Matlab file and save it
    topic_transmitter.save_mat(
        type_vech, final_data_vech, topic_transmitter.data_path, save_file_name_vech)
    topic_transmitter.save_mat(
        type_obj, final_data_obj, topic_transmitter.data_path, save_file_name_obj)
    topic_transmitter.save_mat(
        type_obj_raw, final_data_obj_raw, topic_transmitter.data_path, save_file_name_obj_raw)

    print "\033[1;33mSaved {} into {}\033[0m".format(
        save_file_name_vech, topic_transmitter.data_path)
    print "\033[1;33mSaved {} into {}\033[0m".format(
        save_file_name_obj, topic_transmitter.data_path)
    print "\033[1;33mSaved {} into {}\033[0m".format(
        save_file_name_obj_raw, topic_transmitter.data_path)

if __name__ == '__main__':
    main()
