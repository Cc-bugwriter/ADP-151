# Coordinate transformation this Verison is for CarMaker Simulation

* tf:
   
  /radar,/lidar,/base_link,/map

* pose_transfomration

  TransformPose is used
* velocity_transfomration is completed using TransformVector

* Rviz and Gui are made for viusalization
* this repo is for simulation, broadcase_node, radar_lidar_fusion, verify_objects_pose,visualization_objects are for simulation. broadcase is for the transformation between /map and /base_link. in radar_lidar_fusion, the absoluet state of objects are acquired, radar and lidar are fused.
* SensoInfo_node is used for the topic transformation between adma topic and vehicle topic, coordinate_transformation proccess to obtainthe absolute info of objects.
