# rtc_oogway_operators
Code for the Ruhr Master School Turtlebot Competition of Team Oogway Operators  
  
To use this code, first modify ```costmap_common_parameters_{}.yaml``` (use the file used for your Turtlebot Version (burger/waffle))  
add sonar to observation_sources: ```observation_sources: scan sonar```  
add the line: ```sonar: {sensor_frame: base_link, data_type: PointCloud, topic: /sonar/point_cloud, marking: true, clearing: true}```  
  
Remember to run ```python sonar_to_costmap.py``` or add it to your launchfile to use it during robot operation.
