# rtc_oogway_operators
Code for the Ruhr Master School Turtlebot Competition of Team Oogway Operators  
  
To use this code, first modify ```costmap_common_parameters_{}.yaml``` (use the file used for your Turtlebot Version (burger/waffle))  
add sonar to observation_sources: ```observation_sources: scan sonar```  
add the line: ```sonar: {sensor_frame: base_link, data_type: PointCloud, topic: /sonar/point_cloud, marking: true, clearing: true}```  
  
Remember to run ```python sonar_to_costmap.py``` or add it to your launchfile to use it during robot operation.


```sonar_to_costmap.py.v1``` and ```sonar_to_costmap.py.v2``` are drafts containing experimental code for noise reduction. They are neither refactored nor considered working properly.
  
  
The ```turtlebot_core``` was modified to work with 5 sonars. In our tests the publishing frequency of turtlebot messages decreased because of the increased load of publishing 5 additional messages. Test if your setup can handle that and the robot still operates (slamming/navigation) as desired.
