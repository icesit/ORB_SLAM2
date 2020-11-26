# Usage of ORB-SLAM with ROS
## subscribe topics
* "/slam/totopo"(std_msgs::String): ask slam to save information to a path, then in the path there will be the "map.bin" file, "Input" folder. example(attention to a "/" at the end): "/home/xuewuyang/tmps/test/20/".
## publish topics
* "/slam/savedone"(std_msgs::String): after slam is saved activated by "/slam/totopo", the subscribed path is published again to this topic to activate next step.
* "/slam/pose"(geometry_msgs::PoseStamped): pose of slam.
* "/slam/posecov"(geometry_msgs::PoseWithCovarianceStamped): pose of slam.
