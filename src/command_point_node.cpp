#include<ros/ros.h>
#include<geometry_msgs/Point.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "command_point_node");
	ros::NodeHandle nh;

	ros::Publisher cmd_pt_pub = nh.advertise<geometry_msgs::Point>("command/point", 10);

	geometry_msgs::Point msg;
	msg.x = 5.0;
	msg.y = 5.0;
	msg.z = 5.0;
	
	while(ros::ok())
	{
		ROS_INFO_ONCE("cmd_point_node: Publishing");
		cmd_pt_pub.publish(msg);
		ros::spinOnce();
	}

	return 0;
}