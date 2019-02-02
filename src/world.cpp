#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
std::string turtle_name;
void poseCallback(const geometry_msgs::PoseConstPtr& msg){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(msg->position.x, msg->position.y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, msg->orientation.z);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ray"));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "mundo");
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("ray/pose", 10, &poseCallback);
	ros::spin();
	return 0;
};
