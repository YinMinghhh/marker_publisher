#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

class MarkerPublisher {
private:
    ros::NodeHandle nh_;
    ros::Subscriber amcl_pose_sub_;
    ros::Publisher marker_pub_;

public:
    MarkerPublisher() {
        amcl_pose_sub_ = nh_.subscribe("/amcl_pose", 10, &MarkerPublisher::callback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    }

    void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        double robot_x = msg->pose.pose.position.x;
        double robot_y = msg->pose.pose.position.y;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.6;
        marker.scale.y = 0.6;
        marker.scale.z = 0.4;

        marker.pose.position.x = robot_x;
        marker.pose.position.y = robot_y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

        marker_pub_.publish(marker);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "marker_publisher");
    MarkerPublisher marker_publisher;

    ros::spin();
    return 0;
}
