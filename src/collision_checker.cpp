#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

class CollisionChecker {
public:
    CollisionChecker() {
        amcl_pose_sub = nh.subscribe("/amcl_pose", 10, &CollisionChecker::pose_callback, this);
        scan_sub = nh.subscribe("/scan", 10, &CollisionChecker::scan_callback, this);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber amcl_pose_sub;
    ros::Subscriber scan_sub;
    double robot_x, robot_y, robot_yaw;

    void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)     {
        robot_x = msg->pose.pose.position.x;
        robot_y = msg->pose.pose.position.y;
        robot_yaw = tf::getYaw(msg->pose.pose.orientation);
    }
    
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        float angle_min = msg->angle_min;
        float angle_increment = msg->angle_increment;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];
            float angle = angle_min + i * angle_increment;

            // 将激光雷达数据转换到小车坐标系
            float obstacle_x = robot_x + range * cos(robot_yaw + angle);
            float obstacle_y = robot_y + range * sin(robot_yaw + angle);

            // 0.6*0.6的尺寸碰撞太少了，为了结果多碰撞这里写的边长1.2
            if (obstacle_x >= robot_x - 0.6 && obstacle_x <= robot_x + 0.6 &&
                obstacle_y >= robot_y - 0.6 && obstacle_y <= robot_y + 0.6) {

                ROS_WARN("Collision detected!");
                break;
            }
        }

    }
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "collision_checker");
    CollisionChecker collision_checker;

    ros::spin();
    return 0;
}
