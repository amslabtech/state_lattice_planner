#include <gtest/gtest.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class StateLatticePlannerTest : public ::testing::Test
{
public:
    StateLatticePlannerTest(void)
    {
        vel_sub = nh.subscribe("/local_path/cmd_vel", 1, &StateLatticePlannerTest::vel_callback, this, ros::TransportHints().reliable().tcpNoDelay(true));
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_map/expand", 1);
        goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_goal", 1);
    }

protected:
    virtual void SetUp()
    {
        std::cout << "SetUp" << std::endl;
        vel_updated = false;
        local_costmap.info.resolution = 0.05;
        local_costmap.info.width = 200;
        local_costmap.info.height = 200;
        local_costmap.info.origin.position.x = -5.0;
        local_costmap.info.origin.position.y = -5.0;
        local_costmap.info.origin.orientation.w = 1.0;
        int data_num = local_costmap.info.width * local_costmap.info.height;
        local_costmap.data.clear();
        for(int i=0;i<data_num;i++){
            local_costmap.data.emplace_back(0);
        }
        local_costmap.header.frame_id = "base_link";
        local_costmap.header.stamp = ros::Time::now();

        odom.pose.pose.position.x = 0;
        odom.pose.pose.position.y = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        odom.pose.pose.orientation = tf2::toMsg(q);
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        ros::spinOnce();
    }

    void vel_callback(const geometry_msgs::TwistConstPtr& msg)
    {
        vel = *msg;
        vel_updated = true;
    }

    ros::NodeHandle nh;
    nav_msgs::OccupancyGrid local_costmap;
    nav_msgs::Odometry odom;
    geometry_msgs::Twist vel;
    ros::Subscriber vel_sub;
    ros::Publisher odom_pub;
    ros::Publisher map_pub;
    ros::Publisher goal_pub;
    bool vel_updated;
};

bool is_goal(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double d = std::sqrt(dx * dx + dy * dy);
    std::cout << "distance to goal: " << d << " m" << std::endl;
    return d < 0.5;
}

TEST_F(StateLatticePlannerTest, Navigation)
{
    std::cout << "Navigation test" << std::endl;
    double dt = 0.1;
    ros::Rate loop_rate(1 / dt);
    while(ros::ok()){
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "odom";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = 5;
        tf2::Quaternion q_;
        q_.setRPY(0, 0, 0);
        goal.pose.orientation = tf2::toMsg(q_);
        goal_pub.publish(goal);

        map_pub.publish(local_costmap);

        if(vel_updated){
            vel_updated = false;
            double r, p, y;
            tf2::Quaternion q;
            tf2::fromMsg(odom.pose.pose.orientation, q);
            tf2::Matrix3x3(q).getRPY(r, p, y);
            odom.pose.pose.position.x += vel.linear.x * cos(y) * dt;
            odom.pose.pose.position.y += vel.linear.y * sin(y) * dt;
            odom.twist.twist = vel;
            std::cout << "v: " << vel.linear.x << ", omega: " << vel.angular.z << std::endl;
        }
        odom.header.stamp = ros::Time::now();
        odom_pub.publish(odom);

        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transform;
        transform.header = odom.header;
        transform.child_frame_id = odom.child_frame_id;
        transform.transform.translation.x = odom.pose.pose.position.x;
        transform.transform.translation.y = odom.pose.pose.position.y;
        transform.transform.rotation = odom.pose.pose.orientation;
        br.sendTransform(transform);

        ros::spinOnce();
        loop_rate.sleep();

        if(is_goal(odom.pose.pose.position, goal.pose.position)){
            break;
        }
    }
    EXPECT_TRUE(1);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "state_lattice_planner_test");
    int r_a_t = RUN_ALL_TESTS();
    return r_a_t;
}
