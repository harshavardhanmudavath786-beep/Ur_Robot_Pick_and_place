// ROS2 includes
// C++ ROS2 API
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

// C++ includes
#include <chrono>
#include <memory>
#include <thread>


// This function creates and returns a pose
// We convert Euler angles to a quaternion object
// necessary to define a pose
geometry_msgs::msg::Pose createPose(double x,
                                    double y,
                                    double z,
                                    double roll,
                                    double pitch,
                                    double yaw)
{
    // populate x,y,z fields
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    // Here, we create a quaternion from the Euler angles
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    // Populate the quaternion fields
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();


    return pose;
}

int main(int argc, char * argv[])
{


    // Here, we declare our poses
    // x,y,z, roll, pitch, and yaw
    // Euler angles are in radians
    // The order of execution is:
    // home->pose1->pose2->gripper open-> pose3->gripper closed->home->open gripper
    geometry_msgs::msg::Pose pose1 = createPose(0.5,  0.1, 0.6, 3.14, 0.0, 0.0);
    geometry_msgs::msg::Pose pose2 = createPose(0.5,  0.1, 0.3, 3.14, 0.0, 0.0);
    geometry_msgs::msg::Pose pose3 = createPose(0.5, -0.1, 0.3, 3.14, 0.0, 0.0);


    // Here, we initialize the ROS2 API
    rclcpp::init(argc, argv);
    // create a node
    auto const node = std::make_shared<rclcpp::Node>(
        "control_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    // Logger to display messages
    auto const logger = rclcpp::get_logger("control_node");


    // Here we spin up a SingleThreadedExecutor
    // MoveIt can receive the current robot state
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner = std::thread([&executor]() { executor.spin(); });


    // Create MoveGroupInterfaces for the arm and the gripper
    using moveit::planning_interface::MoveGroupInterface;
    // the group for controlling the arm is called "arm"
    // the group for controlling the gripper is called "gripper"
    // this is defined by MoveIt setup assistant while creating the package moveit_robot_setup
    MoveGroupInterface armGroupControl(node, "arm");
    MoveGroupInterface gripperGroupControl(node, "gripper");


    // To track success and plan
    moveit::core::MoveItErrorCode success;
    MoveGroupInterface::Plan plan;


    // Over here, we start the movement process


    // First, we go to the home pose.
    // Note that the home pose is defined by the MoveIt Setup assistant
    // while creating the package moveit_robot_setup
    RCLCPP_INFO(logger, "Going to home pose!");
    armGroupControl.setNamedTarget("home");
    // perform the planning step
    success = armGroupControl.plan(plan);
    // if we have a valid plan, let us execute it
    if (success == moveit::core::MoveItErrorCode::SUCCESS)
    {
        armGroupControl.execute(plan);
    }

    // Go from home to pose1
    RCLCPP_INFO(logger, "Going from home to pose1!");
    armGroupControl.setPoseTarget(pose1);
    success = armGroupControl.plan(plan);
    if (success == moveit::core::MoveItErrorCode::SUCCESS)
    {
        armGroupControl.execute(plan);
    }

    // Go from pose1 to pose2
    RCLCPP_INFO(logger, "Going from pose1 to pose2");
    armGroupControl.setPoseTarget(pose2);
    success = armGroupControl.plan(plan);
    if (success == moveit::core::MoveItErrorCode::SUCCESS)
    {
        armGroupControl.execute(plan);
    }

    // Open the gripper in pose2
    RCLCPP_INFO(logger, "Open the gripper");
    gripperGroupControl.setNamedTarget("gripper_open");
    success = gripperGroupControl.plan(plan);
    if (success == moveit::core::MoveItErrorCode::SUCCESS)
    {
        gripperGroupControl.execute(plan);
    }

    // Introduce the delay to open/closed the gripper
    RCLCPP_INFO(logger, "1 second pause to open/close the gripper");
    rclcpp::sleep_for(std::chrono::seconds(1));


    // pose2 to pose3
    RCLCPP_INFO(logger, "Going from pose2 to pose3");
    armGroupControl.setPoseTarget(pose3);
    success = armGroupControl.plan(plan);
    if (success == moveit::core::MoveItErrorCode::SUCCESS)
    {
        armGroupControl.execute(plan);
    }

    // close the gripper in pose3
    RCLCPP_INFO(logger, "Close the gripper");
    gripperGroupControl.setNamedTarget("gripper_close");
    success = gripperGroupControl.plan(plan);
    if (success == moveit::core::MoveItErrorCode::SUCCESS)
    {
        gripperGroupControl.execute(plan);
    }

    // Introduce the delay to open/closed the gripper
    RCLCPP_INFO(logger, "1 second pause to open/close the gripper");
    rclcpp::sleep_for(std::chrono::seconds(1));


    // pose3 to home
    RCLCPP_INFO(logger, "Going from pose3 to home");
    armGroupControl.setNamedTarget("home");
    success = armGroupControl.plan(plan);
    if (success == moveit::core::MoveItErrorCode::SUCCESS)
    {
        armGroupControl.execute(plan);
    }

    // open the gripper in home
    RCLCPP_INFO(logger, "Open the gripper");
    gripperGroupControl.setNamedTarget("gripper_open");
    success = gripperGroupControl.plan(plan);
    if (success == moveit::core::MoveItErrorCode::SUCCESS)
    {
        gripperGroupControl.execute(plan);
    }

    // Introduce the delay to open/closed the gripper
    RCLCPP_INFO(logger, "1 second pause to open/close the gripper");
    rclcpp::sleep_for(std::chrono::seconds(1));


    // end
    RCLCPP_INFO(logger, "Movement done.");
    rclcpp::shutdown();
    spinner.join();
    return 0;
}