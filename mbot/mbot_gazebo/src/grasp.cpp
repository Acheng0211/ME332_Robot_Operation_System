#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

GraspingDemo::GraspingDemo(ros::NodeHandle n_, float pregrasp_x, float pregrasp_y, float pregrasp_z, float length, float breadth) : it_(n_), armgroup("manipulator"),{
    this->nh_ = n_;
    // 获取base_link和camera_link之间的关系，也就是手眼标定的结果
    try
    {
        this->tf_camera_to_robot.waitForTransform("/base_link", "/camera_link", ros::Time(0), ros::Duration(50.0));
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
        ros::Duration(1.0).sleep();
    }

    // 如果查询得到的话，就将结果保存到camera_to_robot_，保存x,y,z和四元数一共7个值
    try
    {
        this->tf_camera_to_robot.lookupTransform("/base_link", "/camera_link", ros::Time(0), (this->camera_to_robot_));
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("[adventure_tf]: (lookup) %s", ex.what());
    }

    grasp_running = false;

    this->pregrasp_x = pregrasp_x;
    this->pregrasp_y = pregrasp_y;
    this->pregrasp_z = pregrasp_z;

    // 让机械臂运动到初始的位置
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::WallDuration(5.0).sleep();
    ROS_INFO_STREAM("Getting into the Grasping Position....");
    // 调用该函数控制机械臂运动到设定的位置
    attainPosition(pregrasp_x, pregrasp_y, pregrasp_z);

    // Subscribe to input video feed and publish object location
    // 订阅图像话题，一旦收到图像信息，就会进入到callback当中
    image_sub_ = it_.subscribe("/probot_anno/camera/image_raw", 1, &GraspingDemo::imageCb, this);
}

void GraspingDemo::attainPosition(float x, float y, float z)
{
    // ROS_INFO("The attain position function called");

    // 获取当前位置
    geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

    // 初始化数据类型
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation = currPose.pose.orientation;

    // 设置抓取前的机械臂位置
    target_pose1.position.x = x;
    target_pose1.position.y = y;
    target_pose1.position.z = z;
    armgroup.setPoseTarget(target_pose1);

    // 机械臂运动
    armgroup.move();
}

void GraspingDemo::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    if (!grasp_running)
    {
        ROS_INFO_STREAM("Processing the Image to locate the Object...");
        // 将图像变换到opencv当中
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // 调用vision_manager中的函数获取目标的位置，位置坐标是以摄像头中心点的位置作为0坐标
        //  ROS_INFO("Image Message Received");
        float obj_x, obj_y;
        vMng_.get2DLocation(cv_ptr->image, obj_x, obj_y);

        // Temporary Debugging
        std::cout << " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
        std::cout << " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;

        // 通过坐标变换，将二维坐标变换为相机坐标系下的三维坐标，在本程序中与URDF建模有关系
        obj_camera_frame.setZ(-obj_y);
        obj_camera_frame.setY(-obj_x);
        obj_camera_frame.setX(0.45);

        // 关键的一行代码，将相机坐标系下的位置转化为base_link坐标系下的坐标
        obj_robot_frame = camera_to_robot_ * obj_camera_frame;
        grasp_running = true;

        // Temporary Debugging
        std::cout << " X-Co-ordinate in Robot Frame :" << obj_robot_frame.getX() << std::endl;
        std::cout << " Y-Co-ordinate in Robot Frame :" << obj_robot_frame.getY() << std::endl;
        std::cout << " Z-Co-ordinate in Robot Frame :" << obj_robot_frame.getZ() << std::endl;
    }
}

void GraspingDemo::initiateGrasping()
{
    // 开启新的线程
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::WallDuration(3.0).sleep();

    // 获取当前的位置
    homePose = armgroup.getCurrentPose();

    // 调用attainObject()函数使机械臂靠近目标
    ROS_INFO_STREAM("Approaching the Object....");
    attainObject();

    // 夹取物体
    ROS_INFO_STREAM("Attempting to Grasp the Object now..");
    grasp();

    // 夹住物体做一个小范围移动
    ROS_INFO_STREAM("Lifting the Object....");
    lift();

    // 机械臂返回到初始状态
    ROS_INFO_STREAM("Going back to home position....");
    goHome();

    grasp_running = false;
}

void GraspingDemo::attainObject()
{
    // ROS_INFO("The attain Object function called");
    attainPosition(obj_robot_frame.getX(), obj_robot_frame.getY(), obj_robot_frame.getZ() + 0.04);

    // Open Gripper
    ros::WallDuration(1.0).sleep();
    grippergroup.setNamedTarget("open");
    grippergroup.move();

    // Slide down the Object
    geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();
    geometry_msgs::Pose target_pose1;

    target_pose1.orientation = currPose.pose.orientation;
    target_pose1.position = currPose.pose.position;

    target_pose1.position.z = obj_robot_frame.getZ() - 0.02;
    armgroup.setPoseTarget(target_pose1);
    armgroup.move();
}

void GraspingDemo::grasp()
{
    // ROS_INFO("The Grasping function called");

    ros::WallDuration(1.0).sleep();
    grippergroup.setNamedTarget("close");
    grippergroup.move();
}

void GraspingDemo::lift()
{
    // ROS_INFO("The lift function called");

    // For getting the pose
    geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation = currPose.pose.orientation;
    target_pose1.position = currPose.pose.position;

    // Starting Postion after picking
    // target_pose1.position.z = target_pose1.position.z + 0.06;

    if (rand() % 2)
    {
        target_pose1.position.y = target_pose1.position.y + 0.02;
    }
    else
    {
        target_pose1.position.y = target_pose1.position.y - 0.02;
    }

    armgroup.setPoseTarget(target_pose1);
    armgroup.move();

    // Open Gripper
    ros::WallDuration(1.0).sleep();
    grippergroup.setNamedTarget("open");
    grippergroup.move();

    target_pose1.position.z = target_pose1.position.z + 0.06;
    armgroup.setPoseTarget(target_pose1);
    armgroup.move();
}

int main(int argc, char **argv)
{
    // 初始化ROS节点，节点名为simple_grasping
    ros::init(argc, argv, "simple_grasping");

    float length, breadth, pregrasp_x, pregrasp_y, pregrasp_z;
    // 节点句柄
    ros::NodeHandle n;

    // 获取参数，首先是桌子的长
    if (!n.getParam("probot_grasping/table_length", length))
        length = 0.3;
    // 桌子的宽
    if (!n.getParam("probot_grasping/table_breadth", breadth))
        breadth = 0.3;

    // 机械臂的初始位置，不让机械臂挡着视野，影响拍照
    if (!n.getParam("probot_grasping/pregrasp_x", pregrasp_x))
        pregrasp_x = 0.20;
    if (!n.getParam("probot_grasping/pregrasp_y", pregrasp_y))
        pregrasp_y = -0.17;
    if (!n.getParam("probot_grasping/pregrasp_z", pregrasp_z))
        pregrasp_z = 0.28;

    // 创建一个对象，将参数传递进去
    GraspingDemo simGrasp(n, pregrasp_x, pregrasp_y, pregrasp_z, length, breadth);
    ROS_INFO_STREAM("Waiting for five seconds..");

    ros::WallDuration(5.0).sleep();

    // 不断查看图像队列，如果有识别到的图像，则进行抓取
    while (ros::ok())
    {
        // Process image callback
        ros::spinOnce();

        // 控制机械臂运动
        simGrasp.initiateGrasping();
    }
    return 0;
}