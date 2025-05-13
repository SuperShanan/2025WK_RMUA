#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "min_snap/min_snap_closeform.h" 

#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

ros::Publisher goal_list_pub;
ros::Publisher poly_coef_pub;
ros::Publisher circlecount_pub;
ros::Publisher final_pub;
ros::Subscriber rviz_goal_sub;
ros::Subscriber eight_goal_sub;
ros::Subscriber odom_sub;
ros::Subscriber shijue_sub;

ros::Timer timer_;
ros::Timer timergetend_;

double replan_distance = 1.5;   //重规划距离，点之间的距离不能太近get_through

ros::Subscriber get_through_sub; 
// bool get_through=false;
int get_through=-1;

int id = 0;
int circlecount = 1;    //穿圈顺序
double mean_vel = 0;
const int GOAL_HEIGHT = 1.1;
nav_msgs::Odometry odom;

geometry_msgs::Pose goal_pt;
geometry_msgs::PoseArray goal_list;
my_planner::minsnapCloseform minsnap_solver;
std::vector<Eigen::Vector3d> waypoints;
quadrotor_msgs::PolynomialTrajectory poly_pub_topic;
Eigen::Vector3d visual_point_position;  //视觉点位
bool whreplan = false;  //是否可以进行重规划
// std::vector<Eigen::Vector3d> list_original{{2.3,0,1.1},{4.0,0.15,0.68},{4.12,1.6,0.8},{4.18,2.7,0.78},{1.16,2.6,1.1},{0.11,2.6,0.84}};
// std::vector<Eigen::Vector3d> list_original{{3.0,0,1.2},{4.0,2.50,0.9},{1.4,2.5,0.8},{1,1,1.0},{0,0,1.0}};//3.30redcircle chongguihua
std::vector<Eigen::Vector3d> list_original{{2.2,0.57,1.30},{2.4,0,1.20}};//所有需要停的位置以及预先点
std::vector<Eigen::Vector3d> over_circle;
std_msgs::Int32 msg;

// geometry_msgs::PoseStamped previous_position;
int stop[2] = {1};    //需要重归划的点的下标，会停下的点下标
int duandian_index=0;
Eigen::Vector3d Have_done_visual_point_position;

void pub_poly_coefs()
{
    Eigen::MatrixXd poly_coef = minsnap_solver.getPolyCoef();
    Eigen::MatrixXd dec_vel = minsnap_solver.getDecVel();
    Eigen::VectorXd time = minsnap_solver.getTime();

    poly_pub_topic.num_segment = goal_list.poses.size() - 1;
    poly_pub_topic.coef_x.clear();
    poly_pub_topic.coef_y.clear();
    poly_pub_topic.coef_z.clear();
    poly_pub_topic.time.clear();
    poly_pub_topic.trajectory_id = id;

    // display decision variable
    ROS_WARN("decision variable:");
    for (int i = 0; i < goal_list.poses.size(); i++)
    {
        cout << "Point number = " << i + 1 << endl
             << dec_vel.middleRows(i * 4, 4) << endl;
    }

    for (int i = 0; i < time.size(); i++)
    {
        for (int j = (i + 1) * 8 - 1; j >= i * 8; j--)
        {
            poly_pub_topic.coef_x.push_back(poly_coef(j, 0));
            poly_pub_topic.coef_y.push_back(poly_coef(j, 1));
            poly_pub_topic.coef_z.push_back(poly_coef(j, 2));
        }
        poly_pub_topic.time.push_back(time(i));
    }

    poly_pub_topic.header.frame_id = "world";
    poly_pub_topic.header.stamp = ros::Time::now();

    poly_coef_pub.publish(poly_pub_topic);
}

void solve_min_snap(double mean_vel)
{
        Eigen::Vector3d wp;
        waypoints.clear();
        for (int i = 0; i < int(goal_list.poses.size()); i++)
        {
            wp << goal_list.poses[i].position.x, goal_list.poses[i].position.y, goal_list.poses[i].position.z;
            waypoints.push_back(wp);
        }
        minsnap_solver.Init(waypoints, mean_vel);
        ROS_INFO("Init success");
        minsnap_solver.calMinsnap_polycoef();
        pub_poly_coefs();
}

void eight_goal_cb(const geometry_msgs::PoseStamped target_info)    //第一次规划到stop[0]
{
    double scale = 1;
    geometry_msgs::Pose tmp_pose;
    tmp_pose.orientation.x = 0;
    tmp_pose.orientation.y = 0;
    tmp_pose.orientation.z = 0;
    tmp_pose.orientation.w = 1;
    for (size_t i=0; i<stop[duandian_index]+1; i++)
    {
        geometry_msgs::Pose tmp_pt = tmp_pose;
        tmp_pt.position.x += list_original[i][0]*scale; 
        tmp_pt.position.y += list_original[i][1]*scale;
        tmp_pt.position.z += list_original[i][2]*scale;
        goal_list.poses.push_back(tmp_pt);
    }
    goal_pt.position = odom.pose.pose.position; //从自身位置开始规划
    goal_list.poses.insert(goal_list.poses.begin(), goal_pt);//插入odom
    goal_list.header.stamp = ros::Time::now();
    goal_list.header.frame_id = "world";
    goal_list.header.seq = id++;
    goal_list_pub.publish(goal_list);
    mean_vel = 0.8;
    solve_min_snap(mean_vel);
    ROS_INFO("solver finished");
    goal_list.poses.clear();
    whreplan = true;
}

void houbanduan(int next_point)
{   
    //if 偏差太大，用粗略点走
    geometry_msgs::Pose tmp_pose;

    tmp_pose = odom.pose.pose;
    goal_list.poses.push_back(tmp_pose);     //1.插入odom

    geometry_msgs::Pose tmp_pt;
    tmp_pt.position.x=visual_point_position(0);
    tmp_pt.position.y=visual_point_position(1);
    tmp_pt.position.z=visual_point_position(2);
    goal_list.poses.push_back(tmp_pt);      //2.加入视觉点
    //给到旋转框后面的点位改方向大小！！！！！！！！！！！！！
    Eigen::Vector3d lst_replan;
    // int now_index = next_point-1;  //当前stop下标
    if(duandian_index==6||duandian_index==2)
    {
        lst_replan[0] = visual_point_position[0]+1.0;
        lst_replan[1] = visual_point_position[1];
        lst_replan[2] = visual_point_position[2];
        mean_vel = 0.8;
    }
    if(duandian_index==5)
    {
        lst_replan[0] = visual_point_position[0]-1.0;
        lst_replan[1] = visual_point_position[1];
        lst_replan[2] = visual_point_position[2];
        mean_vel = 0.8;
    }
    if(duandian_index==4)
    {
        lst_replan[0] = visual_point_position[0];
        lst_replan[1] = visual_point_position[1]+1.0;
        lst_replan[2] = visual_point_position[2];
        mean_vel = 0.8;
    }
    if (duandian_index==0)
    {
        lst_replan[0] = visual_point_position[0];
        lst_replan[1] = visual_point_position[1]-1.0;
        lst_replan[2] = visual_point_position[2];
        mean_vel = 0.8;
    }
// cout<<"tmp_pt:"<<tmp_pt.position.x<<tmp_pt.position.y<<tmp_pt.position.z;
if(duandian_index!=0){
    tmp_pt.position.x=lst_replan[0];
    tmp_pt.position.y=lst_replan[1];
    tmp_pt.position.z=lst_replan[2];
    goal_list.poses.push_back(tmp_pt);      //3.插入旋转框后面的点
}
    for (size_t i=next_point; i<=stop[duandian_index+1]; i++)  //4.加入剩余点至下一个stop
    {
        geometry_msgs::Pose tmp_pt;
        tmp_pt.position.x = list_original[i][0]; 
        tmp_pt.position.y = list_original[i][1];
        tmp_pt.position.z = list_original[i][2];
        goal_list.poses.push_back(tmp_pt);
    }

    goal_list.header.stamp = ros::Time::now();
    goal_list.header.frame_id = "world";
    goal_list.header.seq = id++;   
    goal_list_pub.publish(goal_list);
    solve_min_snap(mean_vel);
    ROS_INFO("solver finished");
    goal_list.poses.clear();
}

void circleposeCallback(const geometry_msgs::Point::ConstPtr &shijuepose)
{
    visual_point_position(0) = shijuepose->x;
    visual_point_position(1) = shijuepose->y;
    visual_point_position(2) = shijuepose->z;
    ROS_INFO("getpointttttttttt");
    houbanduan(stop[duandian_index]+1);   //基于list_original的下一个
    duandian_index++;
}

// void checkgetend(const ros::TimerEvent &e)
// {
//     Eigen::Vector3d odm={odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z};
//     if((odm-list_original[stop[duandian_index]]).norm()<1.0 && get_through==stop[duandian_index])
//     {
//         ROS_INFO("GETTHROUGHHHHHHhHHHHHHHHHH");
//         ROS_INFO("GETTHROUGHHHHHHhHHHHHHHHHH");
//         ROS_INFO("GETTHROUGHHHHHHhHHHHHHHHHH");
//         ROS_INFO("GETTHROUGHHHHHHhHHHHHHHHHH");
//         ROS_INFO("GETTHROUGHHHHHHhHHHHHHHHHH");
//         if(visual_point_position(0)!=0 && visual_point_position(1)!=0 && visual_point_position(2)!=0){
//             houbanduan(stop[duandian_index]+1);//next point index
//             duandian_index++;
//         }
//     }
// }

//到达最后停机坪上方20cm发送命令修改receive_traj
void checktingjiping(const ros::TimerEvent &e){
    if(duandian_index!=13)return;
    //发送了最后一个点再计算距离
    Eigen::Vector3d odm={odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z};
    if(odm - fianl_point < 0.1){final_pub.publish(1);}
}
void odom_goal_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "min_snap_generator");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);

    odom_sub = nh.subscribe("/odom_topic", 10, odom_goal_cb);  

    eight_goal_sub = nh.subscribe("/rviz_goal", 10, eight_goal_cb);//初始化轨迹
    goal_list_pub = nh.advertise<geometry_msgs::PoseArray>("/goal_list", 10);
    poly_coef_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/poly_coefs", 10);
    // original_goal_sub = nh.subscribe("/airsim_node/drone_1/circle_poses", 10, original_circles_cb);//障碍圈的粗略数据
    // circlecount_pub = nh.advertise<std_msgs::Int32>("/circleorder", 10); 
    shijue_sub = nh.subscribe("/point_w",10,circleposeCallback);//更新点位重规划

    // timer_ = nh.createTimer(ros::Duration(0.5), getCircleCenter);  //0.02s检查一下是否到达目标点，到达后更新目标点位
    timergetend_ = nh.createTimer(ros::Duration(0.5), checktingjiping);  //0.02s检查一下是否到达目标点，到达后更新目标点位
    final_pub = nh.advertise<std_msgs::Int32>("/zhongdian",10);

    poly_pub_topic.num_order = 7;
    poly_pub_topic.start_yaw = 0;
    poly_pub_topic.final_yaw = 0;
    poly_pub_topic.mag_coeff = 0;
    poly_pub_topic.order.push_back(0);
    
    // ros::param::get("/min_snap_generator/meanvel", mean_vel);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}