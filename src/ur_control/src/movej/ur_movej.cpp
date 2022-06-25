/**
 * @file ur_movel.cpp
 * @author your name (you@domain.com)
 * @brief URDE 关节空间MoveJ伺服控制
 * @version 0.1
 * @date 2022-03-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <geometry_msgs/PointStamped.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/utilities/error.h>
#include <kdl/utilities/utility.h>
#include <kdl/velocityprofile_trap.hpp>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <trac_ik/trac_ik.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <velocity_profile/interpolate.h>

#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */

using namespace ur_rtde;
using namespace KDL;
using namespace std::chrono;

#pragma region  //* rivz 显示工具初始化
namespace rvt = rviz_visual_tools;
namespace rviz_visual_tools
{
    class RvizVisualToolsDemo
    {
    private:
        ros::NodeHandle nh_;

    public:
        rvt::RvizVisualToolsPtr visual_tools_;

        RvizVisualToolsDemo( )
        {
            visual_tools_.reset( new rvt::RvizVisualTools( "base", "/rviz_visual_tools" ) );
            visual_tools_->loadMarkerPub( );  // create publisher before waiting

            ROS_INFO( "Sleeping 1 seconds before running demo" );
            ros::Duration( 1.0 ).sleep( );

            // Clear messages
            visual_tools_->deleteAllMarkers( );
            visual_tools_->enableBatchPublishing( );
        }

        /**
         * @brief 在Rviz 中显示坐标系
         * 
         * @param frame 
         * @param i 
         */
        void testRows( KDL ::Frame frame, int i )
        {
            // Create pose
            geometry_msgs::Pose pose;
            pose.position.x = frame.p.x( );
            pose.position.y = frame.p.y( );
            pose.position.z = frame.p.z( );
            frame.M.GetQuaternion( pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w );
            visual_tools_->publishAxisLabeled( pose, std::to_string( i ), XSMALL );
        }
    };

}  // namespace rviz_visual_tools

#pragma endregion

std::vector< KDL::JntArray > moveJ( const KDL::JntArray& q_init, const KDL::JntArray& q_target, double vel, double acc )
{
    std::vector< rocos::DoubleS > doubleS_profil;
    doubleS_profil.resize( 6 );

    KDL::JntArray q_interp{ 6 };
    std::vector< KDL::JntArray > traj;
    std::vector< bool > need_plan( 6, false );

    double max_time = 0;
    double dt       = 0;

    for ( int i = 0; i < 6; i++ )
    {
        if ( q_init.data[ i ] != q_target.data[ i ] )
        {
            need_plan[ i ] = true;
            doubleS_profil[ i ].planDoubleSProfile( 0, q_init.data[ i ], q_target.data[ i ], 0, 0, vel, acc, 2 * acc );
            max_time = std::max( max_time, doubleS_profil[ i ].getDuration( ) );
        }
    }

    for ( int i = 0; i < 6; i++ )
    {
        if ( q_init.data[ i ] != q_target.data[ i ] )
            if ( doubleS_profil[ i ].getDuration( ) < max_time )
                doubleS_profil[ i ].my_scaleToDuration( max_time, 0, q_init.data[ i ], q_target.data[ i ], 0, 0 );
    }

    //** 轨迹计算 **//
    while ( dt < max_time )
    {
        for ( int i = 0; i < 6; i++ )
        {
            q_interp.data[ i ] = need_plan[ i ] ? doubleS_profil[ i ].pos( dt ) : q_init.data[ i ];
        }
        traj.push_back( q_interp );
        dt += 0.002;
    }
    return traj;
    //**-------------------------------**//
}

int main( int argc, char* argv[] )
{
    //** 变量初始化 **//

    ros::init( argc, argv, "UR_RVIE" );
    ros::NodeHandle node_handle;
    bool is_remoted_control = false;
    node_handle.param( "is_remoted_control", is_remoted_control, false );

 
    ros::Publisher publisher      = node_handle.advertise< sensor_msgs ::JointState >( "my_joint_state", 1000 );
    ros::Publisher publisher_rivz = node_handle.advertise< nav_msgs::Path >( "nav_msgs/Path", 1000 );
    sensor_msgs::JointState command_msg;
    ros::Rate rate( 500 );
    std::vector< sensor_msgs::JointState > joint_command;
    KDL::JntArray q_init{ 6 };
    TRAC_IK::TRAC_IK tracik_solver( "base", "wrist_3_link" );
    geometry_msgs::PoseStamped display_point;
    std::vector< geometry_msgs::PointStamped > display_point_vector;
    nav_msgs::Path display_path;
    rviz_visual_tools::RvizVisualToolsDemo demo;
    KDL::Chain UR_chain;
    tracik_solver.getKDLChain( UR_chain );
    ChainFkSolverPos_recursive ur_fk{ UR_chain };
    std::vector< double > joint_positions;
    double velocity       = 0.5;
    double acceleration   = 0.5;
    double dt             = 1.0 / 500;  // 2ms
    double lookahead_time = 0.1;
    double gain           = 300;
    const std::vector< double > joint_q{ 90 * M_PI / 180, -45 * M_PI / 180, -90 * M_PI / 180, -45 * M_PI / 180, 90 * M_PI / 180, 0 };
    KDL::JntArray q_target{ 6 };

    //**-------------------------------**//

    for ( int i = 0; i < 6; i++ )
    {
        q_init.data[ i ]   = joint_q[ i ];
        q_target.data[ i ] = 0;
    }
    q_target.data[ 1 ] = -M_PI / 2;
    q_target.data[ 3 ] = -M_PI / 2;

    command_msg.name = ros::V_string{ "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };
    command_msg.position.resize( 6 );
    display_point.header.frame_id = "base";
    display_path.header.frame_id  = "base";

  
    //** 起始和终止位姿设置 **//
    std::vector< KDL::JntArray > traj_joint = moveJ( q_init, q_target, 0.3, 0.3 );
    std::vector< KDL::Frame > traj_pos;
    traj_pos.resize( traj_joint.size( ) );

    for ( int i = 0; i < traj_joint.size( ); i++ )
    {
        ur_fk.JntToCart( traj_joint[ i ], traj_pos.at( i ) );
        for ( int j = 0; j < 6; j++ )
        {
            command_msg.position[ j ] = traj_joint[ i ].data[ j ];
        }
        joint_command.push_back( command_msg );
    }
    //**-------------------------------**//

    int index = 0;  //rviz 里打印的frame的标号
    for ( auto pos_goal : traj_pos )
    {
        //** 数据记录 **//

        display_point.pose.position.x = pos_goal.p.x( );
        display_point.pose.position.y = pos_goal.p.y( );
        display_point.pose.position.z = pos_goal.p.z( );
        pos_goal.M.GetQuaternion( display_point.pose.orientation.x, display_point.pose.orientation.y, display_point.pose.orientation.z, display_point.pose.orientation.w );
        display_path.poses.push_back( display_point );
        //**-------------------------------**//
        //** rviz显示 **//
        if ( ( index++ ) % 200 == 0 )
            demo.testRows( pos_goal, index );
        //**-------------------------------**//
    }

    demo.visual_tools_->trigger( );          //frame 显示
    publisher_rivz.publish( display_path );  //轨迹显示

    std::string order{ };
    for ( int j = 0; j < 100; j++ )
    {
        for ( auto i : joint_command )
        {
            publisher.publish( i );  //500hz控制机器人运动
            rate.sleep( );
        }
        std::cin >> order;
        if ( order == std::string{ "go" } ) break;
    }

    for ( unsigned int i = 0; i < joint_command.size( ); i++ )
    {
        auto t_start = high_resolution_clock::now( );
      
        for ( int index = 0; index < 6; index++ )
        {
            command_msg.position[ index ] = joint_positions[ index ];
        }
        publisher.publish( command_msg );  // 500hz显示机器人运动

        auto t_stop     = high_resolution_clock::now( );
        auto t_duration = std::chrono::duration< double >( t_stop - t_start );

        if ( t_duration.count( ) < dt )
        {
            std::this_thread::sleep_for( std::chrono::duration< double >( dt - t_duration.count( ) ) );
        }
    }

    std::cout << GREEN << " all is finished " << std::endl;
    if ( is_remoted_control )
    {
    }
    return 0;
}
