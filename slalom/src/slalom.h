/*------------------------------------------------------------------------------
* ------------------------------------------------------------------------------
*Author: Michael Wachl
*Year: 2018/2019
*Course: Technik autonomer Systeme
*Group: 5
*Univerity: Technische Universität München
* ------------------------------------------------------------------------------
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
*This class is used to stear an ac car through an slalom course
*It takes the fornt laser scan data and transforms it to a pointcloud for further
*processing. The a cutoff in lokal x and y coordinates is applied and the point
*cloud is clustered with kd-tree. A cone has a typical cluster size, therefore
*the cone is the only cluster left after filtering and clustering.
*All cones coordinates are stored to calculate the directions of the goals.
*Dynamic reconfigure is used to change parameters in runtime.
*E.g. offsets to the cone position for the goal and the arriving angle can 
*be changed, filtering parameters etc. 
*There are two modes: static mode are predefined goals and dynamic mode where 
*all positions of the cones and starting position are used to calculate the goals.
*The goals are giving to the move_base with a simple action client, therefore 
*the lokal planner is used to calculate the path to the goal.
------------------------------------------------------------------------------*/



#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/PointCloud.h"
#include <tf/transform_datatypes.h>

//to transform scan to pointcloud
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include <laser_geometry/laser_geometry.h>
#include "message_filters/subscriber.h"

//for pointclouds
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>

//for dyn reconfig
#include <slalom/slalomConfig.h>
#include <dynamic_reconfigure/server.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define PI 3.14159265

typedef pcl::PointXYZ PointT;

class CSlalom {
public:
    CSlalom(ros::NodeHandle nh):
        node_(nh),
        laser_sub_(node_, "/scan", 10),
        laser_notifier_(laser_sub_,tfListener_, "base_link", 10)
    {
        init_pos_sub_ = node_.subscribe("/initialpose", 1, &CSlalom::initPosCb, this); //get the inital position (from rviz)
        cloul_sub_ = node_.subscribe ("/slalom_cloud", 1, &CSlalom::cloud_cb, this);
        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/slalom_cloud", 1, false);
        pos_publisher_ = node_.advertise<geometry_msgs::PoseStamped> ("/cone_pose", 1, false);
        laser_notifier_.registerCallback(boost::bind(&CSlalom::scanCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.01));

    }
    //variables
    geometry_msgs::PoseStamped start_pos_, current_goal_, first_cone_, second_cone_, third_cone_, fourth_cone_;
    double start_angle_, current_angle_;
    std::vector<geometry_msgs::Pose> slalomtraject_; // vector of goals, with position and orientation
    bool pos_res_ = false;
    bool DYNAMIC_MODE_ = false;
    bool GO_OUT_ = false;
    bool detect_first_cone_= false, detect_second_cone_= false , detect_third_cone_= false , detect_fourth_cone_ = false, go_out_ = false;
    bool first_cone_detected_ = false, second_cone_detected_  = false, third_cone_detected_=false, fourth_cone_detected_=false;
    ros::Publisher  pos_publisher_;
    //funcions
    geometry_msgs::PoseStamped calculateGoal();
    void paramCallback(slalom::slalomConfig &config, uint32_t level);
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

private:
    //variables
    //static slalom parameters in cm
    double FIRST_DIST_ = 1.25;
    double CONE_DIST_ = 1.8;
    double OFF_SET_Y_ = 0.4;
    double OFF_SET_X_ = 0.3;
    double OVERSTEERING_ANGLE_ = 8.0;
    //Parameter for cone detection
    double X_LIM_MIN_ = 0.5;
    double X_LIM_MAX_ = 2.4; //2m look ahead
    double Y_LIM_MIN_ = -1;
    double Y_LIM_MAX_ = 1;
    double CLUSTER_TOLERANCE_ = 0.04; //4cm
    int CLUSTER_MIN_ = 8;
    int CLUSTER_MAX_ = 25;
    ros::Time current_time_;
    //ros variables
    ros::NodeHandle node_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    ros::Publisher point_cloud_publisher_;
    ros::Subscriber init_pos_sub_, cloul_sub_;
    double distance_;
    //functions
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void initPosCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void setTrajectory(); 
    bool angleInTolerance(Eigen::Vector4f centroid);
    void saveCone(Eigen::Vector4f centroid);
    geometry_msgs::PointStamped transformPoint(geometry_msgs::PointStamped);
    geometry_msgs::PoseStamped calculateAngleBetweenPoints(geometry_msgs::PoseStamped, geometry_msgs::PoseStamped);

};
//End Class Definition
