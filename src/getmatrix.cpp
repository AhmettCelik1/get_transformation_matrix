#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <tf/transform_datatypes.h>
#include "geometry_msgs/TransformStamped.h"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "getmatrix");
    ros::NodeHandle n;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    geometry_msgs::TransformStamped transformStamped;

    try
    {
        ros::Duration(1).sleep();
        listener.lookupTransform("velodyne", "cam01", ros::Time(0), transform);
        tf::transformStampedTFToMsg(transform, transformStamped);
    }

    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    // print transformStamped
    std::cout << transformStamped << std::endl;


    Eigen::Isometry3d eigen_transform;
    tf2::transformToEigen(transformStamped);

    //print eigen_transform
    std::cout << eigen_transform.matrix() << std::endl;

    

}
