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

    //! Tf2 stuff for extrinsic matrix.
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped t_trans_par_to_child;

    ROS_INFO("Waiting for transform from velodyne to cam01"); 

    try
    {
        ros::Duration(1.0).sleep();
        t_trans_par_to_child = tfBuffer.lookupTransform("velodyne", "cam01", ros::Time(0));
    }

    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(0.1).sleep();
    }

    std::cout << "transformstamp: " << t_trans_par_to_child << std::endl;

    Eigen::Isometry3d transformation_matrix = tf2::transformToEigen(t_trans_par_to_child);

    Eigen::Matrix4d eigen_transformation_matrix = transformation_matrix.matrix();

    std::cout << "Eigen transformation matrix: " << std::endl;

    std::cout << eigen_transformation_matrix << std::endl;

    return 0;
}
