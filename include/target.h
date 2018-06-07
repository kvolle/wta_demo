#ifndef INCLUDE_TARGET_H
#define INCLUDE_TARGET_H
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "ros/subscription.h"
#include <geometry_msgs/TransformStamped.h>

/**
 * @brief Target struct
 * @detials Structure for holding information pertaining to a given target
 */
class Target {
    public:
        ros::NodeHandle m_n;
        ros::Subscriber targetTransform;
        geometry_msgs::TransformStamped transform;
        float desired_pk=0.0;
        std::string topic;
        void targetTransformCallback(const geometry_msgs::TransformStamped::ConstPtr &);
        void subscribe(std::string t);
        Target(ros::NodeHandle nh, float pk);
};

#endif  // INCLUDE_TARGET_H
