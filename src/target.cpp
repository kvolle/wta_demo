#include "target.h"

Target::Target(ros::NodeHandle nh, float pk): desired_pk(pk),m_n(nh) {

    transform.transform.translation.x = 0.;
    transform.transform.translation.y = 0.;
}
void Target::subscribe(std::string t) {
    this->topic = t;
    targetTransform = m_n.subscribe(t, 10, &Target::targetTransformCallback, this);
}

void Target::targetTransformCallback(const geometry_msgs::TransformStamped_<std::allocator<void> >::ConstPtr &msg) {
    this->transform = *msg;
}
