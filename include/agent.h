#ifndef INCLUDE_AGENT_H
#define INCLUDE_AGENT_H

#include <geometry_msgs/TransformStamped.h>
#include "kobuki_msgs/RobotStateEvent.h"
#include "string"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "ros/subscription.h"
#include "ros/publisher.h"
#include "wta_demo/StateMsg.h"
#include "target.h"
/**
* @struct Location
* @details Structure for holding (x,y) of object as floats
*/
struct Location {
    float x_pos; /**< Holds the x-axis value of the position */
    float y_pos; /**< Holds the y-axis value of the position */
};

/**
* @struct Model
* @details Used for message passing.  ADD MORE KYLE.
*/
struct Model {
        Location position;  /**< TBD */
        int target_id;  /**< TBD */
        float effectiveness;
        int confidence_index;  /**< TBD */
        float attrition_estimate;  /**< TBD */
        bool ready;  /**< TBD */
        int heartbeats;  /**< TBD */
        Model():ready(false),target_id(0),heartbeats(0){}  /**< TBD */
        //ros::Subscriber state_sub;
};
class Agent {
    public:
        // The following are stolen from Env_Obj
        ros::NodeHandle m_n;
        Location location_marker;
        ros::Subscriber StateSubscriber;
        int id;


        bool ready; /**< Represents if the bot is ready to move to target or not */
        int num_targets; /**< The number of targets in the environment */
        int num_agents; /**< The number of agents in the environment */
        std::vector<Target> all_targets; /**< A vector holding all of the target's in the environment */
        std::vector<float> pk_on_targets; /**< A vector holding all of the target Pk values */
        wta_demo::StateMsg msg; /**< A message that holds the x and y position of the bot*/
        std::vector<Model> models; /**< TBD - KYLE */
        std::vector<ros::Subscriber> state_msg_subscriptions; /**< A vector of subscribers to the other turtlebots in the environment */
        float max_cost; /**< TBD - KYLE */

        ros::Subscriber hardwareSubscriber; /**< TBD - KYLE */
        ros::Publisher publisher; /**< Publishes this agent's state message */
        Location actual_goal; /**< The location of the agent's selected target */
        ros::Publisher desired_state; /**< The position of the agent's desired target */
        geometry_msgs::TransformStamped desired_state_msg; /**< TBD - KYLE */
        std::string desired_state_handle;  /**< String containing name of desired state topic */
        std::vector<float> effectiveness; /**< TBD - KYLE */

        ros::Subscriber goalPoseSubscriber; /** < Listens to the current goal **/
        void goalPoseCallback(const geometry_msgs::TransformStamped::ConstPtr &);

        Agent(ros::NodeHandle& n, ros::NodeHandle& nPrivate, int id,int target_number,int agent_number);  /**<Constructor of Agent class */
        ~Agent();  /**< Destructor of Agent class */
        void ownPositionCallback(const geometry_msgs::TransformStamped::ConstPtr& ); /**< Updates bot's location and location value inside Model */
        void stateCallback(const wta_demo::StateMsg::ConstPtr&); /**< Stores turtlbot's position while updating target, attrition value, heartbeat counter, and bot's ready state */
        void hardwareStateCallback(const kobuki_msgs::RobotStateEvent::ConstPtr&); /**< Sets bot to ready when turtlebot base moves into state 3 */
        void subscribeToTopic(ros::Subscriber, std::string); /**< Lets bots subscribe to other bots running in the environment */
        void broadcast(); /**< Broadcasts a message to all other agents */
        void pk_from_model(); /**< Creates an estimate of Pk against all targets */
        void decision_function(); /**< KYLE FIX */
        void simulated_annealing(); /**< Used for agent selection of target because it examines what happens if the bot changes targets */
        float get_distance(Location , Location ); /**< Takes the position of two items and calculates distance.  Still needed? */
        float attrition_estimate(int); /**< Estimates probability of attrition */
        float cost_function(std::vector<float> &); /**< Simple cost function KYLE FIX */
        float cost_function_tiers(std::vector<float> &); /** Cost function using tiers (hardcoded) */
};
#endif  // INCLUDE_AGENT_H
