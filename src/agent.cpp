#include "../include/agent.h"

/**
*************************************************************************************************************
* @brief This file contains the subscriptions and broadcasts needed for a turtlebot to choose its goal based
* on other turtlebot locations.
*
* Project:  Pathfinder2
* @file agent.cpp
*
* @author Brian Janisch bjanisch@tamu.edu
* @author Allison Holt allison.holt@auburn.edu
* @author Kyle Volle kyle.volle@gmail.com
* @date 08/05/2015
*
* @details TBD - Kyle
*************************************************************************************************************
*/
/**
* @brief Constructor for the Agent class that calls upon the constructor for EnvObj
* Author:  Kyle Volle
* @details TBD - KYLE
*
* @param [in] n ROS specific member variable
* @param [in] nPrivate ROS specific member variable
* @param [in] id The identifier for the EnvObj constructor
* @param [in] target_number The number of targets in the environment
* @param [in] agent_number The number of agents in the environment
*/
Agent::Agent(ros::NodeHandle& n, ros::NodeHandle& nPrivate, int _id,int target_number,int agent_number):ready(false),id(_id)
{
    num_agents = agent_number;
    num_targets = target_number;
    std::string m_poseTopic;
    desired_state = m_n.advertise<geometry_msgs::TransformStamped>("desired_state/pose",1000);
    hardwareSubscriber = n.subscribe("mobile_base/events/robot_state",10,&Agent::hardwareStateCallback,this);
    all_targets.reserve(num_targets + 1);
    models.reserve(num_agents);
//            all_targets[0].StateSubscriber = n.subscribe("/bot/pose",10,&Target::stateCallback,&all_targets[0]);

    float desired=0.;  // Desired Pk
    all_targets.push_back(Target("bot/pose",desired));  // Initialize null target
    for(int t=1;t<=num_targets;t++) {  // from 1 to num_targets because goal0 is for null target
        //subscribe to goal pose
        std::ostringstream target_id;
        target_id << t;
        std::string t_str(target_id.str());
        std::string name_goal = "/goal" + t_str;
        m_poseTopic = name_goal + "/goal_pose";
        if (!n.getParam("/scenario/desired_pk"+name_goal,desired)) {
            desired = 0.;
            std::cerr << "Parameter was not found: Pk_d of " << name_goal << std::endl;
        }
        all_targets.push_back(Target(m_poseTopic, desired));
        //all_targets[t+1].StateSubscriber = n.subscribe(m_poseTopic,10,&Target::stateCallback,&all_targets[t+1]);
    }
    Model tmp_model;
    for (int a=0;a<num_agents;a++)
    {

        models.push_back(tmp_model);

        std::ostringstream id_str_tmp;
        id_str_tmp << a;
        std::string a_str(id_str_tmp.str());
        ros::Subscriber tmp_sub;
        state_msg_subscriptions.push_back(tmp_sub);

        if (a!=id)
        {
            m_poseTopic ="/robot" + a_str +"/state";
            state_msg_subscriptions[a] = n.subscribe(m_poseTopic,10,&Agent::stateCallback,this);
        }
        else
        {
            m_poseTopic = "bot/ned/pose";
            state_msg_subscriptions[a] = n.subscribe(m_poseTopic,10,&Agent::ownPositionCallback,this);
            if (!n.getParam("/scenario/effectiveness/robot"+ a_str,this->effectiveness)) {
                    std::cerr << "Parameter was not found: Effectiveness of /robot" << a_str << std::endl;
                    this->effectiveness.resize(all_targets.size(),0.);
            }
        }
    }
    publisher = n.advertise<wta_demo::StateMsg>("state",1000);
    pk_from_model();
    max_cost = cost_function(pk_on_targets);
    //subscribeToTopic(this->goalPoseSubscriber, all_targets[this->target_id].topic, &goalPoseCallback);
    goalPoseSubscriber = m_n.subscribe(all_targets[models[id].target_id].topic, 10, &Agent::goalPoseCallback, this);
}
void Agent::goalPoseCallback(const geometry_msgs::TransformStamped::ConstPtr &desired_pose) {
    std::cout<<"GoalPose\n";
    desired_state_msg = *desired_pose;
    desired_state.publish(desired_state_msg);
}

/**
* Author:  Kyle Volle
* @details Takes the pose of this bot and stores the x-axis and y-axis values as well as
* updating the model for itself which is used in model propogation
*
* @param [in] pose The turtlebot's position in the environment
*/
void Agent::ownPositionCallback(const geometry_msgs::TransformStamped::ConstPtr& pose) {
    // location_marker is probably redundant
    location_marker.x_pos = pose->transform.translation.x;
    location_marker.y_pos = pose->transform.translation.y;
    models[id].position = location_marker;
    models[id].ready = ready;
}

/**
* Author:  Kyle Volle
* @details The destructor for the Agent class.  It is currently empty.  It was used
* for debugging purposes.
*/
Agent::~Agent()
{

}

/**
* Authors:  Brian Janisch and Kyle Volle
* @details Takes the pose of a turtlebot and stores the x-axis and y-axis values. It also updates the agent's ready state,
* current target, and attrition estimate.  Finally, it resets the heartbeat counter for that agent.
*
* @param [in] pose A custom state message from one of the other bots
*/
void Agent::stateCallback(const wta_demo::StateMsg::ConstPtr& pose) {
    // This function should only be used for receiving messages that this bot didn't publish
    if (pose->agent_id != id)
    {
        models[pose->agent_id].position.x_pos= pose->x_state;
        models[pose->agent_id].position.y_pos= pose->y_state;
        models[pose->agent_id].ready = pose->ready;
        models[pose->agent_id].target_id = pose->target_id;
        models[pose->agent_id].attrition_estimate = pose->attrition_estimate;
        models[pose->agent_id].effectiveness = pose->effectiveness;
        models[pose->agent_id].heartbeats=0;
        for (unsigned int a=0; a < models.size(); a++) {
            std::cout << models[a].target_id << " ";
        }
        std::cout << std::endl;

        pk_from_model();
        for (unsigned int t=0; t < pk_on_targets.size(); t++) {
            std::cout << pk_on_targets[t] << " ";
        }
        std::cout << std::endl;
      //std::cout<<"agent ID  "<<pose->agent_id<<std::endl;
      //std::cout<<"models.ready in stateCallback  "<<models[pose->agent_id].ready<<std::endl;
    }
}

/**
* Author:  Kyle Volle
* @details This callback makes the bot "ready" when the bot is in full mode
* updating the model for itself which is used in model propogation
*
* @param [in] state The turtlebot's current state in the environment
*/
void Agent::hardwareStateCallback(const kobuki_msgs::RobotStateEvent::ConstPtr& hw_state) {
    ready = (hw_state->state == 1);
    this->models[this->id].ready = this->ready;
}

/**
* Author:  Brian Janisch
* @details This function allows the turtlebot to subscribe to all the other turtlebots
* in the environment and all the goals in the environment.
*
* @param [in] m_poseTopic The name of the topic to subscribe to
*/
void Agent::subscribeToTopic(ros::Subscriber sub,std::string m_poseTopic)
{
    //StateSubscriber= m_n.subscribe(m_poseTopic, 10, &Agent::stateCallback,this);
    sub = m_n.subscribe("bot/pose",10,&Agent::ownPositionCallback,this);
}

/**
* Author:  Kyle Volle
* @details Broadcasts a message to all other agents. For the moment, we assume perfect communication.
* Explain what the message is and why this needs to happen, KYLE.
*/
void Agent::broadcast()
{
    msg.x_state = location_marker.x_pos;
    msg.y_state = location_marker.y_pos;
    msg.agent_id = id;
    msg.ready = ready;
    msg.target_id = models[id].target_id;
    std::cout << "\t\t["<< msg.target_id <<"]]\n";
    msg.attrition_estimate = models[id].attrition_estimate;
    msg.effectiveness = models[id].effectiveness;
    // This commented out section is for the daisy chaining of messages, which may or may not be implemented eventually
    /*
    for (uint a = 0;a<models.size();a++)
    {
        msg.models[a].target_id = models[a].target_id;
        msg.models[a].x_pos = location_marker.x_pos;
        msg.models[a].y_pos = location_marker.y_pos;
        msg.models[a].attrition_estimate = models[a].attrition_estimate;
        msg.models[a].ready = models[a].ready;
    }*/
    publisher.publish(msg);
}

/**
 * Author:  Kyle Volle
 * @details This function takes the model built up from communication and creates an estimate of Pk against all targets.
 */
void Agent::pk_from_model() {
    /*
    pk_on_targets.clear();
    //pk_on_targets.resize(all_targets.size(),0.0f);
    for (uint t=0;t<pk_on_targets.size();t++)
    {
        for (uint a=0;a<models.size();a++)
        {
            if((int)t == models[a].target_id)
            {
                // Hardcoding effectivenesses
                // TODO remove this
                pk_on_targets[t] = 1.0f-(1.0f-pk_on_targets[t])*(1.0f - 0.5f + 0.5f*models[a].attrition_estimate);
            }
            //models[a].ready = false;
        }
    }
    */
    pk_on_targets.clear();
    pk_on_targets.resize(all_targets.size(),0.0);
    for (uint a=0; a < models.size(); a++) {
        pk_on_targets[models[a].target_id] = 1.0 - (1.0-pk_on_targets[models[a].target_id])*(1 - models[a].effectiveness + models[a].effectiveness*models[a].attrition_estimate);
    }
}

/**
 * Author:  Kyle Volle
 * @details This function takes considers the results of this agent changing targets unilaterally and selects the target that
 * results in the best possible choice assuming no other agents change their targets.  KYLE FIX.
 */
void Agent::simulated_annealing()
{
    // First check that all surviving agents are in full mode
    bool ready_check = ready;
    for (uint a=0;a<num_agents;a++)
    {
        ready_check = ready_check & models[a].ready;
    }

    // This function takes the model made from messages and calculates the Pk
    pk_from_model();
    if(ready_check)
    {
        float plan_cost, min_cost,tmp_attrition;
        int new_target;
        std::vector<float> result_pk = pk_on_targets;
        new_target= models[id].target_id;
        min_cost = cost_function(result_pk);

        //int t = rand()%all_targets.size();
        int t = 0;
        result_pk = pk_on_targets;
        tmp_attrition = attrition_estimate(t);
        result_pk[models[id].target_id] = 1.0f - (1.0f- result_pk[models[id].target_id])/(1.0f-effectiveness[models[id].target_id]+effectiveness[models[id].target_id]*models[id].attrition_estimate);
        result_pk[t] = 1.0f - (1.0f-result_pk[t])*(1.0f - effectiveness[t]+effectiveness[t]*tmp_attrition);
        //std::cout << "Goal "<< t << " Plan: " << result_pk[0] << "  " <<result_pk[1] << "  "  << result_pk[2] << std::endl;
        //std::cout << "Target of Bot1: " << models[1].target_id << " Target of Bot0: " << models[0].target_id << std::endl;

        plan_cost = cost_function(result_pk);
        if (plan_cost < min_cost)
        {
            min_cost = plan_cost;
            new_target = t;
        }
        else
        {
            float acceptance = (2.0*min_cost/max_cost)*exp((min_cost-plan_cost)/1.0f);
            if ((rand()%1000)< 1000*acceptance)
            {
                min_cost = plan_cost;
                new_target = t;
            }
        }

        //printf("Selected target %d with cost of %5.4f\n",new_target,min_cost);
        //actual_goal = all_targets[new_target].location_marker;
        models[id].target_id = new_target;
        models[id].attrition_estimate = attrition_estimate(new_target);

        // Publish the location the selected target
        /*
        desired_state_msg.transform.translation.x = actual_goal.x_pos;
        desired_state_msg.transform.translation.y = actual_goal.y_pos;
        desired_state.publish(desired_state_msg);
        */
        //goalPoseSubscriber = n.subscribe(all_targets[models[id].target_id].topic, 10, &Agent::goalPoseCallback, this);
    }
}

/**
 * Author:  Kyle Volle
 * @details This function takes considers the results of this agent changing targets unilaterally and selects the target that
 * results in the best possible choice assuming no other agents change their targets.  KYLE FIX.
 */
void Agent::decision_function()
{
    // First check that all surviving agents are in full mode
    bool ready_check = ready;
    for (uint a=0;a<num_agents;a++) {
        ready_check = ready_check & models[a].ready;
    }
    // This function takes the model made from messages and calculates the Pk
    pk_from_model();
    if(ready_check) {
        float plan_cost, min_cost,tmp_attrition;
        int new_target;
        std::vector<float> result_pk = pk_on_targets;
        new_target= models[id].target_id;
        min_cost = cost_function(result_pk); // cost_function(result_pk) or:cost_function_tiers(result_pk); // changed to without tiers June 09, 2016


        // Go through each potential target and calculate the cost function for that assignment
        // When this for-loop exits, the minimum cost target will be selected. Will not be worse than current assignment
        for (uint t=0;t<all_targets.size();t++)
        {
            result_pk = pk_on_targets;
            tmp_attrition = attrition_estimate(t);
            result_pk[models[id].target_id] = 1.0f - (1.0f- result_pk[models[id].target_id])/(1.0f-effectiveness[models[id].target_id]+effectiveness[models[id].target_id]*models[id].attrition_estimate);
            result_pk[t] = 1.0f - (1.0f-result_pk[t])*(1.0f - effectiveness[t]+effectiveness[t]*tmp_attrition);
            std::cout << "Goal "<< t << " Plan: " << result_pk[0] << "  " <<result_pk[1] << "  "  << result_pk[2] << std::endl;
            plan_cost = cost_function(result_pk); // or: cost_function(result_pk); //cost_function_tiers(result_pk);
            std::cout << "Target of Bot:0 " << models[0].target_id << " Target of Bot1: " << models[1].target_id  << " for cost of " << plan_cost<< std::endl;

            if (plan_cost < min_cost)
            {
                min_cost = plan_cost;
                new_target = t;
            }
         }
        //printf("Selected target %d with cost of %5.4f\n",new_target,min_cost);
        //actual_goal = all_targets[new_target].location_marker;
        models[id].target_id = new_target;
        models[id].attrition_estimate = attrition_estimate(new_target);
        models[id].effectiveness = effectiveness[new_target];
        goalPoseSubscriber = m_n.subscribe(all_targets[models[id].target_id].topic, 10, &Agent::goalPoseCallback, this);
        std::cout << "Subscribed to " << all_targets[new_target].topic << std::endl;
    }
    /*
        // publish my ready flag
    ready = 1;
    int ready_check = ready;
        //std::cout << "ready_check\n";
        for (uint a=0;a<num_agents;a++)
        {
          if (a!=id){
        ready_check = ready_check & models[a].ready;
        std::cout<<"a  "<<a<<"id  "<<id<<"models "<<models[a].ready<<std::endl;
                    }
        }
        std::cout<<"ready_check   "<<ready_check<<std::endl;
        */
        // subscribe to StateMsg and get ready flag

        // check if ready flag is set to true in all agents
        // if that is true {
    /*if(ready_check)  {
        // Publish the location the selected target
        desired_state_msg.transform.translation.x = actual_goal.x_pos;
        desired_state_msg.transform.translation.y = actual_goal.y_pos;
        desired_state.publish(desired_state_msg);
    }*/
}

/**
* Author:  Brian Janisch
* @details The method takes in the position of two items and calculates the distance between them.
*
* @param [in] item1 The position of the first object you want to use to calculate distance
* @param [in] item2 The position of the second object you want to use to calculate distance
* @param [out] distance The distance between the two objects
*/
float Agent::get_distance(Location goal, Location bot)
{
    float distance;
    distance = sqrt( pow((goal.x_pos-bot.x_pos),2) + pow((goal.y_pos-bot.y_pos),2) ); // sqrt((x1-x2)^2 + (y1-y2)^2)
    return distance;
}

/**
  * Author: Kyle Volle
  * @details This method takes in the id of a target and estimates the probability of attrition for the turtlebot to reach that target.
  *
  * @param [in] target_id The unique identifier for the relevant target
  * @param [out] attrition Estimated probability of attrition based on straight line path and constant rate of attrition
  */
float Agent::attrition_estimate(int target_id)
{
    const float ATTRITION_RATE = 0.01;
    //float distance = get_distance(location_marker,all_targets[target_id].location_marker);
    float distance = 0.0;
    float attrition = 1-pow(1-ATTRITION_RATE,distance);
    return attrition;
}

/**
  * Author: Kyle Volle
  * @details Simplest cost function from simulation work.  But how does it work???  KYLE FIX
  *
  * @param [in] pk This vector represents the resulting Pks on each target if the plan being evaluated is adopted
  * @param [out] cost This float is a measure of the quality of the plan
  */
float Agent::cost_function(std::vector<float> & pk)
{
    float cost = 0.0f;
    for (uint t=0;t<pk.size();t++)
    {
        if(pk[t]<all_targets[t].desired_pk)
        {
            cost+=(all_targets[t].desired_pk - pk[t])/(1-all_targets[t].desired_pk);
        }
    }
    return cost;
}
/**
  * Author: Kyle Volle
  * @details Simplest cost function from simulation work.  But how does it work???  KYLE FIX
  *
  * @param [in] pk This vector represents the resulting Pks on each target if the plan being evaluated is adopted
  * @param [out] cost This float is a measure of the quality of the plan
  */
float Agent::cost_function_tiers(std::vector<float> & pk)
{
    float cost = 0.0f;
    /*if (pk[2]<all_targets[2].desired_pk)
    {
    cost+=(all_targets[2].desired_pk - pk[2])/(1.0-all_targets[2].desired_pk);
    cost+=1.0; // add a penalty for ignoring the low prior.
    }
    else
    {
    if (pk[1]<all_targets[1].desired_pk)
    {
        cost+=(all_targets[1].desired_pk - pk[1])/(1.0-all_targets[1].desired_pk);
    }
    }*/

    return cost;
}
