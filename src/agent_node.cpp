#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"

//#include "../include/bot_communicator.h"
#include "../include/agent.h"
//#include "../include/target.h"

#include <cmath>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <typeinfo>
#include <vector>

/**
*************************************************************************************************************
* @brief This file contains the logic needed for a turtlebot to choose its goal based on other turtlebot
* locations.
*
* Project:  Pathfinder2
* @file agent_node.cpp
*
* @author Brian Janisch bjanisch@tamu.edu
* @author Allison Holt allison.holt@auburn.edu
* @author Kyle Volle kyle.volle@gmail.com
* @date 08/07/2015
*
* @details Agent node allows a turtlebot to analyze its position, the position of the other turtlebots,
* and the Pk and attrition of the goals in order to determine which goal each turtlebot should move towards.
* As of now, the goals chosen are based on the maximum Pk.
*************************************************************************************************************
*/

/**
* Author:  Brian Janisch
* @details The main function grabs the parameters from the launch file (turtlebot name, number of turtlebots, and
* number of goals) and stores them in an agent object.  Using that information and the information given
* from all the subscribers, the main method calculates the different options for the Pk and attrition values between
* the turtlebots and the goals.  The main method will select the target with the highest probability of success.  The
* turtlebots are constantly reevaluating their choice, so, if something in the environment changes, then the turtlebots
* will adapt.
*
* @param [in] argc The number of arguments passed in from the command line
* @param [in] argv An vector containing the arguments passed in froctum the command line
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_selector");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    std::string BOT_ID;
    int NUM_GOALS, NUM_BOTS;

    geometry_msgs::TransformStamped msg; /** The message type to publish desired_state after cost function evalulation */

    /* Get params from launch file */


    if (!n_private.getParam("bot_id", BOT_ID))
    {
        std::cout <<"Couldn't get bot_id, setting default..." <<std::endl;
        BOT_ID = "robot0";
    }

    if (!n_private.getParam("num_goals", NUM_GOALS))
    {
        std::cout <<"Couldn't get num_goals, setting default..." <<std::endl;
        NUM_GOALS = 2;
    }

    if (!n_private.getParam("num_bots", NUM_BOTS))
    {
        //std::cout <<"Couldn't get num_bots, setting default..." <<std::endl;
        NUM_BOTS= 2;
        std::cout <<"Couldn't get num_bots, setting default..." <<std::endl;
    }
    NUM_GOALS = 2;
    NUM_BOTS = 2;  //NOT SURE HOW I"M GONNA DO THIS
    std::string tmp = BOT_ID.substr(BOT_ID.length()-1,1);
    Agent agent(n,n_private,atoi(tmp.c_str()),NUM_GOALS,NUM_BOTS);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        agent.broadcast();
//        agent.desired_state.publish(agent.desired_state_msg);
//        std::cout<<"ready"<<agent.ready<<std::endl;  //
        //if (agent.ready)
        //{
            // This rand() function is to prevent a race condition resulting in agents alternating quickly
            //if (rand()%100<50) // commented June 09, 2016
            //{
                agent.decision_function();
               // agent.simulated_annealing();
            //}
        //printf("xpos: %4.3f\nypos: %4.3f\ntarget: %d\n",agent.models[agent.id].position.x_pos,agent.models[agent.id].position.y_pos,agent.models[agent.id].target_id);
        //}
/*
        // If another agent hasn't been heard from in 6 cycles, assume it is dead
        for (uint a=0;a<NUM_BOTS;a++)
        {
          //std::cout << "BOT 0 <3: " << agent.models[0].heartbeats << "  BOT 1 <3: " << agent.models[1].heartbeats << "\n";

          if ((a!=agent.id)&&(agent.models[a].ready==1)&&(agent.models[a].heartbeats >10))
          {
            // Attrited agent unable to reach its target, this alone should be enough
            agent.models[a].attrition_estimate = 1.0;
            // Setting the desired target to null should also be enough to accomplish the goal
            agent.models[a].target_id = 0;
            agent.models[a].ready = true;
            //std::cout << "Agent " << a<< " is assumed to be targeting 0*****\n";
          }

          if (a!=agent.id)
          {
            agent.models[a].heartbeats++;
          }
        }
        */
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
