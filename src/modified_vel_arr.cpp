// components basic vel arr uses
#include <bits/stdc++.h>
#include <fstream> // kaha use?
#include <algorithm> // kaha use?
#include <math.h> // kaha use?
#include "ros/ros.h"
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

// components ros action server use
#include <actionlib/server/simple_action_server.h>
#include <igvc_self_drive_gazebo/stoplineAction.h>

using namespace std;

int a = 1;          // max accl; unit not sure // CHANGE THE VARAIBLE NAME
float vmax = 2.2;   // in m/s unit from 5mph // MAKE A SUBSCRIBER TO UPDATE THIS!

std_msgs::Float64MultiArray Vel;
float currentvel = 2.2;
double cvStopLineDistance = 2.4;
int stopLineIndexOnPath;

ros::Publisher pub_vel_arr;
bool stopBehaviourTrigger;

float distTwoWaypoints = 0.1;

/*
Can implement a ros server here!
Make its method and and an callback
callback mein could make trigger
Could be easier to give feedback too
and hence the result!
*/
class StopBehaviour
{
// protected:
public:
    ros::NodeHandle stopAction_nh;
    actionlib::SimpleActionServer<igvc_self_drive_gazebo::stoplineAction> actionServer;
    
    std::string actionName;

    igvc_self_drive_gazebo::stoplineFeedback feedback;
    igvc_self_drive_gazebo::stoplineResult result;

public:
    StopBehaviour(std::string name):actionServer(stopAction_nh, name,
                                    boost::bind(&StopBehaviour::executeCB, this, _1),false),actionName(name)
    {
        actionServer.start();
    }

    ~StopBehaviour(void)
    {
    }

    void executeCB(const igvc_self_drive_gazebo::stoplineGoal::ConstPtr &goal)
    {
        ros::Rate rate(1);
        bool success = true;

        ROS_INFO("%s: Executing, trigger is true and we wish to stop before the distance of %ld", actionName.c_str(), goal->stopBeforeDistance);
        while ( (cvStopLineDistance > (goal->stopBeforeDistance)) && (currentvel > 0.1) )
        {   cout << "cvStopLineDistance:" << cvStopLineDistance <<"\n";

            if(actionServer.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", actionName.c_str());
                actionServer.setPreempted();
                success =  false;
                break;
            }

            feedback.floatDistStopLine = cvStopLineDistance;
            feedback.stopLineIndexOnPath = stopLineIndexOnPath;
            stopBehaviourTrigger = true;
            actionServer.publishFeedback(feedback);
            cout << "stopLineIndexonPath:" << stopLineIndexOnPath << "\n";
            //cvStopLineDistance-=0.1;
            rate.sleep();
        }
        if(success)
        {
            if (currentvel >= 0.1)
            {
                ROS_INFO("velocity not zeroed out, Immediate breaks");
                // DO SMTHING FOR IMMEDIATE BREAKS
                success = false;
                result.immediateBreak = true;
            }
            if (cvStopLineDistance < goal->stopBeforeDistance)
            {
                ROS_INFO("Stop Threshold violated");
                result.crossedThreshold = true;
                
            }
            if (cvStopLineDistance - goal->stopBeforeDistance > 1)
            {
                ROS_INFO("need to move thoda aage");
                success = false;
                result.StoppedBefore = true;
            }
            result.success = success;
            ROS_INFO("%s Succeeded:", actionName.c_str());
            actionServer.setSucceeded(result);
            stopBehaviourTrigger = false;
        }
        
    }

};

vector<double> SignumScalingFunction(std_msgs::Float64MultiArray *Vel, int stopLineIndexOnPath)
{   
    cout << "Signum Scale: scale function called\n";
    vector<double>vec ;
    float m, f;

    for(int i=0; i<=(*Vel).data.size(); i++)
    {
        m = -currentvel/ stopLineIndexOnPath;
        cout << "m: " << m << "\n";

        f = m * i + currentvel;
        
        if (f<0)
            {
              f = 0.0;    
            }
        cout << "f: " << f << "\nvel.data[" << i << "]: " <<  ((*Vel).data[i]) << "\n";
        // (*Vel).data.insert((*Vel).data.begin() + i , f); 
        vec.push_back(f);

    }
    return vec;

}

void CV_Stopline_callback(const std_msgs::Float64::ConstPtr& msg)
{
    cvStopLineDistance = msg->data;
}

void CurrentVel(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    float vx = msg->twist[11].linear.x; // 11?? 
    float vy = msg->twist[11].linear.y;
    currentvel = sqrt((vx*vx)+(vy*vy));   // pyth why?
    std::cout << "CALLED!!!! Current Vel: "<<currentvel << std::endl;
}

void PathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    int NumData = msg->poses.size();
    // cout<<(NumData)<<endl;
    vector<double> PathDis;
    PathDis.push_back(0.0);
    for(int i=1; i<=NumData; i++)
    {
        double x1 = msg->poses[i-1].pose.position.x ,x2 = msg->poses[i].pose.position.x;
        double y1 = msg->poses[i-1].pose.position.y ,y2 = msg->poses[i].pose.position.y;
        PathDis.push_back(PathDis[i-1]+sqrt(((x2-x1)*(x2-x1))+((y2-y1)*(y2-y1))));

        /*
        if trigger :
            compare PathDis to Actual cv stop distance
            find the index position to stop
        */
        if (stopBehaviourTrigger)
        {
            if ( (cvStopLineDistance >= PathDis[i-1]) && (cvStopLineDistance < PathDis[i]) )
            {
                stopLineIndexOnPath = i-1;
            }
        }
    }
    std::cout << "First Push: "<< currentvel << std::endl; 
    Vel.data.push_back(currentvel);
    float u = Vel.data[0]; 
    for(int i=1; i<=NumData; i++)
    {   
        float v;

        if (u*u - 2*a*PathDis[i] > 0){
            v = sqrt((u*u) - (2*a*PathDis[i]));
        }

        else{
            v = 0;
        }
        
        if(v < vmax && cvStopLineDistance < 10)
        {        
            Vel.data.push_back(v);
        }
        else
        {
            Vel.data.push_back(vmax);
        }
    }

    /*
    Implement the zero ing velocity till k th index 
    an zero always after that!
    k - threshold_index actually!! (threshold index by manager (based upon the task!))
    
    */
    if (stopBehaviourTrigger)
    {
        Vel.data = SignumScalingFunction(&Vel, stopLineIndexOnPath);
    }

    for(int i = 0; i < 8; i++){
        std::cout << i << " " << Vel.data[i] << '\n';
    }
	pub_vel_arr.publish(Vel);	
    Vel.data.clear();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vel_arr");
	ros::NodeHandle RosNodeH;

    ros::Subscriber something = RosNodeH.subscribe("/gazebo/model_states",1,CurrentVel); // CHAGE THE SUB NAME
	ros::Subscriber sub_path = RosNodeH.subscribe("/best_path",1,PathCallback); 

    ros::Subscriber stopLineDistance_Sub = RosNodeH.subscribe("dm/distance", 1000, CV_Stopline_callback);
    pub_vel_arr = RosNodeH.advertise<std_msgs::Float64MultiArray>("/best_velocity", 1);			

	ros::Rate loop_rate(4);
    StopBehaviour stopbehave("stop_action");

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}



// keep a practical measure of how par the points are in the path recieved
// so that jab saftey laye path dis mein i ka daal rhe h toh kitna faraq pad rha h
// client jaha par bhi likhe maanager script just ensure client se goal tabhi call ho jab measured distance is 
// just within the reach when the path is recieved

/*
(cvStopLineDistance > goal->StopBeforeDistance) || (currentvel > 0.1) )
4. yes || no : yes :: gaadi aage hi nhi badhegi..invalid sa case h
3. no || yes: yes :: IMMEDIATE BREAK!
1. yes|| yes: yes :: WORKS
2. no || no : no :: thik h..par loop break
*/

/*
std_msgs::Float64MultiArray SignumScalingFunction(std_msgs::Float64MultiArray *Vel, int stopLineIndexOnPath)
{   
    cout << "scale function called";
    std_msgs::Float64MultiArray manipulated_vel;
    float k_dist = stopLineIndexOnPath * distTwoWaypoints;
    float a = k_dist / 2;
    float n = k_dist / 10;
    float m = (pathdis_index_array - a)/n;
    float j =  np.exp(-1*m); // exp cpp
    float l = 1 + j;
    float m, j, l , f, f_orig;
    for(int i=0; i<=(*Vel).size(); i++)
    {
        m = (i-a)/n;
        j = exp(-1*m);
        l = 1 + j;
        f = (-1/l) + 1;
        f_orig = f * ((*Vel)[i]);
        
        manipulated_vel.data.push_back(f_orig);   
    }
    return manipulated_vel;
*/

/*
float k_dist = stopLineIndexOnPath * distTwoWaypoints;
    float a = k_dist / 2;
    float n = k_dist / 10;
    // float m = (pathdis_index_array - a)/n;
    // float j =  np.exp(-1*m); // exp cpp
    // float l = 1 + j;
    float m, j, l , f, f_orig;
    for(int i=0; i<=(*Vel).data.size(); i++)
    {
        m = (i-a)/n;
        cout << "m: " << m << "\n";
        j = exp(-1*m);
        cout << "j: " << j << "\n";
        l = 1 + j;
        cout << "l: " << l << "\n";
        f = (-1/l) + 1;
        cout << "f: " << f << "\n";
        f_orig = f * ((*Vel).data[i]);
        cout << "f: " << f << "vel.data[i]: " <<  ((*Vel).data[i]) << "f_orig: " << f_orig << "\n";
        (*Vel).data.insert((*Vel).data.begin() + i , f_orig);   
    }
*/

/*
ros::NodeHandle distance_to_stop_line;
 */