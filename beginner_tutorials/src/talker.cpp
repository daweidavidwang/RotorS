#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include <ros/console.h>
#include "RVO.h"
#include <sstream>
#ifndef RVO_OUTPUT_TIME_AND_POSITIONS
#define RVO_OUTPUT_TIME_AND_POSITIONS 1
#endif

#include <random>
#include <time.h>
#include <cmath>
#include <cstddef>
#include <vector>
#include <string>
#include <math.h>
#include <ctime>
#include <chrono>
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */


#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif
std::clock_t start;
const int NUM_ROBOT = 200;
const float env_size = 50;
const int scenario_type =3;
int x = 0; //using in UNIFORM function 
double robot_pos[NUM_ROBOT][3];
double path_length[NUM_ROBOT];
double path_optimal[NUM_ROBOT];
double arrive_time[NUM_ROBOT];
double arrive_time_optimal[NUM_ROBOT];
double robot_vel[NUM_ROBOT][3];
bool obstacle_flag[NUM_ROBOT];
bool arrive_flag[NUM_ROBOT];
/* Store the goals of the agents. */
std::vector<RVO::Vector3> goals;
double start_location[NUM_ROBOT][3];
RVO::RVOSimulator *sim = new RVO::RVOSimulator(0,10,10,1,0.4,2);
std::string int2str(const int inttmp)
{
  std::string string_tmp;
  std::string result;
  std::stringstream stream;
  stream<<inttmp;
  string_tmp = stream.str();
  if(string_tmp.length()==1)
    result = "00"+string_tmp;
  else if(string_tmp.length()==2)
    result = "0" + string_tmp;
  else 
    result = string_tmp;
  return result;
}
double distance(double *ind_a, double *ind_b)
{
  return sqrt(pow((ind_a[0]-ind_b[0]),2)+pow((ind_a[1]-ind_b[1]),2)+pow((ind_a[2]-ind_b[2]),2));
}
void setupScenario(RVO::RVOSimulator *sim,ros::Publisher &control_pub)
{
	/* Specify the global time step of the simulation. */
	sim->setTimeStep(0.125f);

	/* Specify the default parameters for agents that are subsequently added. */
	sim->setAgentDefaults(15.0f, 10, 10.0f, 2.0f, 2.0f);
  if(scenario_type == 0)
  {

	/* Add agents, specifying their start position, and store their goals on the opposite side of the environment. */

    int countnum = 0;
	for (float a = 0; a < M_PI; a += 0.1f) {
		const float z = 100.0f * std::cos(a);
		const float r = 100.0f * std::sin(a);

		for (size_t i = 0; i < r / 2.5f; ++i) {
      countnum++;
			const float x = r * std::cos(i * 2.0f * M_PI / (r / 2.5f));
			const float y = r * std::sin(i * 2.0f * M_PI / (r / 2.5f));

			sim->addAgent(RVO::Vector3(x, y, z));
      gazebo_msgs::ModelState control_msg;
      control_msg.model_name = std::string("Robot_") + int2str(countnum);
      control_msg.pose.position.x = x;
      control_msg.pose.position.y = y;
      control_msg.pose.position.z = z;
      control_msg.reference_frame = "world";
      control_pub.publish(control_msg);
			goals.push_back(-sim->getAgentPosition(sim->getNumAgents() - 1));
      RVO::Vector3 tmp = -sim->getAgentPosition(sim->getNumAgents() - 1);
      control_msg.model_name = std::string("Goal_") + int2str(countnum);
      control_msg.pose.position.x = tmp.x();
      control_msg.pose.position.y = tmp.y();
      control_msg.pose.position.z = tmp.z();
      control_msg.reference_frame = "world";
      control_pub.publish(control_msg);
		}
	}
  }
  else if(scenario_type == 1) //circle scenario
  {
    for(int i = 0; i<NUM_ROBOT;i++)
    {
      float angle = i*2*M_PI/NUM_ROBOT;
      float x = env_size * cos(angle);
      float y = env_size * sin(angle);
      float z = 10;
      sim->addAgent(RVO::Vector3(x, y, z));
      obstacle_flag[i]=0;
      arrive_flag[i] =0;
      path_length[i] = 0;
      gazebo_msgs::ModelState control_msg;
      char name_int [3];
      sprintf(name_int,"%3d",i);
      start_location[i][0] = x;
      start_location[i][1] = y;
      start_location[i][2] = z;
      control_msg.model_name = std::string("Robot_") + name_int;
      control_msg.pose.position.x = x;
      control_msg.pose.position.y = y;
      control_msg.pose.position.z = z;
      control_msg.reference_frame = "world";
      control_pub.publish(control_msg);
			goals.push_back(RVO::Vector3(-x*2, -y*2, z));
      control_msg.model_name = std::string("Goal_") + name_int;
      control_msg.pose.position.x = -x*2;
      control_msg.pose.position.y = -y*2;
      control_msg.pose.position.z = z;
      control_msg.reference_frame = "world";
      control_pub.publish(control_msg);
    }
  }
  else if(scenario_type==2) //random scenario
  {
    std::default_random_engine e;
    std::uniform_real_distribution<double> u(-env_size,env_size);
    std::uniform_real_distribution<double> p(1.0,env_size);
    bool succ = 0;
    double sx, sy, gx, gy, sz, gz;
    double goals_tmp[NUM_ROBOT][3];
    int init_count = 0;
    for(int i = 0; i<NUM_ROBOT;i++)
    {
      succ = 0;
      while(!succ)
      {
        sx = u(e);
        sy = u(e);
        gx = u(e);
        gy = u(e);
        sz = p(e);
        gz = p(e);
        double s[3] = {sx,sy,sz};
        double g[3] = {gx,gy,gz};
        succ = 1;
        if(distance(s,g)<0.5)
          succ = 0;
        if(init_count)
        {
          for(int i=0;i<init_count;i++)
            if(distance(s,start_location[i])<2)
              succ = 0;
            if(distance(g,goals_tmp[i])<2)
              succ = 0;
        }
      }
      start_location[i][0] = sx;
      start_location[i][1] = sy;
      start_location[i][2] = sz;
      goals_tmp[i][0] = gx;
      goals_tmp[i][1] = gy;
      goals_tmp[i][2] = gz;
    }
    for(int ind = 0; ind<NUM_ROBOT;ind++)
    {
      obstacle_flag[ind]=0;
      arrive_flag[ind] =0;
      path_length[ind] = 0;
      gazebo_msgs::ModelState control_msg;
      char name_int [3];
      sprintf(name_int,"%3d",ind);
      sim->addAgent(RVO::Vector3(start_location[ind][0], start_location[ind][1], start_location[ind][2]));
      control_msg.model_name = std::string("Robot_") + name_int;
      control_msg.pose.position.x = start_location[ind][0];
      control_msg.pose.position.y = start_location[ind][1];
      control_msg.pose.position.z = start_location[ind][2];
      control_msg.reference_frame = "world";
      control_pub.publish(control_msg);
			goals.push_back(RVO::Vector3(goals_tmp[ind][0], goals_tmp[ind][1], goals_tmp[ind][2]));
      control_msg.model_name = std::string("Goal_") + name_int;
      control_msg.pose.position.x = goals[ind][0];
      control_msg.pose.position.y = goals[ind][1];
      control_msg.pose.position.z = goals[ind][2];
      control_msg.reference_frame = "world";
      control_pub.publish(control_msg);
    }
  }
  else if(scenario_type==3) //ball scenarios
  {
    int parts = int(sqrt(NUM_ROBOT))+1;
    for(int i= 0;i<NUM_ROBOT;i++)
    {
           double stheta = (i%parts)/float(parts) * M_PI;
           double  sfy = int(i/parts)/float(parts) * M_PI*2;
            double sx = env_size * sin(stheta) * cos(sfy);
            double sy = env_size * sin(stheta) * sin(sfy);
            double sz = env_size * cos(stheta) + env_size;
            start_location[i][0] = sx;
            start_location[i][1] = sy;
            start_location[i][2] = sz+env_size;
            obstacle_flag[i]=0;
            arrive_flag[i] =0;
            path_length[i] = 0;
            gazebo_msgs::ModelState control_msg;
            char name_int [3];
            sprintf(name_int,"%3d",i);
            sim->addAgent(RVO::Vector3(sx, sy, sz+env_size));
            control_msg.model_name = std::string("Robot_") + name_int;
            control_msg.pose.position.x = sx;
            control_msg.pose.position.y = sy;
            control_msg.pose.position.z = sz+env_size;
            control_msg.reference_frame = "world";
            control_pub.publish(control_msg);
            goals.push_back(RVO::Vector3(-sx*2, -sy*2, -sz*2+2*env_size));
            control_msg.model_name = std::string("Goal_") + name_int;
            control_msg.pose.position.x = -sx*2;
            control_msg.pose.position.y = -sy*2;
            control_msg.pose.position.z = -sz*2+2*env_size;
            control_msg.reference_frame = "world";
            control_pub.publish(control_msg);
    }
  }
  ros::Rate loop_rate(5); //wait gazebo launch
  loop_rate.sleep();
}

#if RVO_OUTPUT_TIME_AND_POSITIONS
void updateVisualization(RVO::RVOSimulator *sim, ros::Publisher &control_pub)
{
	/* Output the current global time. */
	std::cout << sim->getGlobalTime();
gazebo_msgs::ModelState control_msg;
	/* Output the position for all the agents. */
  //ROS_ERROR("Number in SIM %d" , sim->getNumAgents());
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    std::cout << " " << sim->getAgentPosition(i);
      control_msg.model_name = std::string("Robot_") + int2str(i);
      control_msg.pose.position.x = robot_pos[i][0];
      control_msg.pose.position.y = robot_pos[i][1];
      control_msg.pose.position.z = robot_pos[i][2];
      if(obstacle_flag[i]==0&&arrive_flag[i]==0)
      {
      control_msg.twist.linear.x = sim->getAgentVelocity(i).x(); 
      control_msg.twist.linear.y = sim->getAgentVelocity(i).y(); 
      control_msg.twist.linear.z = sim->getAgentVelocity(i).z();
      }
      else
      {
        control_msg.twist.linear.x = 0; 
      control_msg.twist.linear.y = 0; 
      control_msg.twist.linear.z = 0;
      }
      control_msg.twist.angular.x = 0;
      control_msg.twist.angular.y = 0;
      control_msg.twist.angular.z = 0;
      control_msg.reference_frame = "world";
    control_pub.publish(control_msg);
      control_msg.model_name = std::string("Goal_") + int2str(i);
      control_msg.pose.position.x = goals[i].x();
      control_msg.pose.position.y = goals[i].y();
      control_msg.pose.position.z = goals[i].z();
      control_msg.twist.linear.x = 0; 
      control_msg.twist.linear.y = 0; 
      control_msg.twist.linear.z = 0; 
      control_msg.twist.angular.x = 0;
      control_msg.twist.angular.y = 0;
      control_msg.twist.angular.z = 0;
      control_msg.reference_frame = "world";
      control_pub.publish(control_msg);
	}

	std::cout << std::endl;
}
#endif


void UNIFORM(double *p)
{
  if(x>60000)
    x=0;
  srand((unsigned)time(NULL));
	int i, a;
	double f;
	for (i = 0; i<2; i++, x = x + 689)
	{
		a = rand() + x; 
		a = a%1000;
		f = (double)a;
		f = f / 1000.0;
		*p = f;
		p++;
	}
}
double GenerateNoiseSeed()
{
  	double A, B, C, E, D, r;
	double uni[2];
	double *p;
  E = 0;
  D = 1;
  UNIFORM(&uni[0]);  //调用UNIFORM函数产生2个均匀分布的随机数并存入数组nui[2]
	 A = sqrt((-2)*log(uni[0]));
		B = 2 * M_PI*uni[1];
		C = A*cos(B);
		r = E + C*D;    //E,D分别是期望和方差
    //ROS_ERROR("%f,%f,%f,%f,%f,%f,%f",uni[0],A,B,C,D,E,r);
    return r;

}
void setRobotPositionVelocity(RVO::RVOSimulator *sim)
{
  for(size_t i = 0; i < NUM_ROBOT; i++)
  {
    sim->setAgentPosition(i,RVO::Vector3(robot_pos[i][0]+GenerateNoiseSeed(),robot_pos[i][1]+GenerateNoiseSeed(),robot_pos[i][2]+GenerateNoiseSeed()));
    sim->setAgentVelocity(i,RVO::Vector3(robot_vel[i][0]+GenerateNoiseSeed(),robot_vel[i][1]+GenerateNoiseSeed(),robot_vel[i][2]+GenerateNoiseSeed()));
  }
}
void setPreferredVelocities(RVO::RVOSimulator *sim)
{
	/* Set the preferred velocity to be a vector of unit magnitude (speed) in the direction of the goal. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		RVO::Vector3 goalVector = goals[i] - sim->getAgentPosition(i);

		if (RVO::absSq(goalVector) > 1.0f) {
			goalVector = RVO::normalize(goalVector);
		}

		sim->setAgentPrefVelocity(i, goalVector);
	}
}

// bool reachedGoal(RVO::RVOSimulator *sim)
// {
// 	/* Check if all agents have reached their goals. */
// 	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
// 		if (RVO::absSq(sim->getAgentPosition(i) - goals[i]) > 4.0f * sim->getAgentRadius(i) * sim->getAgentRadius(i)) {
// 			return false;
// 		}
// 	}

// 	return true;
// }

bool reachedGoal(void)
{
  for(int i = 0; i <NUM_ROBOT;i++)
    if((obstacle_flag[i]==0&&arrive_flag[i]==0))
      if((std::clock()-start)/(double) CLOCKS_PER_SEC < 120)
        return false;
  return true;
}
void collision_detection(void)
{
  for(int ind_a=0;ind_a<NUM_ROBOT;ind_a++)
    for(int ind_b=ind_a+1;ind_b<NUM_ROBOT;ind_b++)
    {
      if(distance(robot_pos[ind_a],robot_pos[ind_b])<0.4)
      {
        if(obstacle_flag[ind_a]==0)
          ROS_ERROR("collision%d and %d",ind_a,ind_b);
        obstacle_flag[ind_a]=1;
        obstacle_flag[ind_b]=1;
        arrive_time[ind_a] = 0;
        arrive_time[ind_b] = 0;
      }

    }
}
void arrive_check(void)
{
  for(int ind = 0; ind<NUM_ROBOT; ind++)
  {
    double goal_tmp[3] = {goals[ind].x(),goals[ind].y(),goals[ind].z()};
    if(distance(goal_tmp,robot_pos[ind])<1)
    {
      arrive_time[ind] = (std::clock() - start)/ (double) CLOCKS_PER_SEC;
      if(arrive_flag[ind]==0)
        ROS_ERROR("arrived:%d at %f",ind, arrive_time[ind]);
      arrive_flag[ind] = 1;
    }
}
}
void getModelLocation_callback(const gazebo_msgs::ModelStatesConstPtr& msg)
{
  int ind = 0;
  char robot_name[] = "Robot_";
  for(int i = 0; i<msg->pose.size(); i++)
  {
    // ROS_ERROR(msg->name[i].substr(0,6).c_str());
    // std::istringstream buffer(msg->name[i].substr(5));
    //   buffer >> ind;
    //   ROS_ERROR("%d",ind);
    if(!strcmp(msg->name[i].substr(0,6).c_str(),robot_name))
    {
      std::size_t pos = msg->name[i].find("_");
      std::istringstream buffer(msg->name[i].substr(pos+1));
      buffer >> ind;
      if(ind<NUM_ROBOT)
      {
      double current_pos[3] = {msg->pose[i].position.x,msg->pose[i].position.y,msg->pose[i].position.z};
      path_length[ind]=+distance(robot_pos[ind], current_pos);
      //ROS_ERROR("%f",path_length[ind]);
      robot_pos[ind][0] = msg->pose[i].position.x;
      robot_pos[ind][1] = msg->pose[i].position.y;
      robot_pos[ind][2] = msg->pose[i].position.z;
      }
    }
  }
  collision_detection();
  arrive_check();
}

void display_result(void)
{
  
  double collision_number = 0;
  double arrive_number = 0;
  double extra_path = 0;
  double extra_time = 0;
  for(int i = 0;i<NUM_ROBOT;i++)
  {
    double goal_tmp[3] = {goals[i].x(),goals[i].y(),goals[i].z()};
    path_optimal[i] = distance(start_location[i],goal_tmp);
    arrive_time_optimal[i] = path_optimal[i]/1;
    if(obstacle_flag[i])
      collision_number++;
    if(arrive_flag[i])
      arrive_number++;
    extra_path =+ path_length[i]/path_optimal[i];
    
    if(arrive_time[i]!=0)
      extra_time =+ arrive_time[i]/arrive_time_optimal[i];
  }
  ROS_ERROR("SUCC_RATE:%f",arrive_number/NUM_ROBOT);
  ROS_ERROR("collision_rate:%f",collision_number/NUM_ROBOT);
  ROS_ERROR("extra_time:%f",extra_time/(NUM_ROBOT-collision_number));
  ROS_ERROR("extra_path:%f",extra_path/NUM_ROBOT);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  
  
  ros::NodeHandle nh;
  gazebo_msgs::ModelState control_msg;
  ros::Rate loop_rate(10); //wait gazebo launch
  loop_rate.sleep();
	
ros::NodeHandle n;
ros::Publisher control_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);

  
  
	setupScenario(sim,control_pub);
  //sim->setTimeStep(10);
	/* Perform (and manipulate) the simulation. */
 start = std::clock(); 
 ros::Subscriber model_state = n.subscribe("/gazebo/model_states", 1, getModelLocation_callback);
	do {
#if RVO_OUTPUT_TIME_AND_POSITIONS
		updateVisualization(sim, control_pub);
#endif
  // ros::Rate loop_rate(10); //wait gazebo launch
  // loop_rate.sleep();
  ros::Duration(0.1).sleep();
  ros::spinOnce();
  //delete sim;
  //RVO::RVOSimulator *sim = new RVO::RVOSimulator();
  
  setRobotPositionVelocity(sim);
		setPreferredVelocities(sim);
		sim->doStep();
	}while (!reachedGoal());

  ROS_ERROR("Finished");
  display_result();
  return 0;
}
