#ifndef WBT_TIME_INTEGRATION_CONSTRAINT_H
#define WBT_TIME_INTEGRATION_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>
#include <string>
#include <iostream>
#include "RobotModel.hpp"

class Time_Integration_Constraint{
public:
	Time_Integration_Constraint();
	~Time_Integration_Constraint();
	RobotModel* robot_model_;	

	double dt; 
	int total_timesteps; 

	sejong::Vector q_vec;
	sejong::Vector qdot_vec;
};
#endif