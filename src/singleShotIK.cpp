#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <Optimizer/snopt/include/snoptProblem.hpp>
#include "RobotModel.hpp"
#include "valkyrie_definition.h"

using namespace std;

// void userFun(int *Status, int *n, double x[], int *needF, int *lenF, double F[], int *needG, int *lenG, double G[], char *cu, int *lencu, int iu[], int *leniu, double ru[], int *lenru) {
//
// }


// pseudocode
// make q vector with all initial joint values
//



int main(int argc, char** argv) {
    RobotModel* robot_model;
    robot_model = RobotModel::GetRobotModel();

    sejong::Vector q_nom(NUM_Q);
    q_nom.setZero();

    // Set initial virtual Joints
    q_nom[0] = 0.0; // x_pos
    q_nom[1] = 0.0; // y_pos
    q_nom[2] = 1.14; // z_pos
    q_nom[NUM_Q - 1] = 1.0; // quaternion w

    // Set other joints to initial configuration
    q_nom[NUM_VIRTUAL + SJJointID::leftHipPitch] = -0.3; //r_joint_[r_joint_idx_map_.find("leftHipPitch"  )->second]->m_State.m_rValue[0] = -0.3;
    q_nom[NUM_VIRTUAL + SJJointID::rightHipPitch] = -0.3;  //r_joint_[r_joint_idx_map_.find("rightHipPitch" )->second]->m_State.m_rValue[0] = -0.3;
    q_nom[NUM_VIRTUAL + SJJointID::leftKneePitch] = 0.6;  //r_joint_[r_joint_idx_map_.find("leftKneePitch" )->second]->m_State.m_rValue[0] = 0.6;
    q_nom[NUM_VIRTUAL + SJJointID::rightKneePitch] = 0.6;//r_joint_[r_joint_idx_map_.find("rightKneePitch")->second]->m_State.m_rValue[0] = 0.6;
    q_nom[NUM_VIRTUAL + SJJointID::leftAnklePitch] = -0.3; //r_joint_[r_joint_idx_map_.find("leftAnklePitch")->second]->m_State.m_rValue[0] = -0.3;
    q_nom[NUM_VIRTUAL + SJJointID::rightAnklePitch] = -0.3; //r_joint_[r_joint_idx_map_.find("rightAnklePitch")->second]->m_State.m_rValue[0] = -0.3;

    q_nom[NUM_VIRTUAL + SJJointID::rightShoulderPitch] = 0.2; //r_joint_[r_joint_idx_map_.find("rightShoulderPitch")->second]->m_State.m_rValue[0] = 0.2;
    q_nom[NUM_VIRTUAL + SJJointID::rightShoulderRoll] = 1.1;  //r_joint_[r_joint_idx_map_.find("rightShoulderRoll" )->second]->m_State.m_rValue[0] = 1.1;
    q_nom[NUM_VIRTUAL + SJJointID::rightElbowPitch] = 0.4;  //r_joint_[r_joint_idx_map_.find("rightElbowPitch"   )->second]->m_State.m_rValue[0] = 0.4;
    q_nom[NUM_VIRTUAL + SJJointID::rightForearmYaw] = 1.5;  //r_joint_[r_joint_idx_map_.find("rightForearmYaw" )->second]->m_State.m_rValue[0] = 1.5;

    q_nom[NUM_VIRTUAL + SJJointID::leftShoulderPitch] = -0.2; //r_joint_[r_joint_idx_map_.find("rightShoulderPitch")->second]->m_State.m_rValue[0] = 0.2;
    q_nom[NUM_VIRTUAL + SJJointID::leftShoulderRoll] = -1.1;  //r_joint_[r_joint_idx_map_.find("rightShoulderRoll" )->second]->m_State.m_rValue[0] = 1.1;
    q_nom[NUM_VIRTUAL + SJJointID::leftElbowPitch] = -0.4;//0.4;  //r_joint_[r_joint_idx_map_.find("rightElbowPitch"   )->second]->m_State.m_rValue[0] = 0.4;
    q_nom[NUM_VIRTUAL + SJJointID::leftForearmYaw] = 1.5;  //r_joint_[r_joint_idx_map_.find("rightForearmYaw" )->second]->m_State.m_rValue[0] = 1.5;

    sejong::Vect3 link_pos;
    int ctr = 0;
    // // iterates through enum to determine position for each link if provided q
    // for (int link_ctr = LK_pelvis; link_ctr != LK_rightFootInFront+1; link_ctr++) {
    //     robot_model->getPosition(q_nom, link_ctr, link_pos);
    //     cout << link_pos << endl;
    //     ctr++;
    // }



}
