#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <Optimizer/snopt/include/snoptProblem.hpp>
#include "RobotModel.hpp"
#include "valkyrie_definition.h"

using namespace std;

sejong::Vector g_q(NUM_Q);
sejong::Vector g_q_nom(NUM_Q);
RobotModel* robot_model = RobotModel::GetRobotModel();

void userFun(int *Status, int *n, double x[], int *needF, int *lenF, double F[], int *needG, int *lenG, double G[], char *cu, int *lencu, int iu[], int *leniu, double ru[], int *lenru) {
    if (*needF != 0) {
        F[0] = ((g_q - g_q_nom).transpose() * (g_q - g_q_nom)); // objective function, UNWEIGHTED
        double *ptr_x = &x[0];
        sejong::Vector sj_x;
        sejong::convert(ptr_x, NUM_Q, sj_x);
        sejong::Vect3 link_pos;
        for (int i = 0; i < NUM_LINK; i++) {
            robot_model->getPosition(sj_x, i, link_pos);
            for (int j = 0; j < 3; j++) {
                F[(3*i) + j + 1] = link_pos[j];
            }
        }
    }
}

int main(int argc, char** argv) {

    g_q.setZero();
    g_q_nom.setZero();

    // Set initial virtual Joints
    g_q_nom[0] = 0.0; // x_pos
    g_q_nom[1] = 0.0; // y_pos
    g_q_nom[2] = 1.14; // z_pos
    g_q_nom[NUM_Q - 1] = 1.0; // quaternion w

    // Set other joints to initial configuration
    g_q_nom[NUM_VIRTUAL + SJJointID::leftHipPitch] = -0.3; //r_joint_[r_joint_idx_map_.find("leftHipPitch"  )->second]->m_State.m_rValue[0] = -0.3;
    g_q_nom[NUM_VIRTUAL + SJJointID::rightHipPitch] = -0.3;  //r_joint_[r_joint_idx_map_.find("rightHipPitch" )->second]->m_State.m_rValue[0] = -0.3;
    g_q_nom[NUM_VIRTUAL + SJJointID::leftKneePitch] = 0.6;  //r_joint_[r_joint_idx_map_.find("leftKneePitch" )->second]->m_State.m_rValue[0] = 0.6;
    g_q_nom[NUM_VIRTUAL + SJJointID::rightKneePitch] = 0.6;//r_joint_[r_joint_idx_map_.find("rightKneePitch")->second]->m_State.m_rValue[0] = 0.6;
    g_q_nom[NUM_VIRTUAL + SJJointID::leftAnklePitch] = -0.3; //r_joint_[r_joint_idx_map_.find("leftAnklePitch")->second]->m_State.m_rValue[0] = -0.3;
    g_q_nom[NUM_VIRTUAL + SJJointID::rightAnklePitch] = -0.3; //r_joint_[r_joint_idx_map_.find("rightAnklePitch")->second]->m_State.m_rValue[0] = -0.3;

    g_q_nom[NUM_VIRTUAL + SJJointID::rightShoulderPitch] = 0.2; //r_joint_[r_joint_idx_map_.find("rightShoulderPitch")->second]->m_State.m_rValue[0] = 0.2;
    g_q_nom[NUM_VIRTUAL + SJJointID::rightShoulderRoll] = 1.1;  //r_joint_[r_joint_idx_map_.find("rightShoulderRoll" )->second]->m_State.m_rValue[0] = 1.1;
    g_q_nom[NUM_VIRTUAL + SJJointID::rightElbowPitch] = 0.4;  //r_joint_[r_joint_idx_map_.find("rightElbowPitch"   )->second]->m_State.m_rValue[0] = 0.4;
    g_q_nom[NUM_VIRTUAL + SJJointID::rightForearmYaw] = 1.5;  //r_joint_[r_joint_idx_map_.find("rightForearmYaw" )->second]->m_State.m_rValue[0] = 1.5;

    g_q_nom[NUM_VIRTUAL + SJJointID::leftShoulderPitch] = -0.2; //r_joint_[r_joint_idx_map_.find("rightShoulderPitch")->second]->m_State.m_rValue[0] = 0.2;
    g_q_nom[NUM_VIRTUAL + SJJointID::leftShoulderRoll] = -1.1;  //r_joint_[r_joint_idx_map_.find("rightShoulderRoll" )->second]->m_State.m_rValue[0] = 1.1;
    g_q_nom[NUM_VIRTUAL + SJJointID::leftElbowPitch] = -0.4;//0.4;  //r_joint_[r_joint_idx_map_.find("rightElbowPitch"   )->second]->m_State.m_rValue[0] = 0.4;
    g_q_nom[NUM_VIRTUAL + SJJointID::leftForearmYaw] = 1.5;  //r_joint_[r_joint_idx_map_.find("rightForearmYaw" )->second]->m_State.m_rValue[0] = 1.5;

    int ctr = 0;
    // // iterates through enum to determine position for each link if provided q
    // for (int link_ctr = LK_pelvis; link_ctr != LK_rightFootInFront+1; link_ctr++) {
    //     robot_model->getPosition(q_nom, link_ctr, link_pos);
    //     cout << link_pos << endl;
    //     ctr++;
    // }

    //////////////////////////////////////////////////////////////////////////////////////////////////

    // SNOPT problem construction
    snoptProblemA singleShotIK;

    double inf = 1.0e+20;

    int start = 0; // cold start
    int nF = NUM_LINK * 3; // dimension of F
    int n = NUM_Q; // number of variables; joints
    int ObjRow = 0; // row index of objective function in F(x)
    int nS = 0;
    int nInf;
    double sInf;
    double ObjAdd = 0.0d+0;
    string Prob = "Valkyrie Single-Shot IK";

    double *x      = new double[n];
    double *xlow   = new double[n];
    double *xupp   = new double[n];
    double *xmul   = new double[n];
    int    *xstate = new    int[n];

    double *F      = new double[nF];
    double *Flow   = new double[nF];
    double *Fupp   = new double[nF];
    double *Fmul   = new double[nF];
    int    *Fstate = new    int[nF];

    // define iGfun, jGvar, A, neA, neG - is this even possible? (Not sure if we have G in an analytical form)

    // define xupp and xlow bounds - need to figure out how to do this in an object-oriented way
    for (int i = 0; i < NUM_Q; i++) {
        xlow[i] = -2*M_PI;
        xupp[i] = 2*M_PI;
    }

    // define Fupp and Flow bounds - need to figure out how to do this in an object-oriented way
    double epsilon = .01;
    for (int i = 0; i < NUM_LINK; i++) {
        for (int j = 0; j < 3; j++) {
            Fupp[(3*i) + j + 1] = 0 + epsilon;
            Flow[(3*i) + j + 1] = 0 - epsilon;
        }
    }

    // assuming G cannot be defined, using reduced parameter SNOPT solver

    singleShotIK.initialize("", 1);
    singleShotIK.setProbName("Valkyrie Single-Shot IK");
    singleShotIK.setPrintFile("Valkyrie Single-Shot IK.out");
    singleShotIK.setIntParameter("Verify level", 3);
    // give solver minimum inputs
    singleShotIK.setIntParameter("Derivative option", 0); // no G provided = 0, some G provided = 1
    singleShotIK.solve(start, nF, n, ObjAdd, ObjRow, userFun, xlow, xupp, Flow, Fupp, x, xstate, xmul, F, Fstate, Fmul, nS, nInf, sInf);

    for (int i = 0; i < NUM_Q; i++) {
        cout << i << ": " << x[i] << endl;
    }
    
    delete []x;
    delete []xlow;
    delete []xupp;
    delete []xmul;
    delete []xstate;
    delete []F;
    delete []Flow;
    delete []Fupp;
    delete []Fstate;
    delete []Fmul;








}
