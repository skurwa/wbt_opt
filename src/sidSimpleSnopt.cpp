#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>


#include <Optimizer/snopt/include/snoptProblem.hpp>

using namespace std;

// USER SPECIFY TEST TYPE
// 1 = all elements specified in G, neG = 16, A empty
// 2 = all non-zero elements in G specified, neG = 11, A empty
// 3 = all non-zero elements in G specified, neG = 11, A contains zero elements
// 4 = missing 1 non-zero element in G (3, 1), neG = 10, A empty
// 5 = missing 1 zero element in G (3, 2), neG = 11, A empty
// 6 = no elements in G specified, A properly defined
const int g_test = 1;

void userFun(int *Status, int *n, double x[], int *needF, int *lenF, double F[], int *needG,
    int *lenG, double G[], char *cu, int *lencu, int iu[], int *leniu, double ru[], int *lenru) {

    if (*needF != 0) {
        F[0] = 3 * (x[0]) + (5 * x[1]) + pow((x[0] + x[2] + x[3]), 2);
        F[1] = x[0] + pow(x[2], 2) + pow(x[3], 2);
        F[2] = (2 * x[2]) + (4 * x[3]);
        F[3] = x[1] + pow(x[3], 4);
    }
    if (*needG != 0) {
        // Ordered by rows, all 16
        if (g_test == 1) {
            G[0] = 3 + 2 * (x[0] + x[2] + x[3]);
            G[1] = 5;
            G[2] = 2 * (x[0] + x[2] + x[3]);
            G[3] = 2 * (x[0] + x[2] + x[3]);
            G[4] = 1;
            G[5] = 0;
            G[6] = 2 * x[2];
            G[7] = 2 * x[3];
            G[8] = 0;
            G[9] = 0;
            G[10] = 2;
            G[11] = 4;
            G[12] = 0;
            G[13] = 1;
            G[14] = 0;
            G[15] = 4*(pow(x[3], 3));
        }
        else if (g_test == 2 || g_test == 3) {
            G[0] = 3 + 2 * (x[0] + x[2] + x[3]);
            G[1] = 5;
            G[2] = 2 * (x[0] + x[2] + x[3]);
            G[3] = 2 * (x[0] + x[2] + x[3]);
            G[4] = 1;
            G[5] = 2 * x[2];
            G[6] = 2 * x[3];
            G[7] = 2;
            G[8] = 4;
            G[9] = 1;
            G[10] = 4*(pow(x[3], 3));
        }
        else if (g_test == 4) {
            G[0] = 3 + 2 * (x[0] + x[2] + x[3]);
            G[1] = 5;
            G[2] = 2 * (x[0] + x[2] + x[3]);
            G[3] = 2 * (x[0] + x[2] + x[3]);
            G[4] = 1;
            G[5] = 0;
            G[6] = 2 * x[2];
            G[7] = 2 * x[3];
            G[8] = 0;
            G[9] = 0;
            G[10] = 2;
            G[11] = 4;
            G[12] = 0;
            G[13] = 0;
            G[14] = 4*(pow(x[3], 3));
        }
        else if (g_test == 5) {
            G[0] = 3 + 2 * (x[0] + x[2] + x[3]);
            G[1] = 5;
            G[2] = 2 * (x[0] + x[2] + x[3]);
            G[3] = 2 * (x[0] + x[2] + x[3]);
            G[4] = 1;
            G[5] = 0;
            G[6] = 2 * x[2];
            G[7] = 2 * x[3];
            G[8] = 0;
            G[9] = 0;
            G[10] = 2;
            G[11] = 4;
            G[12] = 0;
            G[13] = 1;
            G[14] = 4*(pow(x[3], 3));
        }
        else if (g_test == 6) {
            // G is not filled
        }

        // Ordered by rows, only non-zero elements

    }
}

int main(int argc, char **argv) {
    snoptProblemA practiceProblem;

    double inf = 1.0e+20;

    int start = 0; // type of start 0 = cold, 1 = cold unless with basis file,  2 = warm
    int nF = 4; // dimension of F (includes constraint functions AND objective function)
    int n = 4; // number of variables (x1, x2, ...)

    double ObjAdd = 0.0d+0;
    int ObjRow = 0; // row index of objective function in F(x)
    string Prob = "testProb";

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

    // USER SPECIFY g_test TYPE
    // 1 = all elements in G specified, neG = 11, A empty
    // 2 = all non-zero elements in G specified, neG = 11, A empty
    // 3 = all non-zero elements in G specified, neG = 11, A contains zero elements
    // 4 = missing 1 non-zero element in G, neG = 10, A empty
    // 5 = missing 1 zero element in G, neG = 11, A empty
    // 6 = no elements in G specified, A properly defined
    int *iGfun, *jGvar, *iAfun, *jAvar;
    int neG, neA;
    double *A;

    if (g_test == 1) {
       iGfun = new int[16] {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3};
       jGvar = new int[16] {0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3};
       neG = 11;

       iAfun = new int[0];
       jAvar = new int[0];
       A  = new double[0];
       neA = 0;
    }
    if (g_test == 2) {
        iGfun = new int[11] {0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3};
        jGvar = new int[11] {0, 1, 2, 3, 0, 2, 3, 2, 3, 1, 3};
        neG = 11;

        iAfun = new int[0];
        jAvar = new int[0];
        A  = new double[0];
        neA = 0;
    }
    if (g_test == 3) {
        iGfun = new int[11] {0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3};
        jGvar = new int[11] {0, 1, 2, 3, 0, 2, 3, 2, 3, 1, 3};
        neG = 11;

        iAfun = new int[5] {1, 2, 2, 3, 3};
        jAvar = new int[5] {1, 0, 1, 0, 2};
        A  = new double[5] {0, 0, 0, 0, 0};
        neA = 0;
    }
    if (g_test == 4) {
        // missing non-zero element (3, 1)
        iGfun = new int[16] {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3};
        jGvar = new int[16] {0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 2, 3};
        neG = 10;

        iAfun = new int[0];
        jAvar = new int[0];
        A  = new double[0];
        neA = 0;
    }
    if (g_test == 5) {
        // missing zero element (3, 2)
        iGfun = new int[16] {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3};
        jGvar = new int[16] {0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 3};
        neG = 11;

        iAfun = new int[0];
        jAvar = new int[0];
        A  = new double[0];
        neA = 0;
    }
    if (g_test == 6) {
        iGfun = new int[0];
        jGvar = new int[0];

        iAfun = new int[5] {1, 0, 2, 3, 1};
        jAvar = new int[5] {0, 1, 2, 2, 3};
        A  = new double[5] {5, 1, 2, 4, 1};
        neA = 5;
    }

    // variables are unconstrained so assign infinite bounding conditions
    for (int i = 0; i <= n; i++) {
        xlow[i] = -inf;
        xupp[i] = inf;
    }

    Flow[0] = -inf;
    Fupp[0] = inf;
    Flow[1] = 2;
    Fupp[1] = 2;
    Flow[2] = 0;
    Fupp[2] = inf;
    Flow[3] = 4;
    Fupp[3] = 4;

    int nS = 0;
    int nInf;
    double sInf;

    practiceProblem.initialize("", 1);
    practiceProblem.setProbName("firstSnoptProblem");
    practiceProblem.setPrintFile("firstSnoptProblem.out");
    practiceProblem.setIntParameter("Derivative option", 1);
    practiceProblem.setIntParameter("Verify level", 3);
    practiceProblem.solve(start, nF, n, ObjAdd, ObjRow, userFun, iAfun, jAvar, A, neA, iGfun, jGvar, neG, xlow, xupp,
                        Flow, Fupp, x, xstate, xmul, F, Fstate, Fmul, nS, nInf, sInf);

    for (int i = 0; i < 4; i++) {
        cout << x[i] << endl;
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
    delete []iAfun;
    delete []jAvar;
    delete []A;
    delete []iGfun;
    delete []jGvar;


}
