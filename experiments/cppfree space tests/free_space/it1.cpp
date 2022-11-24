#include "ibex.h"
#include <ctime>

using namespace ibex;
using namespace std;


float dt = 0.01;
Interval speed(0,1), omega(0,1);
IntervalVector Intnoise(2,Interval(-0.5,0.5));
vector<IntervalVector> Pose, out;

// First try with intervals. The issue is that it scaled a lot in terms of individual boxes to check

void PropagateForward()
{
    Variable p(2);

    Function pose(p,Return(speed*cos(p[2]),speed*sin(p[2])));

    NumConstraint pos(p,p-(Pose[0]+dt*pose(Pose[0]))=Intnoise);
    CtcFwdBwd ctcPosef(pos);
    CtcCompo ComPosef(ctcPosef);
    CtcFixPoint fix_fwd(ComPosef);
    fix_fwd.contract(Pose[1]);
}

int run1()
{
 // Discarded due to poor scaling
    std::clock_t c_start = std::clock();

    IntervalVector InitBox(2);
    InitBox[0]=Interval(-0.5,0.5);
    InitBox[1]=Interval(-0.5,0.5);

    IntervalVector world(2);
    world[0]=Interval(-20,20);
    world[1]=Interval(-20,20);

    Pose.push_back(InitBox);
    Pose.push_back(world);

//    PropagateForward();

    vector<IntervalVector> boxes;

    IntervalVector meas1(2);
    meas1[0]=Interval(0,1);
    meas1[1]=Interval(2,5);

    boxes.push_back(meas1);

    IntervalVector meas2(2);
    meas2[0]=Interval(2,3);
    meas2[1]=Interval(2,5);

    boxes.push_back(meas2);

    IntervalVector meas3(2);
    meas3[0]=Interval(5,6);
    meas3[1]=Interval(2,5);

    boxes.push_back(meas3);

    IntervalVector meas4(2);
    meas4[0]=Interval(7,8);
    meas4[1]=Interval(2,5);

    boxes.push_back(meas4);

    IntervalVector* result; // to store the result
    for (auto meas: boxes){

        int n=Pose[1].diff(meas,result);

        for (int i=0; i<n; i++) {
            out.push_back(result[i]);
        }

    }

    IntervalVector final_box = out[0];
    IntervalVector temp = out[0];
    cout << "initial_box" << endl;
    cout << out[0] << endl;

    for (int i=1; i<out.size(); i++){

        temp &= out[i];
        if (temp.is_empty()){
            temp = final_box;
        }
        else{
            final_box = temp;
            cout << "out" << endl;
            cout << out[i] << endl;
            cout << "final_box" << endl;
            cout << final_box << endl;
            cout << "--------------------------------------------" << endl;
        }
    }

    cout << final_box << endl;

    std::clock_t c_end = std::clock();
    delete[] result; // don't forget to free memory!


    double time_elapsed_ms = 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;
    std::cout << "CPU time used: " << time_elapsed_ms << " ms\n";


}
