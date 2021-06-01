#include "controlmain.h"

void ControlMain::path_generator(double x0, double xf, double tf, double ta, double h, std::vector<double> *path, int path_index)
{
    if(path_index == 1){
        double x_step = (xf - x0)/(tf/h);
        double x = x0;
        for(double t = 0; t < tf; t += h){
            path->push_back(x);
            x += x_step;
        }
    }
    else{
        double td = tf - ta;
        double vd = (xf - x0)/td;
        double xa = x0 + 0.5*ta*vd;
        double xd = xf - 0.5*ta*vd;

        double pos0, posf, vel0, velf, acc0, accf, ts;
        double a0, a1, a2, a3, a4, a5;

        // section of acceleration
        pos0 = x0; posf = xa; vel0 = 0; velf = vd; acc0 = 0; accf = 0; ts = ta;
        a0 = pos0;
        a1 = vel0;
        a2 = acc0/2;
        a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
        a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
        a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

        for(double t = 0; t < ts; t += h){
            path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
        }

        // section of constant velocity
        pos0 = xa; posf = xd; vel0 = vd; velf = vd; acc0 = 0; accf = 0; ts = td - ta;
        a0 = pos0;
        a1 = vel0;
        a2 = acc0/2;
        a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
        a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
        a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

        for(double t = 0; t < ts; t += h){
            path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
        }

        // section of deceleration
        pos0 = xd; posf = xf; vel0 = vd; velf = 0; acc0 = 0; accf = 0; ts = tf - td;
        a0 = pos0;
        a1 = vel0;
        a2 = acc0/2;
        a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
        a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
        a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

        for(double t = 0; t < ts; t += h){
            path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
        }
    }
}
