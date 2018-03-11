#include "PID.h"
#include <vector>
#include <iostream>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

// TODO take into account time?

void PID::Init(double Kp, double Ki, double Kd) {    
    P = vector<double>(3);
    P[0] = Kp;
    P[1] = Ki;
    P[2] = Kd;
    
    dP = vector<double>(3);
    dP[0] = 1;
    dP[1] = 1;
    dP[2] = 1;


    // Getting the max double to bootstrap our best error
	numeric_limits<double> nl;
    best_error = nl.max();

    ctes = vector<double>(0);
    previous_cte = 0.0;
    current_cte = 0.0;    
    current_P_index = 0;
    phase = 0;
    count = 0;
    min_cycles = 5;
    
}

// double CheckError(){
//     double total_err = TotalError();
//     if(total_err < best_err){
//         if(phase)
        
//         dP[current_P_index] *= 1.1
//     }
// }

void PID::UpdateError(double cte) {    
    cout << "UPDATE ERROR!! (" << count << ")" << endl;
    double P_sum = P[0] + P[1] + P[2];
    if(count >= min_cycles){
        cout << "***************TWIDDLE (P_SUM = " << P_sum << ")" << endl;        
        if(P_sum >= 0.0001){
            Twiddle();
        }
        
    }
    
    previous_cte = current_cte;
    current_cte = cte;
    ctes.push_back(current_cte);
    ++count;

    current_steer = CalculateSteer(P[0], P[1], P[2]);
}

double PID::TotalError() {
    double sum_cte = 0.0;
    for(auto cte: ctes){
        sum_cte += cte * cte;
    }

    return sum_cte / ctes.size();
}

void PID::Twiddle(){
    if(count == min_cycles){
        best_error = TotalError();
        ctes.clear();
    }
    if(ctes.size() == 0){
        cout << "TWIDDLE CTE RESET" << endl;
        if(phase == 0){
            P[current_P_index] += dP[current_P_index];
        }else if(phase == 1){
            P[current_P_index] -= 2 * dP[current_P_index];
        }
    }
    
    if(ctes.size() == min_cycles){        
        double err = TotalError();
        cout << "*** TWIDDLE COMPUTED ERROR: " << err << endl;
        if(err < best_error){
            best_error = err;
            dP[current_P_index] *= 1.1;
            // Move on to the next component of PID
            current_P_index = (current_P_index + 1) % P.size();
        }else{
            if(phase == 0){
                phase = 1;
            }else{
                P[current_P_index] += dP[current_P_index];
                dP[current_P_index] *= 0.9;
                // Move on to the next component of PID
                current_P_index = (current_P_index + 1) % P.size();
            }
        }
        // make sure to clear (and thus resize) our cte vector
        ctes.clear();
    }
}


    /**
     * Start with a value k for a few turns and measure the average error
     * Then try k + dk and if the error is less then dk * 1.1
     * Otherwise try with k - 2 * dk and if the error is less dk * 1.1
     * Otherwise k + dk and set dk * 0.9
     * 
     * 
     * 
     * 
     */ 

    // double p_comp = Kp * current_cte;
    // double d_comp = 0.0;
    // if(count > 0){
    //     d_comp = Kd * (current_cte - previous_cte);
    // }

    // double i_comp = Ki * sum_cte;


    // double steer = -p_comp - d_comp - i_comp;

    // if(steer < -1){
    //     return -1;
    // }
    // if(steer > 1){
    //     return 1;        
    // }
    // return steer;
    


double PID::CalculateSteer(double kp, double ki, double kd){
    double p_comp = kp * current_cte;
    double d_comp = 0.0;
    if(count > 0){
        d_comp = kd * (current_cte - previous_cte);
    }

    double sum_cte = 0.0;
    for(auto cte: ctes){
        sum_cte += cte;
    }

    double i_comp = ki * sum_cte;

    double steer = -p_comp - d_comp - i_comp;

    if(steer < -1){
        return -1;
    }
    if(steer > 1){
        return 1;        
    }
    return steer;    
}

