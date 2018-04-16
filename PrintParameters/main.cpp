#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <sstream>
#include <vector>
#include <algorithm>
#include <map>
#include <iterator>
#include <cmath>
#include <numeric>

class Vehicle {
public:
    int v_id{};
    float v_length{}, v_width{};
    std::vector <long> global_time;
    std::vector <int> preceding, lane;
    std::vector <float> local_x, local_y, speed, TTC, H, PSD;
    std::vector <double> ACI;
};
//Conditions for ACI
double normal_cumulative(float mean, float stddev, double raw_x);
double MADR(double raw_x);
double reaction_time(double raw_x);
double condition_1_A(float v1, float d1);
double condition_1_B(float v1, float d1);
double condition_2_1A(float v1, float v2, float d1, float d1_2);
double condition_2_1B(float v1, float v2, float d1, float d1_2);
double condition_2_2A(float v1, float v2, float d1, float d1_2);
double condition_2_2B(float v1, float v2, float d1, float d1_2);
double condition_3_211(float v1, float v2, float d1, float d1_2);
double condition_3_221(float v1, float v2, float d1, float d1_2);
double condition_4_211(float v1, float v2, float d1, float d1_2, float R);
double condition_4_221(float v1, float v2, float d1, float d1_2, float R);
double AggregatedCrashIndex(float v1, float v2, float d1, float d1_2, float R);


int main() {
    //Thresholds
    float H_threshold = 0.5; //H
    float TTC_threshold = 1.5; //TTC
    float d_max = 3.75; //d_max for PSD
    float d1 = 1.5; //Hypothetical deceleration ACI

    //Open CSV files
    std::ifstream indata;
    std::ofstream outdata;
    outdata.open("../../CSV/FromPrintParameters/Conflicts-us-101-3-3.csv", std::ios::app);
    indata.open("../../CSV/FromGettingCSV/us-101-3-3.csv");
    outdata << "Conflict,Conflict_Value,V_ID,Global_Time,Lane" << std::endl;
    std::string row;


    //Objects data
    std::vector <long> global_time_v;
    std::vector <float> local_x_v;
    std::vector <float> local_y_v;
    std::vector <float> speed_v;
    std::vector <int> preceding_v;
    std::vector <float> H_v;
    std::vector <int> lane_v;

    //Map of vehicles
    std::map <int, Vehicle> v;
    Vehicle temp;

    //CREATE A MAP OF VEHICLES
    int old_v_id = 0;
    while(indata) {
        indata >> row;
        std::istringstream ss(row);
        std::string token;
        std::vector <std::string> k;
        //Extract row
        while (getline(ss, token, ',')) {
            k.push_back(token);
        }
        //Select variables from the row and convert to SI
        int v_id ; std::stringstream s1(k[0]);
        s1 >> v_id;
        float v_length ; std::stringstream s2(k[8]);
        s2 >> v_length; v_length = static_cast<float>(v_length * 0.3048);
        float v_width ; std::stringstream s3(k[9]);
        s3 >> v_width; v_width = static_cast<float>(v_width * 0.3048);
        long global_time ; std::stringstream s4(k[3]);
        s4 >> global_time;
        float local_x ; std::stringstream s5(k[4]);
        s5 >> local_x; local_x = static_cast<float>(local_x * 0.3048);
        float local_y ; std::stringstream s6(k[5]);
        s6 >> local_y; local_y = static_cast<float>(local_y * 0.3048);
        float speed ; std::stringstream s7(k[11]);
        s7 >> speed; speed = static_cast<float>(speed * 0.3048);
        int preceding ; std::stringstream s8(k[20]);
        s8 >> preceding;
        float H ; std::stringstream s9(k[23]);
        s9 >> H;
        int lane ; std::stringstream s10(k[13]);
        s10 >> lane;

        //Update vectors of variables
        if(v_id==old_v_id){
            global_time_v.push_back(global_time);
            local_x_v.push_back(local_x);
            local_y_v.push_back(local_y);
            speed_v.push_back(speed);
            preceding_v.push_back(preceding);
            H_v.push_back(H);
            lane_v.push_back(lane);
        }
            //Create a vehicle object, delete vectors, and initialise next vehicle
        else{
            temp.global_time = global_time_v;
            temp.local_x = local_x_v;
            temp.local_y = local_y_v;
            temp.speed = speed_v;
            temp.preceding = preceding_v;
            temp.H = H_v;
            temp.lane = lane_v;

            v.insert(std::pair <int, Vehicle> (temp.v_id, temp));
            global_time_v.clear();
            local_x_v.clear();
            local_y_v.clear();
            speed_v.clear();
            preceding_v.clear();
            lane_v.clear();
            H_v.clear();

            temp.v_id = v_id;
            temp.v_length = v_length;
            temp.v_width = v_width;
            old_v_id = v_id;
        }
    }
    //NOW, THE MAP OF VEHICLES IS DONE

    //Calculate Parameters
    std::vector <float> TTC_v, PSD_v, dist_y_v, dist_x_v;
    std::vector <double> ACI_v;
    float TTC, PSD, dist_y, dist_x;
    double ACI;
    bool actual;
    std::map <int, Vehicle> :: iterator itr;
    for(itr = v.begin(); itr != v.end(); ++itr) {
        //Longitudinal and lateral distance
        for (int j = 0; j < itr->second.global_time.size(); j++) {
            long t = itr->second.global_time[j];
            float speed_f = itr->second.speed[j];
            float local_y_f = itr->second.local_y[j];
            float local_x_f = itr->second.local_x[j];
            float width_f = itr->second.v_width;
            //FIND Preceding car
            auto leader = v.find(itr->second.preceding[j]);
            Vehicle lead = leader->second;
            int l_id = leader->first;
            //FIND Global time
            long m = distance(lead.global_time.begin(), find(lead.global_time.begin(), lead.global_time.end(), t));
            float speed_l = lead.speed[m];
            float local_y_l = lead.local_y[m];
            float local_x_l = lead.local_x[m];
            float length_l = lead.v_length;
            float width_l = lead.v_width;
            //Longitudinal
            dist_y = local_y_l - length_l - local_y_f;
            //Lateral
            dist_x = 99;
            ACI = 0;
            //TAKE INTO ACCOUNT THAT "FIND" CAN FAIL
            if (l_id == itr->second.preceding[j] && t == lead.global_time[m]) {
                if (local_x_f > local_x_l) {
                    dist_x = local_x_f - local_x_l - (width_f + width_l) / 2;
                } else {
                    dist_x = local_x_l - local_x_f - (width_f + width_l) / 2;
                }
            }
            //Is the car ACTUALLY behind another vehicle?
            actual = dist_x < 0;
            //Calculate TTC and PSD
            if (speed_f > speed_l && actual) {
                TTC = (dist_y / (speed_f - speed_l));
                PSD = 2 * TTC * d_max / speed_f;
            } else {
                TTC = 999; PSD = 999;
            }
            if (!actual){
                itr->second.H[j] = 99;
            }
            else {
                float R = 0.92;
                ACI = AggregatedCrashIndex(speed_l, speed_f, d1, dist_y, R);
            }

            TTC_v.push_back(TTC); PSD_v.push_back(PSD); ACI_v.push_back(ACI);
        }

        //Insert Conflicts in vehicle object
        itr->second.TTC = TTC_v; itr->second.PSD = PSD_v; itr->second.ACI = ACI_v;

        TTC_v.clear(); PSD_v.clear(); ACI_v.clear();
    }

    for(itr = v.begin(); itr != v.end(); ++itr) {
        //Select TTC under Threshold
        float TTC_min = TTC_threshold;
        std::vector <int> indicator_TTC;
        int j = 0; bool print = false;
        for(int i = 0; i<itr->second.TTC.size(); i++){
            if(itr->second.TTC[i] < TTC_threshold){
                if(itr->second.TTC[i] < TTC_min && itr->second.TTC[i] > 0){
                    TTC_min = itr->second.TTC[i];
                    j = i; print = true; }}
            else{
                if(print){ indicator_TTC.push_back(j), print = false; }
                TTC_min = TTC_threshold; }
        }
        //Select PSD under Threshold
        float PSD_min = 1; print = false;
        std::vector <int> indicator_PSD;
        for(int i = 0; i<itr->second.PSD.size(); i++){
            if(itr->second.PSD[i] < 1){
                if(itr->second.PSD[i] < PSD_min && itr->second.PSD[i] > 0){
                    PSD_min = itr->second.PSD[i];
                    j = i; print = true; }}
            else{
                if(print){ indicator_PSD.push_back(j), print = false; }
                PSD_min = 1; }
        }

        //Select H under Threshold
        float H_min = H_threshold; print = false;
        std::vector <int> indicator_H;
        for(int i = 0; i<itr->second.H.size(); i++){
            if(itr->second.H[i] < H_threshold){
                if(itr->second.H[i] < H_min && itr->second.H[i] > 0){
                    H_min = itr->second.H[i];
                    j = i; print = true; }}
            else{
                if(print){ indicator_H.push_back(j), print = false; }
                H_min = H_threshold; }
        }
        //Select ALL ACI bigger than 0.0001
        std::vector <int> indicator_ACI;
        for(int i = 0; i<itr->second.ACI.size(); i++){
            if(itr->second.ACI[i] > 0.0001){
                indicator_ACI.push_back(i); }}

        //Print TTC
        for (int i : indicator_TTC) {
            std::string string; string.append("TTC,");
            //Convert TTC into Safety Surrogate
            float TTCSurrogate = TTC_threshold - itr->second.TTC[i];
            string.append(std::to_string(TTCSurrogate)); string.append(",");
            string.append(std::to_string(itr->first)); string.append(",");
            string.append(std::to_string(itr->second.global_time[i])); string.append(",");
            string.append(std::to_string(itr->second.lane[i]));
            outdata << string << std::endl;
        }
        //Print PSD
        for (int i : indicator_PSD) {
            std::string string; string.append("PSD,");
            //Convert PSD into Safety Surrogate
            float PSDSurrogate = 1 - itr->second.PSD[i];
            string.append(std::to_string(PSDSurrogate)); string.append(",");
            string.append(std::to_string(itr->first)); string.append(",");
            string.append(std::to_string(itr->second.global_time[i])); string.append(",");
            string.append(std::to_string(itr->second.lane[i]));
            outdata << string << std::endl;
        }
        //Print H
        for (int i : indicator_H) {
            std::string string; string.append("H,");
            //Convert PSD into Safety Surrogate
            float HSurrogate = H_threshold - itr->second.H[i];
            string.append(std::to_string(HSurrogate)); string.append(",");
            string.append(std::to_string(itr->first)); string.append(",");
            string.append(std::to_string(itr->second.global_time[i])); string.append(",");
            string.append(std::to_string(itr->second.lane[i]));
            outdata << string << std::endl;
        }
        //Print ACI
        for (int i : indicator_ACI) {
            std::string string; string.append("ACI,");
            string.append(std::to_string(itr->second.ACI[i])); string.append(",");
            string.append(std::to_string(itr->first)); string.append(",");
            string.append(std::to_string(itr->second.global_time[i])); string.append(",");
            string.append(std::to_string(itr->second.lane[i]));
            outdata << string << std::endl;
        }
    }

    return 0;

}

//Calculate ACI probabilities
double normal_cumulative(float mean, float stddev, double raw_x) {
    // constants
    double a1 =  0.254829592, a2 = -0.284496736, a3 =  1.421413741;
    double a4 = -1.453152027, a5 =  1.061405429, p  =  0.3275911;

    // Normalize distribution
    double x = (raw_x - mean) / stddev;

    // Save the sign of x
    int sign = 1;
    if (x < 0) { sign = -1; }
    x = fabs(x)/sqrt(2.0);

    // A&S formula 7.1.26
    double t = 1.0/(1.0 + p*x);
    double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

    return 0.5*(1.0 + sign*y);
}

double MADR(double raw_x){
    float mean = 8.45;
    float stddev = 1.4;
    return normal_cumulative(mean, stddev, raw_x);
}

double reaction_time(double raw_x){
    float mean = 0.92;
    float stddev = 0.28;
    return normal_cumulative(mean, stddev, raw_x);
}

//CONDITIONS 1, 2, 3, AND 4
double condition_1_A(float v1, float d1){
    double T1 = v1 / d1;
    /*double a =  1 - reaction_time(T1);*/
    return 1 - reaction_time(T1);
}

double condition_1_B(float v1, float d1){
    double T1 = v1 / d1;
    /*double a =  reaction_time(T1);*/
    return reaction_time(T1);
}

double condition_2_1A(float v1, float v2, float d1, float d1_2){
    double TA = (d1_2 + pow(v1, 2) / (2 * d1)) / v2;
    /*double a =  1 - reaction_time(TA);*/
    return 1 - reaction_time(TA);
}

double condition_2_1B(float v1, float v2, float d1, float d1_2){
    double TB = pow(pow(v1 - v2, 2) + 2 * d1 * d1_2, 0.5) - (v2 - v1) / (2 * d1);
    /*double a =  1 - reaction_time(TB);*/
    return 1 - reaction_time(TB);
}

double condition_2_2A(float v1, float v2, float d1, float d1_2){
    double TA = (d1_2 + pow(v1, 2) / (2 * d1)) / v2;
    /*double a =  reaction_time(TA);*/
    return reaction_time(TA);
}

double condition_2_2B(float v1, float v2, float d1, float d1_2){
    double TB = pow(pow(v1 - v2, 2) + 2 * d1 * d1_2, 0.5) - (v2 - v1) / (2 * d1);
    /*double a =  reaction_time(TB);*/
    return reaction_time(TB);
}

double condition_3_211(float v1, float v2, float d1, float d1_2){
    double T1 = v1 / d1;
    double C3 = (T1 * v2 - T1 * v1 - 2 * d1_2) / (3 * v2 + 3 * v1 - d1 * T1);
    /*double a =  1 - reaction_time(C3);*/
    return 1 - reaction_time(C3);
}

double condition_3_221(float v1, float v2, float d1, float d1_2){
    double T1 = v1 / d1;
    double C3 = (T1 * v2 - T1 * v1 - 2 * d1_2) / (3 * v2 + 3 * v1 - d1 * T1);
    /*double a =  reaction_time(C3);*/
    return reaction_time(C3);
}

double condition_4_211(float v1, float v2, float d1, float d1_2, float R){
    double BRAD1 = pow(v2, 2) / ( 2 * (pow(v1, 2) / (2 * d1) + d1_2 - v2 * R));
    /*double a =  1 - MADR(BRAD1);*/
    return MADR(BRAD1);
}

double condition_4_221(float v1, float v2, float d1, float d1_2, float R){
    double BRAD2 = d1 + (pow(v2 - v1 + d1 * R, 2) / (2 * d1_2 + 2 * R * (v2 - v1 - d1 * R / 2)));
    /*double a =  1 - MADR(BRAD2);*/
    return MADR(BRAD2);
}

double AggregatedCrashIndex(float v1, float v2, float d1, float d1_2, float R){
    double A1 = condition_1_A(v1, d1) * condition_2_1A(v1, v2, d1, d1_2);
    double B1 = condition_1_B(v1, d1) * condition_2_1B(v1, v2, d1, d1_2);
    double A211 = condition_1_A(v1, d1) * condition_2_2A(v1, v2, d1, d1_2) * condition_3_211(v1, v2, d1, d1_2) * condition_4_211(v1, v2, d1, d1_2, R);
    double B211 = condition_1_B(v1, d1) * condition_2_2B(v1, v2, d1, d1_2) * condition_3_211(v1, v2, d1, d1_2) * condition_4_211(v1, v2, d1, d1_2, R);
    double B221 = condition_1_B(v1, d1) * condition_2_2B(v1, v2, d1, d1_2) * condition_3_221(v1, v2, d1, d1_2) * condition_4_221(v1, v2, d1, d1_2, R);
    double aggregated = A1 + B1 + A211 + B211 + B221;
    return aggregated;
}
