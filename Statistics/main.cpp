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

class SampleTime {
public:
    float averageSpeed{}, density{}, trafficFlow{};
    std::vector <int> vehicles;
    std::vector <float> speed, TTC, H, PSD;
    std::vector <double> ACI;
};

class Statistics {
public:
    float averageSpeed{}, density{}, trafficFlow{};
    float TTC{}, H{}, PSD{};
    double ACI{};
};

class Lane {
public:
    int laneID{};
    std::map <long, SampleTime> st;
    std::map <int, Statistics> stats;
};


int main() {

    //Open CSV files
    std::ifstream indata;
    std::ofstream outdata;
    outdata.open("../../CSV/FromStatistics/Statistics-us-101-3-3.csv", std::ios::app);
    indata.open("../../CSV/FromGettingCSV/us-101-3-3.csv");
    outdata << "Lane,Period,Av_Speed,Density,Traffic_Flow,TTCs,PSDs,Hs,ACIs" << std::endl;
    std::string row;

    //Skip the title
    indata >> row;

    //NUMBER OF LANES, TRACK LENGTH, AND START AND FINISH TIME

    //Initialise them first
    int numberOfLanes = 0;
    float startTrack = 9999;
    float finishTrack = 0;
    long startTime = 9999999999999999;
    long finishTime = 0;
    //Find them
    while(indata){
        indata >> row;
        std::istringstream ss(row);
        std::string token;
        std::vector <std::string> k;
        //Extract row
        while (getline(ss, token, ',')) {
            k.push_back(token);
        }

        long globalTime; std::stringstream gt(k[3]); gt >> globalTime;
        int lane; std::stringstream l(k[13]); l >> lane;
        float localY; std::stringstream y(k[5]); y >> localY;
        localY = static_cast<float>(localY * 0.3048);

        if(globalTime > finishTime){
            finishTime = globalTime; }
        if(globalTime < startTime){
            startTime = globalTime; }
        if(lane > numberOfLanes){
            numberOfLanes = lane; }
        if(localY > finishTrack){
            finishTrack = localY; }
        if(localY < startTrack){
            startTrack = localY; }
    }

    //Calculate number of periods
    int lengthPeriod = 60000;
    long numberOfPeriods = ( finishTime - startTime ) / lengthPeriod + 1;

    //Calculate length of track
    float totalLength = finishTrack - startTrack;

    //Create Map of SampleTimes
    std::map <long, SampleTime> ST;
    SampleTime sampleTime;
    for(long i=startTime; i<finishTime ; i=i+100){
        ST.insert(std::pair <long, SampleTime> (i, sampleTime));
    }

    //Create Map of Statistics
    std::map <int, Statistics> Stats;
    Statistics statistics;
    for(int i=1; i<=numberOfPeriods; i++){
        Stats.insert(std::pair <int, Statistics> (i, statistics));
    }

    //Create a vector to contain all lanes
    std::vector <Lane> lanesVector;
    for(int i = 1; i <= numberOfLanes; i++){
        Lane temp;
        temp.laneID = i;
        temp.st = ST;
        temp.stats = Stats;
        lanesVector.push_back(temp);
    }

    //Open general CSV again
    indata.close();
    std::ifstream indata2;
    indata2.open("../../CSV/FromGettingCSV/us-101-3-3.csv");
    //Skip title
    indata2 >> row;

    //Read CSV again and store vehicles and speed in SampleTime
    while(indata2) {
        indata2 >> row;
        std::istringstream ss(row);
        std::string token;
        std::vector<std::string> k;

        //Extract row
        while (getline(ss, token, ',')) {
            k.push_back(token);
        }

        //Read global time, lane, v_ID and speed for every row
        long globalTime; std::stringstream gt(k[3]); gt >> globalTime;
        int lane;std::stringstream l(k[13]); l >> lane;
        int v_ID; std::stringstream v(k[0]); v >> v_ID;
        float speed; std::stringstream s(k[11]); s >> speed; speed = static_cast<float> (speed * 0.3048);

        //Insert speed and v_ID in its time slot
        auto it = lanesVector[lane - 1].st.find(globalTime);
        if (it != lanesVector[lane - 1].st.end()) {
            it->second.vehicles.push_back(v_ID);
            it->second.speed.push_back(speed);
        }
    }
    //Open CONFLICTS CVS
    indata2.close();
    std::ifstream indata3;
    indata3.open("../../CSV/FromPrintParameters/Conflicts-us-101-3-3.csv");
    //Skip title
    indata3 >> row;

    //Read safety parameters
    while(indata3) {
        indata3 >> row;
        std::istringstream ss(row);
        std::string token;
        std::vector<std::string> k;

        //Extract row
        while (getline(ss, token, ',')) {
            k.push_back(token);
        }
        //Read Type, TTC, PSD, H
        std::string type; std::stringstream ty(k[0]); ty >> type;
        float conflict; std::stringstream con(k[1]); con >> conflict;
        long globalTime; std::stringstream gt(k[3]); gt >> globalTime;
        int lane; std::stringstream l(k[4]); l >> lane;

        //Insert conflict value in its time slot
        auto it = lanesVector[lane - 1].st.find(globalTime);
        if (it != lanesVector[lane - 1].st.end()) {
            //Push back the value
            if(type == "TTC"){
                it->second.TTC.push_back(conflict);
            }
            if(type == "H"){
                it->second.H.push_back(conflict);
            }
            if(type == "PSD"){
                it->second.PSD.push_back(conflict);
            }
            if(type == "ACI"){
                it->second.ACI.push_back(conflict);
            }
        }
    }
    //CREATE VECTOR OF LANES, WITH MAPS PER GLOBAL TIME AND STATISTICS
    //For each Lane
    for(int i = 0; i<numberOfLanes; i++){
        //Calculate average speed, density, and traffic flow per each time slot
        std::vector <float> averageSpeedVector, densityVector, trafficFlowVector;
        std::vector <float> TTCVector, HVector, PSDVector;
        std::vector <double> ACIVector;
        std::map <long, SampleTime> :: iterator ite;
        //Initialise counters
        int periodCounter = 0;
        int tenthCounter = 0;
        for(ite = lanesVector[i].st.begin(); ite != lanesVector[i].st.end(); ++ite) {
            //Avoid -NAN
            if(ite->second.speed.empty()){
                ite->second.averageSpeed = 0;
            }
            //Calculate average speed
            else{
                ite->second.averageSpeed = static_cast<float> (accumulate(ite->second.speed.begin(), ite->second.speed.end(), 0.0) / ite->second.speed.size());
            }
            //Calculate density
            ite->second.density = ite->second.vehicles.size() / totalLength;
            //Calculate traffic flow
            ite->second.trafficFlow = ite->second.density * ite->second.averageSpeed;

            //Accumulate data for Statistics
            averageSpeedVector.push_back(ite->second.averageSpeed);
            densityVector.push_back(ite->second.density);
            trafficFlowVector.push_back(ite->second.trafficFlow);
            //Accumulate conflicts
            for(float j : ite->second.TTC) {
                TTCVector.push_back(j);
            }
            for(float j : ite->second.PSD) {
                PSDVector.push_back(j);
            }
            for(float j : ite->second.H) {
                HVector.push_back(j);
            }
            for(double j : ite->second.ACI) {
                ACIVector.push_back(j);
            }

            tenthCounter = tenthCounter + 100;

            //Separate groups of "lengthPeriod" to apply STATISTICS
            if (tenthCounter >= lengthPeriod) {
                periodCounter = periodCounter + 1;
                //Reset tenthCounter
                tenthCounter = 0;
                auto it = lanesVector[i].stats.find(periodCounter);
                //Calculate average traffic flow, density and average speed. Store them
                if(it != lanesVector[i].stats.end()) {
                    it->second.averageSpeed = static_cast<float> (accumulate(averageSpeedVector.begin(), averageSpeedVector.end(), 0.0) / averageSpeedVector.size());
                    it->second.trafficFlow = static_cast<float> (accumulate(trafficFlowVector.begin(), trafficFlowVector.end(), 0.0) / trafficFlowVector.size());
                    it->second.density = static_cast<float> (accumulate(densityVector.begin(), densityVector.end(), 0.0) / densityVector.size());
                    //Accumulate safety parameters
                    it->second.H = static_cast<float>(accumulate(HVector.begin(), HVector.end(), 0.0));
                    it->second.PSD = static_cast<float>(accumulate(PSDVector.begin(), PSDVector.end(), 0.0));
                    it->second.TTC = static_cast<float>(accumulate(TTCVector.begin(), TTCVector.end(), 0.0));
                    it->second.ACI = accumulate(ACIVector.begin(), ACIVector.end(), 0.0);
                    //Reset vectors
                    averageSpeedVector.clear();
                    trafficFlowVector.clear();
                    densityVector.clear();
                    HVector.clear();
                    PSDVector.clear();
                    TTCVector.clear();
                    ACIVector.clear();
                }
            }
        }
    }
    //NOW THE VECTOR OF LANES IS IMPLEMENTED

    //Print out STATISTICS
    for(int i = 0; i<numberOfLanes; i++){
        std::map <int, Statistics> :: iterator ite;
        for(ite = lanesVector[i].stats.begin(); ite != lanesVector[i].stats.end(); ite++){
            std::string string;
            string.append(std::to_string(lanesVector[i].laneID) + ",");
            string.append(std::to_string(ite->first) + ",");
            string.append(std::to_string(ite->second.averageSpeed) + ",");
            string.append(std::to_string(ite->second.density) + ",");
            string.append(std::to_string(ite->second.trafficFlow) + ",");
            string.append(std::to_string(ite->second.TTC) + ",");
            string.append(std::to_string(ite->second.H) + ",");
            string.append(std::to_string(ite->second.PSD)+ ",");
            string.append(std::to_string((ite->second.ACI)));
            outdata << string << std::endl;
        }
    }
}