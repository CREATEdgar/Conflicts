#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <sstream>
#include <vector>

using namespace std;

int main()
{
	ifstream indata;
	ofstream outdata;
	outdata.open("../../CSV/FromGettingCSV/us-101-3-3.csv", ios::app);
	indata.open("../../CSV/NGSIM.csv");
    string row;
    indata >> row;
	/* Write down the TITLES */
    outdata << row << endl;
    int v_id = 0;
    int new_v_id;
    bool print = false;
    while(indata) {
        indata >> row;
        istringstream ss(row);
        string token;
        vector <string> v;
        while(getline(ss, token, ',')) {
            v.push_back(token);
        }
        std::string lastCar;
        std::string lastMoment;

        if(print){
            new_v_id ; stringstream ss2(v[0]) ; ss2 >> new_v_id;
            if(new_v_id >= v_id){
                if(v[10]!="1" && (v[24]=="us-101")){
                    outdata << row << endl;
                    v_id = new_v_id;
                }
            }
        }

        if(v[0] == "1914" && v[3] == "1118848579300" && v[24]=="us-101"){
            print = true;
        }


    }
	return 0;
}