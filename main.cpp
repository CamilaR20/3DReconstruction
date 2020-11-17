#include <iostream>
#include <fstream>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;
using json = nlohmann::json;

void get_k(string dir, Matx33d *K){
    json calib;
    ifstream file(dir);
    if (!file.is_open()) {
        cout << "Unable to read file: " << dir << endl;
        exit(0);
    }
    file >> calib;
    float fx  = calib["K"][0][0], fy  = calib["K"][1][1], cx = calib["K"][0][2], cy = calib["K"][1][2];
    *K = Matx33d( fx, 0, cx,
                 0, fy, cy,
                 0, 0,  1);
}

int main(int argc, char* argv[]) {
    Matx33d K;
    get_k(argv[1], &K);

    // Print output
    cout << "----------------------------" << endl;
    cout << "Intrinsics: " << endl << K << endl << endl;
    cout << "----------------------------" << endl;

    return 0;
}

