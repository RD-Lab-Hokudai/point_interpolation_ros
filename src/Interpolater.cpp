#include <iostream>

#include "models/envParams.cpp"
#include "models/hyperParams.cpp"
#include "data/loadParams.cpp"
#include "interpolate.cpp"

using namespace std;
using namespace open3d;

ofstream ofs;

int main(int argc, char *argv[])
{
    string params_name = argc >= 2 ? argv[1] : "13jo_0219_rgb_mrf";
    cout << params_name << endl;
    EnvParams params_use = loadParams(params_name);
    HyperParams hyperParams = getDefaultHyperParams(params_use.isRGB);
    ofs = ofstream(params_use.of_name);

    for (int i = 0; i < params_use.data_ids.size(); i++)
    {
        double time, ssim, mse, mre;
        interpolate(params_use.data_ids[i], params_use, hyperParams, time, ssim, mse, mre, true);
        ofs << params_use.data_ids[i] << "," << time << "," << ssim << "," << mse << "," << mre << "," << endl;
    }
    return 0;
}