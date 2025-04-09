#include <iostream>
#include <math.h>
#include <tuple>

using namespace std;

double new_mean, new_var;

tuple<double, double> measurement_update(double mean1, double var1, double mean2, double var2)
{
    new_mean = (var2 * mean1 + var1 * mean2) / (var1 + var2);
    new_var = 1/ ((1/var1) + (1/var2));
    return make_tuple(new_mean, new_var);
}

tuple<double, double> state_prediction(double mean1, double var1, double mean2, double var2)
{
    new_mean = mean1 + mean2;
    new_var = var1 + var2;
    return make_tuple(new_mean, new_var);
}

int main()
{
    // Measurements and measurement variance
    double measurements[5] = {5,6,7,9,10};
    double measument_sig = 4;

    // Motions and motion variance
    double motion[5] = {1,1,2,1,1};
    double motion_sig = 2;

    // Initial state
    double mean = 4;
    double var = 5;

    // Loop through all measurements
    for(int i = 0; i<sizeof(measurements)/ sizeof(measurements[0]); i++)
    {

        // update
        tie(mean, var) = measurement_update(mean, var, measurements[i], measument_sig);
        printf("Update: [%f, %f]\n", new_mean, new_var);
        
        //prediction
        tie(mean, var) = state_prediction(mean, var, motion[i], motion_sig);
        printf("Prediction: [%f, %f]\n", new_mean, new_var);



    }
    return 0;
}

