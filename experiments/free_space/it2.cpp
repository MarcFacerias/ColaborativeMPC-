// Test code done expanding the obstacle avoidance that Eugenio did, need to be improved but a priory should work fine
//TODO: Need to fix the code and design the code structure so that the input matrix (ordered is generated smartly )
#include <vector>
#include <algorithm>
#include <ctime>

using namespace std;

vector<float> s, ey_lb, ey_ub, r_ey_lb, r_ey_ub;
float ey_max = 10, ey_min = -10;

void check_obstable(int h_range, int l_range){

    float temp_lb = ey_min, temp_ub = ey_max;

    for (int i=l_range; i<= h_range; i++){

        if (temp_lb < ey_lb[i]){
            temp_lb = ey_lb[i];
        }

        if (temp_ub > ey_ub[i]){
            temp_ub = ey_ub[i];
        }

    }

    r_ey_lb.push_back(temp_lb);
    r_ey_ub.push_back(temp_ub);

}

void run(){
    std::clock_t c_start = std::clock();

    //Obstacle 1

    for (float cs = 0; cs<3 ; cs+=0.1){

        s.push_back(cs);
        ey_lb.push_back(1);
        ey_ub.push_back(3);

    }

    //Obstacle 2

    for (float cs = 5; cs<7 ; cs+=0.1){

        s.push_back(cs);
        ey_lb.push_back(-3);
        ey_ub.push_back(-1);

    }

    //Obstacle 3

    for (float cs = 8; cs<10 ; cs+=0.1){

        s.push_back(cs);
        ey_lb.push_back(0);
        ey_ub.push_back(3);

    }

    // Supose that we start at s = 0 up to s = 10
    int horizon = 1;
    for(float ps = 0; ps<10 ; ps+=0.1){

      // search window + 1m (we could expand it backwards)
      auto const it_ub = lower_bound(s.begin(), s.end(), ps + horizon ) - s.begin();
      auto const it_lb = upper_bound(s.begin(), s.end(), ps) - s.begin();

       cout << "Search space around " << ps << endl;
       cout << "first index: " << it_lb << " value " << s[it_lb] << endl;
       cout << "last index: "  << it_ub << " value " << s[it_ub] << endl;

      if ((abs(s[it_ub] - s[it_lb]) < horizon) && (abs(s[it_ub] - s[it_lb]) > 0)){

          check_obstable(it_ub, it_lb);

      }else{

          cout << "No limit detected" << endl;
          r_ey_lb.push_back(ey_min);
          r_ey_ub.push_back(ey_max);

      }

    }
    std::clock_t c_end = std::clock();
    double time_elapsed_ms = 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;

    int aux_s = 0;
    for (int i = 0; i < r_ey_lb.size(); i++ ){

//        cout << "s: " << s[i] << endl;
        cout << "lb: " << r_ey_lb[i] << endl;
        cout << "ub: " << r_ey_ub[i] << endl;

    }

    std::cout << "CPU time used: " << time_elapsed_ms << " ms\n";
}





