#include "fed/dynamic_solution.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{   
    DynamicSolution dyn_sol;
    Matrix<double, 5, 1> x;
    Vector3d v;
    
    // x << 0,0,0,0,0;
    x << 0,0,1,0.6,0.8;
    v << 5,-2,4;
    
    auto start = chrono::steady_clock::now(); 
    dyn_sol.dynamic_matrices(x,v);
    auto end = chrono::steady_clock::now();
    chrono::duration<double, micro> elapsed = end - start;
    cout << "time: " << elapsed.count() << "us" << endl;

    cout << dyn_sol.m_dm.M9 << endl;

    cout << "==============================" << endl;

    cout << dyn_sol.m_dm.C9.transpose() << endl;

    cout << "==============================" << endl;

    cout << dyn_sol.m_dm.M << endl;

    cout << "==============================" << endl;

    cout << dyn_sol.m_dm.C.transpose() << endl;
    
    return 0;
} 