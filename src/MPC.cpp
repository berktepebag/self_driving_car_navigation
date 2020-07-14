#include <iostream>
#include "rc_car_2dnav/MPC.h"

MPC mpc;

size_t N_timesteps = 10; // timestep length
double dt = 0.20;        // timestep duration

const double Lf = 0.22; //TODO: make this global parameter
double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 0.35;

size_t x_start = 0;
size_t y_start = x_start + N_timesteps;
size_t psi_start = y_start + N_timesteps;
size_t v_start = psi_start + N_timesteps;
size_t cte_start = v_start + N_timesteps;
size_t epsi_start = cte_start + N_timesteps;
size_t delta_start = epsi_start + N_timesteps;
size_t a_start = delta_start + N_timesteps - 1;

class FG_eval
{
public:
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
    void operator()(ADvector &fg, const ADvector &vars)
    {
        // Initial Constraints
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // Rest of the constraints
        for (int i = 0; i < N_timesteps - 1; i++)
        {
            CppAD::AD<double> x1 = vars[x_start + i + 1];
            CppAD::AD<double> y1 = vars[y_start + i + 1];
            CppAD::AD<double> psi1 = vars[psi_start + i + 1];
            CppAD::AD<double> v1 = vars[v_start + i + 1];
            CppAD::AD<double> cte1 = vars[cte_start + i + 1];
            CppAD::AD<double> epsi1 = vars[epsi_start + i + 1];

            CppAD::AD<double> x0 = vars[x_start + i];
            CppAD::AD<double> y0 = vars[y_start + i];
            CppAD::AD<double> psi0 = vars[psi_start + i];
            CppAD::AD<double> v0 = vars[v_start + i];
            CppAD::AD<double> cte0 = vars[cte_start + i];
            CppAD::AD<double> epsi0 = vars[epsi_start + i];

            CppAD::AD<double> delta0 = vars[delta_start + i];
            CppAD::AD<double> a0 = vars[a_start + i];

            // Reference Path
            CppAD::AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;

            // ROS_INFO("a %.2f b %.2f c %.2f d %.2f", coeffs[0],coeffs[1],coeffs[2],coeffs[3]);
            // ROS_INFO("atan: %.2f", CppAD::atan(coeffs[1]*2*x0*coeffs[2]+3*x0*x0*coeffs[3]));

            CppAD::AD<double> psides0 = CppAD::atan(coeffs[1] * 2 * x0 * coeffs[2] + 3 * x0 * x0 * coeffs[3]);

            // ROS_INFO("i: %d psi0: %.2f cos: %.2f sin: %.2f ",i, 180/3.1415*psi0,CppAD::cos(psi0),CppAD::sin(psi0));
            fg[2 + i + x_start] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[2 + i + y_start] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[2 + i + psi_start] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
            fg[2 + i + v_start] = v1 - (v0 + a0 * dt);
            fg[2 + i + cte_start] = cte1 - ((f0 - y0) + v0 * CppAD::sin(epsi0) * dt);
            fg[2 + i + epsi_start] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
        }

        fg[0] = 0;
        for (int i = 0; i < N_timesteps; i++)
        {
            fg[0] += 1000 * CppAD::pow(vars[cte_start + i], 2);  // CTE error
            fg[0] += 1000 * CppAD::pow(vars[epsi_start + i], 2); // Psi error
            fg[0] += 150 * CppAD::pow(vars[v_start + i] - ref_v, 2);
        }
        for (int i = 0; i < N_timesteps - 1; i++)
        {
            fg[0] += 50 * CppAD::pow(vars[delta_start + i], 2);
            fg[0] += 50 * CppAD::pow(vars[a_start + i], 2);
        }
        for (int i = 0; i < N_timesteps - 2; i++)
        {
            //Higher multiplier helps smoother steering transitions by looking at square of
            //the difference of delta between t+1 & t.
            //Lesson 19.10 Tuning MPC
            fg[0] += 1 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
            fg[0] += 1 * CppAD::pow(vars[a_start + i + 1] - vars[delta_start + i], 2);
        }
    }
};

MPC::MPC(){};
MPC::~MPC(){};

std::vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];

    size_t n_vars = 6 * N_timesteps + 2 * (N_timesteps - 1); // number of varibles
    size_t n_constraints = 6 * N_timesteps;                  // number of constraints

    Dvector vars(n_vars); // initial condition of variables
    for (int i = 0; i < n_vars; i++)
    {
        vars[i] = 0;
    }

    // lower and upper bounds for varibles
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    for (int i = 0; i < delta_start; i++)
    {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
    for (int i = delta_start; i < a_start; i++)
    {
        vars_lowerbound[i] = Lf * -0.55; //-0.4363 * Lf;
        vars_upperbound[i] = Lf * 0.55;  //0.4363 * Lf;
    }
    for (int i = a_start; i < n_vars; i++)
    {
        vars_lowerbound[i] = -1.0; //-1.0;
        vars_upperbound[i] = 1.0;
    }
    // lower and upper bounds for constraints
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0.0;
        constraints_upperbound[i] = 0.0;
    }
    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    FG_eval fg_eval(coeffs);

    // options
    // std::string options;
    // // turn off any printing
    // options += "Integer print_level  0\n";
    // options += "String sb            yes\n";
    // // maximum iterations
    // options += "Integer max_iter     10\n";
    // //approximate accuracy in first order necessary conditions;
    // // see Mathematical Programming, Volume 106, Number 1,
    // // Pages 25-57, Equation (6)
    // options += "Numeric tol          1e-6\n";
    // //derivative tesing
    // options += "String derivative_test   second-order\n";
    // // maximum amount of random pertubation; e.g.,
    // // when evaluation finite diff
    // options += "Numeric point_perturbation_radius   0.\n";
    // options += "Sparse  true        forward\n";
    // // options += "Numeric max_cpu_time          0.5\n";
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    CppAD::ipopt::solve_result<Dvector> solution; // solution
    CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
                                          constraints_upperbound, fg_eval, solution); // solve the problem

    // std::cout<<"solution: "<<solution.x<< std::endl;

    //
    //check some of the solution values
    //
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    //
    auto cost = solution.obj_value;
    // ROS_INFO("Cost %.2f", cost);

    std::vector<double> result;

    result.push_back(solution.x[delta_start]);
    result.push_back(solution.x[a_start]);

    for (int i = 0; i < N_timesteps - 1; i++)
    {
        result.push_back(solution.x[x_start + i + 1]);
        result.push_back(solution.x[y_start + i + 1]);
    }
    return result;
}

// Takes in coeffs and x point to calculate y point on polynomial.
double MPC::polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

Eigen::VectorXd MPC::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);

    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}