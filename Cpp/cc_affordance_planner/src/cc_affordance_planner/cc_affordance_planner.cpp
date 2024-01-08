#include <cc_affordance_planner/cc_affordance_planner.hpp>
CcAffordancePlanner::CcAffordancePlanner() {}

PlannerResult CcAffordancePlanner::affordance_stepper(const Eigen::MatrixXd &slist, const double &theta_adf,
                                                      const int task_offset_tau)
{

    PlannerResult plannerResult; // Result of the planner

    //** Alg1:L1: Determine relevant matrix and vector sizes based on task_offset_tau
    const size_t robot_chain_noj = slist.cols() - aff_chain_noj_; // nof joints the robot has

    // Declare thetalist_ with correct size for later use in IK solver and closure_error_optimizer. This is to prevent
    // the overhead of a variable declaration inside functions that are call in a loop
    thetalist_.conservativeresize(slist.cols()); // helper vector variable holding theta_p, theta_sg

    //**Alg1:L1 and Alg1:L2: Set start guesses and step goal
    Eigen::VectorXd theta_sg = Eigen::VectorXd::Zero(task_offset_tau);
    Eigen::VectorXd theta_pg = Eigen::VectorXd::Zero(robot_chain_noj - task_offset_tau);
    Eigen::VectorXd theta_sd = theta_sg; // We extract the size for theta_sd and also the reference to
                                         // start from. We'll set the affordance goal in the loop with
                                         // respect to this reference

    //**Alg1:L3: Compute no. of iterations, stepper_max_itr_m to final goal, theta_adf
    const int stepper_max_itr_m = theta_adf / p_aff_step_deltatheta_a + 1;

    //**Alg1:L4: Initialize loop counter, loop_counter_k; success counter, success_counter_s
    int loop_counter_k = 0;
    int success_counter_s = 0;

    auto start_time = std::chrono::high_resolution_clock::now();

    //**Alg1:L5: Define affordance step, p_aff_step_deltatheta_a : Defined as class variable

    while (loop_counter_k < stepper_max_itr_m) //**Alg1:L6
    {

        loop_counter_k = loop_counter_k + 1; //**Alg1:L7:

        //**Alg1:L8: Update aff step goal:
        // If last iteration, adjust affordance step accordingly
        if (loop_counter_k == (stepper_max_itr_m - 1)) //**Alg1:L9
        {
            p_aff_step_deltatheta_a = theta_adf - p_aff_step_deltatheta_a * (stepper_max_itr_m - 1); //**Alg1:L10
        }                                                                                            //**Alg1:L11

        // Set the affordance step goal as aff_step away from the current pose
        theta_sd(task_offset_tau_ - 1) = theta_sd(task_offset_tau_ - 1) + p_aff_step_deltatheta_a; //**Alg1:L12

        //**Alg1:L13: Call Algorithm 2 with args, theta_sd, theta_pg, theta_sg, slist
        std::optional<Eigen::VectorXd> ik_result =
            CcAffordancePlanner::cc_ik_solver(slist, theta_pg, theta_sg, theta_sd);

        if (ik_result.has_value()) //**Alg1:L14
        {
            //**Alg1:L15: Record solution, theta_p, theta_sg
            const Eigen::VectorXd &traj_point = ik_result.value();
            plannerResult.jointTraj.push_back(traj_point); // recorded as a point in the trajectory solution

            //**Alg1:L16 Update guesses, theta_pg, theta_sg
            theta_sg = traj_point.tail(task_offset_tau_);
            theta_pg = traj_point.head(thetalist_0.size() - task_offset_tau_);

            success_counter_s = success_counter_s + 1; //**Alg1:L17
        }                                              //**Alg1:L18

    } //**Alg1:L19

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << "Planning time: " << duration.count() << " microseconds\n";

    if (!plannerResult.jointTraj.empty())
    {
        plannerResult.success = true;

        if (loop_counter_k == success_counter_s)
        {
            plannerResult.trajFullorPartial = "Full";
        }
        else
        {
            plannerResult.trajFullorPartial = "Partial";
        }
    }
    else
    {
        plannerResult.success = false;
        plannerResult.trajFullorPartial = "Unset";
    }
    return plannerResult;
}

std::optional<Eigen::VectorXd> cc_ik_solver(const Eigen::MatrixXd &slist, const Eigen::VectorXd &theta_p,
                                            const Eigen::VectorXd &theta_sg, const Eigen::VectorXd &theta_sd)
{

    //** Alg2:L1: Set dt as small time increment: Set as class private variable

    Eigen::VectorXd oldtheta_p =
        Eigen::VectorXd::Zero(theta_p.size()); // capture theta_p to compute joint velocities below for Alg2:L8

    //**Alg2:L2: Initialize loop counter, loop_counter_i
    int loop_counter_i = 0;

    //**Alg2:L3: Start closure error at 0
    rho = Eigen::VectorXd::Zero(twist_length_); // twist length is 6

    //**Alg2:L4: Set max. no. of iterations, p_max_itr_l, and error thresholds, p_task_err_threshold_eps_s,
    // p_closure_err_threshold_eps_r

    // Compute error
    bool err =
        (((theta_sd - theta_sg).norm() > p_task_err_threshold_eps_s) || rho.norm() > p_closure_err_threshold_eps_r);

    while (err && loop_counter_i < p_max_itr_l) //**Alg2:L5
    {
        loop_counter_i = loop_counter_i + 1; //**Alg2:L6

        //**Alg2:L7: Compute Np, Ns as screw-based Jacobians
        thetalist_ << theta_p, theta_sg;
        Eigen::MatrixXd rJ = AffordanceUtil::JacobianSpace(slist, thetalist_);
        Np = rJ.leftCols(rJ.cols() - task_offset_tau_);
        Ns = rJ.rightCols(task_offset_tau_);

        //**Alg2:L8: Compute theta_pdot using Eqn. 23
        Eigen::MatrixXd theta_pdot = (theta_p - oldtheta_p) / dt_;

        //**Alg2:L9: Compute N using Eqn. 22
        Eigen::MatrixXd pinv_Ns; // pseudo-inverse of Ns
        pinv_Ns = Ns.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd pinv_theta_pdot; // pseudo-inverse of theta_pdot
        pinv_theta_pdot = theta_pdot.completeOrthogonalDecomposition().pseudoInverse();

        Eigen::MatrixXd N = -pinv_Ns * (Np + rho * pinv_theta_pdot);

        oldtheta_p = theta_p; // capture theta_p to compute joint velocities below for Alg2:L8

        //**Alg2:L10: Update theta_p using Eqn. 24
        Eigen::MatrixXd pinv_N;
        pinv_N = N.completeOrthogonalDecomposition().pseudoInverse(); // pseudo-inverse of N

        theta_p = theta_p + pinv_N * (theta_sd - theta_sg); // Update using Newton-Raphson

        //**Alg2:L11: Call Algorithm 3 with args, theta_sg, theta_p, slist, Np, Ns
        CcAffordancePlanner::closure_error_optimizer(slist, Np, Ns, theta_p,
                                                     theta_sg); // theta_sg and theta_p returned by referece

        // Check error
        err =
            (((theta_sd - theta_sg).norm() > p_task_err_threshold_eps_s) || rho.norm() > p_closure_err_threshold_eps_r);

    } //**Alg2:L12

    //**Alg2:L13: Follows
    if (!err)
    {
        thetalist_ << theta_p, theta_sg; // return thetalist_ corrected by closure_error_optimizer
        return thetalist_;
    }
    else
    {
        return std::nullopt; // Represents no value (similar to nullptr for pointers)
    }
}

void closure_error_optimizer(const Eigen::MatrixXd &slist, const Eigen::MatrixXd &Np, const Eigen::MatrixXd &Ns,
                             Eigen::VectorXd &theta_p,
                             Eigen::VectorXd &theta_sg) // theta_sg and theta_p returned by referece
{

    //**Alg3:L1: Compute forward kinematics to chain's end link, Tse
    thetalist_ << theta_p, theta_sg;
    Eigen::Matrix4d Tse = AffordanceUtil::FKinSpace(mErr_, slist, thetalist_); // HTM of actual end of ground link

    //**Alg3:L2: Compute closure error twist
    rho = AffordanceUtil::Adjoint(Tse) * AffordanceUtil::se3ToVec(AffordanceUtil::MatrixLog6(AffordanceUtil::TransInv(
                                             Tse))); // mErr_ is the desired htm for the end of ground link

    //**Alg3:L3: Adjust joint angles for closure error
    Eigen::MatrixXd N(Np.rows(), Np.cols() + Ns.cols());
    Nc << Np, Ns; // Combine Np and Ns horizontally
    Eigen::MatrixXd pinv_N;
    pinv_Nc = Nc.completeOrthogonalDecomposition().pseudoInverse(); // pseudo-inverse of N
    const Eigen::VectorXd delta_theta = pinv_Nc * rho;              // correction differential

    theta_p = theta_p + delta_theta.head(q.size() - task_offset_tau_);
    theta_sg = theta_sg + delta_theta.tail(task_offset_tau_);

    // Compute final error
    thetalist_ << theta_p, theta_sg;
    Tse = AffordanceUtil::FKinSpace(mErr_, slist, thetalist_);
    rho = AffordanceUtil::Adjoint(Tse) *
          AffordanceUtil::se3ToVec(AffordanceUtil::MatrixLog6(AffordanceUtil::TransInv(Tse)));
}
