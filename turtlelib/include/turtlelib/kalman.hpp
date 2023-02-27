#ifndef KALMAN_INCLUDE_GUARD_HPP
#define KALMAN_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.

#include "turtlelib/diff_drive.hpp"
#include <vector>
#include <armadillo>
#include <unordered_set>

namespace turtlelib
{
    /// @brief number of obstacles
    constexpr int N = 3;

    /// \brief Modeling the kinematics of a differential drive robot with a given 
    /// wheel track and wheel radius.
    class EKF
    { 
        RobotConfig q;          // current robot config
        RobotConfig q_prev;     // previous robot_config
        arma::vec xi;           // combined state vector xi_t
        arma::vec xi_pred;      // predicted state vector xi_t_hat_minus
        arma::mat covariance;   // covariance matrix sigma
        arma::mat covar_pred;   // covariance prediction sigma_t_hat_minus  
        double Q_val;           // basic sensor noise (from get_random() from zero mean Gaussian dist w/variance)
        double R_val;           // noise for 2nx2n covariance matrix
        std::unordered_set<unsigned int> measure_set{}; // set of measurements to keep track of what landmarks we've seen before

    public:
        /// @brief Default EKF constructor.
        EKF();

        /// @brief EKF given robot config and obstacles.
        /// @param rq - robot config
        EKF(RobotConfig rq);

        /// @brief Set robot config variable q
        /// @param new_config 
        void setConfig(RobotConfig new_config);

        /// @brief Get state estimate xi_pred
        /// @return state estimate xi_pred
        arma::vec getStateEst();

        /// @brief Calculate A_t matrix
        /// @param dq - change in robot state 
        /// @return At
        arma::Mat<double> A_mat(arma::vec dq);

        /// @brief Initialize covariance matrix (Sigma_0)
        void init_covariance();

        /// @brief Compute state and covariance prediction
        void predict();

        /// @brief Update/correct the state and covariance predictions
        /// @param obs_x - sensed obstacle x-coordinate
        /// @param obs_y - sensed obstacle y-coordinate
        /// @param j - sensed obstacle index
        /// @return xi - the corrected state prediction
        arma::vec update(double obs_x, double obs_y, unsigned int j);

        // arma::mat update(double obs_x, double obs_y, unsigned int j);

    };

}

#endif