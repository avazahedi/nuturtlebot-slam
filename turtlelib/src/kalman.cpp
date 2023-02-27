#include "turtlelib/kalman.hpp"
#include <cmath>
#include <iostream>

namespace turtlelib{

    EKF::EKF(): 
        q{RobotConfig {0.0, 0.0, 0.0}},
        q_prev{RobotConfig {0.0, 0.0, 0.0}},
        xi{arma::vec(2*N+3, arma::fill::zeros)},
        xi_pred{arma::vec(2*N+3, arma::fill::zeros)},
        xi_prev{arma::vec(2*N+3, arma::fill::zeros)}
    {
        init_covariance();
    }
    
    EKF::EKF(RobotConfig rq): 
        q{rq},
        q_prev{rq},
        xi{arma::vec(2*N+3, arma::fill::zeros)},
        xi_pred{arma::vec(2*N+3, arma::fill::zeros)},
        xi_prev{arma::vec(2*N+3, arma::fill::zeros)}
    {
        xi(0) = q.theta;
        xi(1) = q.x;
        xi(2) = q.y;

        init_covariance();
    }

    void EKF::setConfig(RobotConfig new_config)
    {
        q.theta = new_config.theta;
        q.x = new_config.x;
        q.y = new_config.y;
    }

    arma::Mat<double> EKF::A_mat(arma::vec dq)
    {
        auto At = arma::Mat<double>(2*N+3, 2*N+3, arma::fill::eye);    // A_t matrix
        At(1,0) += -dq(2);
        At(2,0) += dq(1);
        return At;
    }

    void EKF::init_covariance()
    {
        arma::mat sig0q(3,3, arma::fill::zeros);
        arma::mat topR(3,2*N, arma::fill::zeros);
        arma::mat botL(2*N,3, arma::fill::zeros);
        arma::mat sig0m(2*N,2*N, arma::fill::eye);
        sig0m *= 9999.0;

        auto join_top = std::move(arma::join_rows(sig0q, topR));
        auto join_bot = std::move(arma::join_rows(botL, sig0m));
        auto sig0 = std::move(arma::join_cols(join_top, join_bot));
        covariance = sig0;
        covar_pred = sig0;
        covar_prev = sig0;
    }

    void EKF::predict()
    {
        arma::vec dq {normalize_angle(q.theta-q_prev.theta), q.x-q_prev.x, q.y-q_prev.y};

        // state prediction
        arma::vec dq_term = arma::join_cols(dq, arma::vec(2*N, arma::fill::zeros));
        xi_pred = xi_prev + dq_term;
        xi_prev = xi_pred; // update previous state

        // covariance prediction
        arma::mat At = A_mat(dq);
        double Q = 0.0; // 1.0, Nick says identity matrix is a good starting point I_3x3 
        // in reality make Q a zero mean gaussian variable
        arma::mat Q_bar(2*N+3, 2*N+3, arma::fill::zeros);
        Q_bar.submat(0, 0, 2, 2).eye();
        Q_bar(0,0) *= Q;
        Q_bar(1,1) *= Q;
        Q_bar(2,2) *= Q;
        covar_pred = At*covar_prev*At.t() + Q_bar;
    }

    void EKF::update(double obs_x, double obs_y, unsigned int j)
    {
        
    }

        
};