#include "turtlelib/kalman.hpp"
#include <cmath>
#include <iostream>

namespace turtlelib{

    EKF::EKF(): 
        q{RobotConfig {0.0, 0.0, 0.0}},
        q_prev{RobotConfig {0.0, 0.0, 0.0}},
        xi{arma::vec(2*N+3, arma::fill::zeros)},
        xi_pred{arma::vec(2*N+3, arma::fill::zeros)},
        Q_val{1.0},
        R_val{1.0}
    {
        init_covariance();
    }
    
    EKF::EKF(RobotConfig rq): 
        q{rq},
        q_prev{rq},
        xi{arma::vec(2*N+3, arma::fill::zeros)},
        xi_pred{arma::vec(2*N+3, arma::fill::zeros)},
        Q_val{1.0},
        R_val{1.0}
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

    arma::vec EKF::getStateEst()
    {
        return xi_pred;
    }
    
    arma::mat EKF::getCovar()
    {
        return covariance;
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

        auto join_top = arma::join_rows(sig0q, topR);
        auto join_bot = arma::join_rows(botL, sig0m);
        auto sig0 = arma::join_cols(join_top, join_bot);
        covariance = sig0;
        covar_pred = sig0;
    }

    void EKF::predict()
    {
        arma::vec dq {normalize_angle(q.theta-q_prev.theta), q.x-q_prev.x, q.y-q_prev.y};

        // state prediction
        arma::vec dq_term = arma::join_cols(dq, arma::vec(2*N, arma::fill::zeros));
        xi_pred += dq_term;
        q_prev = q;

        // covariance prediction
        arma::mat At = A_mat(dq);
        Q_val = 1.0; // 1.0, Nick says identity matrix is a good starting point I_3x3 
        // in reality make Q a zero mean gaussian variable
        arma::mat Q_bar(2*N+3, 2*N+3, arma::fill::zeros);
        Q_bar.submat(0, 0, 2, 2).eye();
        Q_bar(0,0) *= Q_val;
        Q_bar(1,1) *= Q_val;
        Q_bar(2,2) *= Q_val;
        covar_pred = At*covariance*At.t() + Q_bar;
    }

    // arma::mat EKF::update(double obs_x, double obs_y, unsigned int j)
    arma::vec EKF::update(double obs_x, double obs_y, unsigned int j)
    {
        double xbar = obs_x;    // obstacle measurements wrt robot
        double ybar = obs_y;

        double rj = sqrt(xbar*xbar + ybar*ybar);    // ri,phii = rj, phij because we know which i corresponds to which j
        double phij = atan2(ybar, xbar);

        double obs_est_x = xi_pred(1) + rj*cos(phij+xi_pred(0));    //
        double obs_est_y = xi_pred(2) + rj*sin(phij+xi_pred(0));

        if (measure_set.count(j)==0)   // if count=0, set does not contain this landmark
        {
            xi_pred(3+2*j) = obs_est_x;
            xi_pred(3+2*j+1) = obs_est_y;
            measure_set.insert(j);
        }

        arma::vec zj = { rj, phij };

        double delta_xj = xi_pred(3+2*j) - xi_pred(1);
        double delta_yj = xi_pred(3+2*j+1) - xi_pred(2);
        double dj = delta_xj*delta_xj + delta_yj*delta_yj;

        double rj_hat = sqrt(dj);
        double phij_hat = atan2(delta_yj, delta_xj) - xi_pred(0);
        phij_hat = normalize_angle(phij_hat);
        arma::vec zj_hat = { rj_hat, phij_hat };

        // construct Hj matrix
        arma::mat Hj1(2, 3);
        arma::mat Hj2(2, 2*j, arma::fill::zeros);
        arma::mat Hj3(2, 2);
        arma::mat Hj4(2, 2*N-2*(j+1), arma::fill::zeros);

        Hj1(0,0) = 0.0;
        Hj1(0,1) = -delta_xj/sqrt(dj);
        Hj1(0,2) = -delta_yj/sqrt(dj);
        Hj1(1,0) = -1.0;
        Hj1(1,1) = delta_yj/dj;
        Hj1(1,2) = -delta_xj/dj;

        Hj3(0,0) = delta_xj/sqrt(dj);
        Hj3(0,1) = delta_yj/sqrt(dj);
        Hj3(1,0) = -delta_yj/dj;
        Hj3(1,1) = delta_xj/dj;

        arma::mat Hj = arma::join_rows(Hj1,Hj2,Hj3,Hj4);

        // calculate Rj matrix
        arma::mat Rj(2,2, arma::fill::eye);
        Rj *= R_val;

        // calculate Kalman gain for this landmark
        arma::mat Kj = (covar_pred*Hj.t())*((Hj*covar_pred*Hj.t() + Rj).i());

        // correct the state prediction
        xi = xi_pred + Kj*(zj - zj_hat);
        xi(0) = turtlelib::normalize_angle(xi(0));
        xi_pred = xi;   // use the corrected prediction for the next step

        // correct the covariance prediction
        arma::mat Ic(3+2*N, 3+2*N, arma::fill::eye);
        covariance = (Ic - Kj*Hj)*covar_pred;
        covar_pred = covariance;    // use the corrected prediction for the next step

        return xi;
    }

        
};