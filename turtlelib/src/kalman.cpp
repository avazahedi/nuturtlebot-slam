#include "turtlelib/kalman.hpp"
#include <cmath>
#include <iostream>

namespace turtlelib{

    EKF::EKF(): 
        q{RobotConfig {0.0, 0.0, 0.0}},
        q_prev{RobotConfig {0.0, 0.0, 0.0}},
        xi{arma::vec(2*N+3, arma::fill::zeros)},
        xi_pred{arma::vec(2*N+3, arma::fill::zeros)}
    {
        init_covariance();
    }
    
    EKF::EKF(RobotConfig rq): 
        q{rq},
        q_prev{rq},
        xi{arma::vec(2*N+3, arma::fill::zeros)},
        xi_pred{arma::vec(2*N+3, arma::fill::zeros)}
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
        double Q_val = 0.1;
        arma::mat Q_bar(2*N+3, 2*N+3, arma::fill::zeros);
        Q_bar.submat(0, 0, 2, 2).eye();
        Q_bar(0,0) *= Q_val;
        Q_bar(1,1) *= Q_val;
        Q_bar(2,2) *= Q_val;
        covar_pred = At*covariance*At.t() + Q_bar;
    }


    int EKF::data_association(double cx, double cy)
    {    
        double r = sqrt(cx*cx + cy*cy);
        double phi = atan2(cy, cx);
        arma::vec z = { r, phi };

        arma::vec tmp = xi_pred;
        // tmp(3+2*num_seen_ldmk) = tmp(1) + r*cos(phi+tmp(0));
        // tmp(3+2*num_seen_ldmk+1) = tmp(2) + r*sin(phi+tmp(0));

        tmp(3+2*num_seen_ldmk+1) = tmp(1) + r*cos(phi+tmp(0));
        tmp(3+2*num_seen_ldmk+1+1) = tmp(2) + r*sin(phi+tmp(0));

        std::vector<double> mdist_list;

        for (int k=0; k < num_seen_ldmk+1; k++)
        {
            // construct Hk matrix
            double delta_xk = tmp(3+2*k) - tmp(1);
            double delta_yk = tmp(3+2*k+1) - tmp(2);
            double dk = delta_xk*delta_xk + delta_yk*delta_yk;
            
            arma::mat Hk1(2, 3);
            arma::mat Hk2(2, 2*k, arma::fill::zeros);
            arma::mat Hk3(2, 2);
            arma::mat Hk4(2, 2*N-2*(k+1), arma::fill::zeros);

            Hk1(0,0) = 0.0;
            Hk1(0,1) = -delta_xk/sqrt(dk);
            Hk1(0,2) = -delta_yk/sqrt(dk);
            Hk1(1,0) = -1.0;
            Hk1(1,1) = delta_yk/dk;
            Hk1(1,2) = -delta_xk/dk;

            Hk3(0,0) = delta_xk/sqrt(dk);
            Hk3(0,1) = delta_yk/sqrt(dk);
            Hk3(1,0) = -delta_yk/dk;
            Hk3(1,1) = delta_xk/dk;

            arma::mat Hk = arma::join_rows(Hk1,Hk2,Hk3,Hk4);

            // R
            arma::mat R(2,2, arma::fill::eye);
            double R_val = 0.01;
            R *= R_val;

            // compute covariance psi
            arma::mat psi = Hk*covar_pred*Hk.t() + R;

            // expected measurement z_hat
            double r_hat = sqrt(dk);
            double phi_hat = atan2(delta_yk, delta_xk) - tmp(0);
            phi_hat = normalize_angle(phi_hat);
            arma::vec z_hat = { r_hat, phi_hat };

            // compute mahalanobis distance m_dist
            arma::vec zdiff = z - z_hat;
            zdiff(1) = normalize_angle(zdiff(1));
            arma::mat m_dist = zdiff.t()*psi.i()*zdiff;

            mdist_list.push_back(m_dist(0));
        }

        double thresh = mdist_list.at(mdist_list.size()-1);
        bool new_ldmk = true;
        int l = num_seen_ldmk+1;

        for (size_t i=0; i < mdist_list.size(); i++)
        {
            if (mdist_list.at(i) < thresh)
            {
                new_ldmk = false;
                thresh = mdist_list.at(i);
                l = i;
            }
        }

        if (new_ldmk == true)
        {
            num_seen_ldmk++;
        }

        return l;

    }


    void EKF::update(double obs_x, double obs_y, unsigned int j)
    {
        // ri, phii = rj, phij because we know which i corresponds to which j
        double rj = sqrt(obs_x*obs_x + obs_y*obs_y);
        double phij = atan2(obs_y, obs_x);

        if (measure_set.count(j)==0)   // if count=0, set does not contain this landmark
        {
            xi_pred(3+2*j) = xi_pred(1) + rj*cos(phij+xi_pred(0));
            xi_pred(3+2*j+1) = xi_pred(2) + rj*sin(phij+xi_pred(0));
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
        double R_val = 0.1;
        Rj *= R_val;

        // calculate Kalman gain for this landmark
        arma::mat Kj = (covar_pred*Hj.t())*((Hj*covar_pred*Hj.t() + Rj).i());

        // correct the state prediction
        arma::mat zterm = zj - zj_hat;
        zterm(1) = turtlelib::normalize_angle(zterm(1));
        // xi = xi_pred + Kj*(zj - zj_hat);
        xi = xi_pred + Kj*zterm;
        xi(0) = turtlelib::normalize_angle(xi(0));
        xi_pred = xi;   // use the corrected prediction for the next step

        // correct the covariance prediction
        arma::mat Ic(3+2*N, 3+2*N, arma::fill::eye);
        covariance = (Ic - Kj*Hj)*covar_pred;
        covar_pred = covariance;    // use the corrected prediction for the next step
    }

        
};