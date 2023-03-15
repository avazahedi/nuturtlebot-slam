#include "turtlelib/circle_fit.hpp"
#include <vector>
#include <armadillo>

namespace turtlelib
{
    turtlelib::Circle circle_fit(std::vector<turtlelib::Vector2D> cluster)
    {
        int n = cluster.size();
        double xsum = 0.0;
        double ysum = 0.0;
        double zsum = 0.0;

        arma::vec zvec(n);
        arma::vec xvec(n);
        arma::vec yvec(n);

        double x_hat = 0.0, y_hat = 0.0;

        for (int i=0; i<n; i++)
        {
            x_hat = cluster.at(i).x;
            y_hat = cluster.at(i).y;
            xsum += x_hat;
            ysum += y_hat;
        }
        double xmean = (1.0/n)*xsum;
        double ymean = (1.0/n)*ysum;

        double xshift = 0.0, yshift = 0.0;
        double z = 0.0;
        for (int j=0; j<n; j++)
        {
            x_hat = cluster.at(j).x;
            y_hat = cluster.at(j).y;
            xshift = x_hat - xmean;
            yshift = y_hat - ymean;
            
            z = pow(xshift,2) + pow(yshift,2);

            zsum += z;

            zvec(j) = z;
            xvec(j) = xshift;
            yvec(j) = yshift;
        }

        double zmean = (1.0/n)*zsum;

        // data matrix Z from n data points     nx4 matrix
        arma::mat Zmat(n, 4);
        for (int p=0; p < n; p++)
        {
            Zmat(p, 0) = zvec(p);
            Zmat(p, 1) = xvec(p);
            Zmat(p, 2) = yvec(p);
            Zmat(p, 3) = 1;
        }

        // constraint matrix H - Hyperaccurate algebraic fit
        arma::mat H (4, 4, arma::fill::zeros);
        H(0,0) = 8.0*zmean;
        H(0,3) = 2;
        H(1,1) = 1;
        H(2,2) = 1;
        H(3,0) = 2;

        // if issues here, try implementing H^-1 manually
        arma::mat H_inv (4, 4, arma::fill::zeros);
        H_inv(0,3) = 0.5;
        H_inv(1,1) = 1;
        H_inv(2,2) = 1;
        H_inv(3,0) = 0.5;
        H_inv(3,3) = -2.0*zmean;

        // SVD of Z
        arma::mat U(4,4,arma::fill::zeros);
        arma::vec s(4,arma::fill::zeros);
        arma::mat V(4,4,arma::fill::zeros);
        arma::svd(U,s,V,Zmat);

        arma::mat sigma = arma::diagmat(s);

        arma::vec A;
        // check if the smallest singular value sigma 4 is less than 10^-12
        if (s(3) <= 1e-12)
        {
            A = V.col(3);
        }
        else
        {
            arma::mat Y = V*sigma*V.t();

            // find eigenvalues and vectors of Q = Y*H^-1*Y
            arma::mat Q = Y*H_inv*Y;
            arma::cx_vec eigval;
            arma::cx_mat eigvec;
            arma::eig_gen(eigval, eigvec, Q);

            double min_pos_eval = 999.0;
            int min_pos_idx = -1;
            for (unsigned int k=0; k < eigval.n_elem; k++)
            {
                if (eigval(k).real() < min_pos_eval && eigval(k).real() > 0.0)
                {
                    min_pos_eval = eigval(k).real();
                    min_pos_idx = k;
                }
            }

            arma::vec A_star{eigvec(0,min_pos_idx).real(), eigvec(1,min_pos_idx).real(), eigvec(2,min_pos_idx).real(), eigvec(3,min_pos_idx).real()};
            A = Y.i()*A_star;
        }

        double a = -A(1) / (2*A(0));
        double b = -A(2) / (2*A(0));
        double R_sq = ( pow(A(1),2) + pow(A(2),2) - 4*A(0)*A(3)) / (4*pow(A(0),2));

        double cx = a + xmean;
        double cy = b + ymean;
        double radius = std::sqrt(R_sq);
        turtlelib::Vector2D center {cx, cy};
        turtlelib::Circle circle {center, radius};
        return circle;
    }

};
