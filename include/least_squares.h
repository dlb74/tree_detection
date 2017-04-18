#ifndef LEAST_SQUARES_H
#define LEAST_SQUARES_H

#include <tuple>
#include <vector>
#include <numeric>
#include <algorithm>
#include <armadillo>

#include <iostream>

#include "const.h"

namespace lscf {
    std::tuple<Radius, V2d> least_square_circle_fit(const std::vector<V2d>& v2ds) {
        size_t data_size = v2ds.size();

        std::vector<double> x(data_size);
        std::transform(v2ds.begin(), v2ds.end(), x.begin(), [](const V2d& v2d) {
            return std::get<0>(v2d);
        });
        std::vector<double> y(data_size);
        std::transform(v2ds.begin(), v2ds.end(), y.begin(), [](const V2d& v2d) {
            return std::get<1>(v2d);
        });

        double x_avg = std::accumulate(x.begin(), x.end(), 0.0F, [](double sum, double cur_val) {
            return sum + cur_val;
        }) / data_size;
        double y_avg = std::accumulate(y.begin(), y.end(), 0.0F, [](double sum, double cur_val) {
            return sum + cur_val;
        }) / data_size;

        std::vector<double> u(data_size);
        for (size_t i = 0; i < data_size; i++) { u[i] = x[i] - x_avg; }
        std::vector<double> v(data_size);
        for (size_t i = 0; i < data_size; i++) { v[i] = y[i] - y_avg; }

        double s_uu = std::accumulate(u.begin(), u.end(), 0.0F, [](double sum, double cur_val) {
            return sum + cur_val * cur_val;
        });
        double s_uuu = std::accumulate(u.begin(), u.end(), 0.0F, [](double sum, double cur_val) {
            return sum + cur_val * cur_val * cur_val;
        });
        double s_vv = std::accumulate(v.begin(), v.end(), 0.0F, [](double sum, double cur_val) {
            return sum + cur_val * cur_val;
        });
        double s_vvv = std::accumulate(v.begin(), v.end(), 0.0F, [](double sum, double cur_val) {
            return sum + cur_val * cur_val * cur_val;
        });

        double s_uv = 0.0F;
        for (size_t i = 0; i < data_size; i++) { s_uv += u[i] * v[i]; }
        double s_uvv = 0.0F;
        for (size_t i = 0; i < data_size; i++) { s_uvv += u[i] * v[i] * v[i]; }
        double s_vuu = 0.0F;
        for (size_t i = 0; i < data_size; i++) { s_vuu += v[i] * u[i] * u[i]; }

        auto b_1 = (s_uuu + s_uvv) * 0.5F;
        auto b_2 = (s_vvv + s_vuu) * 0.5F;

        arma::mat A = { { s_uu, s_uv },
                        { s_uv, s_vv } };
        arma::vec b = { b_1, b_2 };

        arma::vec x2 = arma::solve(A, b);

        double u_c = x2[0];
        double v_c = x2[1];
        double radius = std::sqrt(u_c * u_c + v_c * v_c + (s_uu + s_vv) / data_size );

        double x_c = u_c + x_avg;
        double y_c = v_c + y_avg;

        return std::make_tuple(radius, V2d({x_c, y_c}));
    };
}

#endif //LEAST_SQUARES_CIRCLE_FIT_LEASTSQUARESCIRCLEFIT_H
