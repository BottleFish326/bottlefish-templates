#ifndef BOTTLEFISH_TEMPLATES_UTILS_GRID2D_HPP
#define BOTTLEFISH_TEMPLATES_UTILS_GRID2D_HPP
#include <Eigen/Core>

namespace bottlefish::utils {

void generate_grid2d(
    double left, double top, double right, double bottom, int rx, int ry,
    Eigen::MatrixXd &V, Eigen::MatrixXi &C
) {
    if(rx < 2 || ry < 2) {
        throw std::invalid_argument("Grid resolution must be at least 2 in each dimension.");
    }
    if(left > right || top > bottom) {
        throw std::invalid_argument("Invalid grid boundaries.");
    }
    V.resize(rx * ry, 2);
    double dx = (right - left) / (static_cast<double>(rx) - 1.0);
    double dy = (bottom - top) / (static_cast<double>(ry) - 1.0);
    for(long i = 0; i < ry; ++i)
        for(long j = 0; j < rx; ++j)
            V.row(i * rx + j) = Eigen::Vector2d(left + j * dx, top + i * dy);
    C.resize((rx - 1) * (ry - 1), 4);
    for(long i = 0; i < ry - 1; ++i)
        for(long j = 0; j < rx - 1; ++j)
            C.row(i * (rx - 1) + j) = Eigen::Vector4i(
                i * rx + j,
                i * rx + rx + j,
                i * rx + j + 1,
                i * rx + rx + j + 1
            );
}

} // namespace bottlefish::utils

#endif // BOTTLEFISH_TEMPLATES_UTILS_GRID2D_HPP