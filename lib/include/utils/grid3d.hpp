#ifndef BOTTLEFISH_TEMPLATES_UTILS_GRID3D_HPP
#define BOTTLEFISH_TEMPLATES_UTILS_GRID3D_HPP
#include <Eigen/Core>

namespace bottlefish::utils {

void generate_grid3d(
    double left, double top, double far, double right, double bottom, double near,
    int rx, int ry, int rz,
    Eigen::MatrixXd &V, Eigen::MatrixXi &C
) {
    if(rx < 2 || ry < 2 || rz < 2) {
        throw std::invalid_argument("Grid resolution must be at least 2 in each dimension.");
    }
    if(left > right || top > bottom || far > near) {
        throw std::invalid_argument("Invalid grid boundaries.");
    }
    V.resize(3, rx * ry * rz);
    double dx = (right - left) / (static_cast<double>(rx) - 1.0);
    double dy = (bottom - top) / (static_cast<double>(ry) - 1.0);
    double dz = (near - far) / (static_cast<double>(rz) - 1.0);
    for(long k = 0; k < rz; ++k) for(long i = 0; i < ry; ++i) for(long j = 0; j < rx; ++j)
        V.col(k * rx * ry + i * rx + j) = Eigen::Vector3d(
            left + j * dx,
            top + i * dy,
            far + k * dz
        );
    C.resize(8, (rx - 1) * (ry - 1) * (rz - 1));
    for(int k = 0; k < rz - 1; ++k) for(int i = 0; i < ry - 1; ++i) for(int j = 0; j < rx - 1; ++j)
        C.col(k * rx * ry + i * rx + j) = Eigen::Vector<int, 8>{
            k * rx * ry + i * rx + j,
            k * rx * ry + i * rx + (j + 1),
            k * rx * ry + (i + 1) * rx + (j + 1),
            k * rx * ry + (i + 1) * rx + j,
            (k + 1) * rx * ry + i * rx + j,
            (k + 1) * rx * ry + i * rx + (j + 1),
            (k + 1) * rx * ry + (i + 1) * rx + (j + 1),
            (k + 1) * rx * ry + (i + 1) * rx + j
        };
}

} // namespace bottlefish::utils

#endif // BOTTLEFISH_TEMPLATES_UTILS_GRID3D_HPP