#include <utils/bbox.hpp>
#include <fmt/format.h>
using namespace bottlefish;
int main() {
    Eigen::MatrixXd box_mut;
    box_mut.resize(2, 3);
    box_mut << 1, 4, 8,
               2, 6, 9;
    utils::BBox<double> bbox(2);
    bbox.set_box_from_points(box_mut);
    fmt::print("BBox center: {}, {}\n", bbox.center()(0), bbox.center()(1));
    return 0;
}