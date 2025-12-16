#ifndef BOTTLEFISH_TEMPLATES_BVH_AABB_HPP
#define BOTTLEFISH_TEMPLATES_BVH_AABB_HPP
#include <Eigen/Core>
#include <absl/container/fixed_array.h>
#include <utils/bbox.hpp>
#include <numeric>

using namespace bottlefish;

namespace bottlefish::bvh {

struct AABBNode {
    double squared_radius;
    utils::BBox<double> bbox;
    int depth;

    absl::FixedArray<std::size_t> ids;
    bool is_leaf = false;

    [[nodiscard]] double squared_distance(Eigen::VectorXd const& point) const {
        return (point - bbox.center()).squaredNorm();
    }
    [[nodiscard]] double distance_lowerbound(Eigen::VectorXd const& point) const {
        return std::sqrt(squared_distance(point)) - std::sqrt(squared_radius);
    }
    [[nodiscard]] double distance_upperbound(Eigen::VectorXd const& point) const {
        return std::sqrt(squared_distance(point)) + std::sqrt(squared_radius);
    }
    [[nodiscard]] bool far_enough(Eigen::VectorXd const& point, double beta) const {
        return squared_distance(point) >= beta * beta * squared_radius;
    }
};

struct AABBTree {
    using Node = AABBNode;
    std::vector<Node> nodes;
    static AABBTree create(
        Eigen::Ref<const Eigen::MatrixXd> V,
        Eigen::Ref<const Eigen::MatrixXi> F,
        int split_limit,
        int number_children
    );
};

int construct_aabb_recursive(
    AABBTree &tree,
    std::span<std::size_t> current_primitive_ids,
    int depth,
    std::span<const utils::BBox<double>> primitive_bbox,
    size_t dim,
    int split_limit,
    int num_children,
    int max_depth);

void build_aabb_tree(
    AABBTree& tree,
    Eigen::Ref<const Eigen::MatrixXd> V,
    Eigen::Ref<const Eigen::MatrixXi> F,
    int split_limit,
    int number_children,
    int max_depth = 14
) {
    std::size_t dim = V.rows();
    absl::FixedArray<utils::BBox<double>> primitive_bboxes(F.cols(), utils::BBox<double>(dim));

    tree.nodes.clear();

#pragma omp parallel for schedule(runtime)
    for (int i = 0; i < F.cols(); ++i) {
        primitive_bboxes[i].set_box_from_points(
            V(Eigen::all, F(Eigen::all, i))
        );
    }

    absl::FixedArray<std::size_t> primitive_ids(F.cols());
    std::iota(primitive_ids.begin(), primitive_ids.end(), 0);
    int full_size = std::min(std::pow(number_children, max_depth), 100000.0);
    tree.nodes.reserve(full_size);
    construct_aabb_recursive(
        tree, 
        primitive_ids, 
        0, 
        primitive_bboxes,
        V.rows(),
        split_limit,
        number_children,
        max_depth);
}

} // namespace bottlefish::bvh

#endif // BOTTLEFISH_TEMPLATES_BVH_AABB_HPP