#include "bvh/aabb.hpp"
#include <absl/container/fixed_array.h>
#include <algorithm>

namespace bottlefish::bvh {

AABBTree AABBTree::create(
    Eigen::Ref<const Eigen::MatrixXd> V,
    Eigen::Ref<const Eigen::MatrixXi> F,
    int split_limit,
    int number_children
) {
    AABBTree tree;
    split_limit = split_limit > 2 ? split_limit : 3;
    number_children = number_children > 2 ? number_children : 2;
    build_aabb_tree(tree, V, F, split_limit, number_children);
    return tree;
}

absl::FixedArray<std::span<std::size_t>> split_aabb(
    std::span<std::size_t> primitive_ids,
    std::span<const utils::BBox<double>> primitive_bbox,
    int k,
    const utils::BBox<double>& current_bbox
) {
    auto span_bbox = current_bbox.span();
    size_t axis = std::ranges::distance(
        span_bbox.begin(),
        std::ranges::max_element(span_bbox)
    );
    std::ranges::sort(
        primitive_ids, [&](size_t l, size_t r) {
            return primitive_bbox[l].center()(axis) < primitive_bbox[r].center()(axis);
        }
    );

    absl::FixedArray<std::span<std::size_t>> ret(k);
    size_t per_size = primitive_ids.size() / k;
    for(int i = 0; i < k - 1; ++i) {
        ret[i] = primitive_ids.subspan(i * per_size, per_size);
    }
    ret[k - 1] = primitive_ids.subspan(
        (k - 1) * per_size, 
        primitive_ids.size() - (k - 1) * per_size
    );
    return ret;
}

size_t construct_aabb_recursive(
    AABBTree &tree,
    std::span<std::size_t> current_primitive_ids,
    int depth,
    std::span<const utils::BBox<double>> primitive_bbox,
    int dim, int split_limit, int num_children, int max_depth
) {
    size_t node_id = tree.nodes.size();
    if(current_primitive_ids.size() < split_limit || depth >= max_depth) {
        auto &inserted_node = tree.nodes.emplace_back(AABBNode{
            .squared_radius = 0.0,
            .bbox = utils::BBox<double>(dim),
            .depth = depth,
            .ids = {current_primitive_ids.begin(), current_primitive_ids.end()},
            .is_leaf = true
        });
        inserted_node.bbox.set_box_from_boxes(dim, primitive_bbox, current_primitive_ids);
        inserted_node.squared_radius = inserted_node.bbox.squared_radius();
    } else {
        size_t node_id = tree.nodes.size();
        auto &inserted_node = tree.nodes.emplace_back(AABBNode{
            .squared_radius = 0.0,
            .bbox = utils::BBox<double>(dim),
            .depth = depth,
            .ids = absl::FixedArray<std::size_t>(num_children),
            .is_leaf = false
        });
        inserted_node.bbox.set_box_from_boxes(dim, primitive_bbox, current_primitive_ids);
        inserted_node.squared_radius = inserted_node.bbox.squared_radius();
        auto split = split_aabb(
            current_primitive_ids,
            primitive_bbox, 
            num_children, 
            inserted_node.bbox
        );
        for(int i = 0; i < num_children; ++i) {
            auto child_split = split[i];
            tree.nodes[node_id].ids[i] = construct_aabb_recursive(
                tree,
                child_split,
                depth + 1,
                primitive_bbox,
                dim,
                split_limit,
                num_children,
                max_depth  
            );
        }
    }
    return node_id;
}

} // namespace bottlefish::bvh