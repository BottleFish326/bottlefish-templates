#ifndef BOTTLEFISH_TEMPLATES_UTILS_BBOX_HPP
#define BOTTLEFISH_TEMPLATES_UTILS_BBOX_HPP
#include <Eigen/Core>
#include <absl/container/fixed_array.h>
#include <cstddef>

namespace bottlefish::utils {
template <typename Scalar = double>
struct BBox {
    using MatrixS = Eigen::Matrix<Scalar, 2, Eigen::Dynamic, Eigen::ColMajor>;
    using VectorS = Eigen::Vector<Scalar, Eigen::Dynamic>;
    BBox(std::size_t dim) : _box(dim * 2), _center(dim) {}
    BBox() : _box(0), _center(0) {}
    
    BBox(const BBox &) = default;
    BBox& operator=(const BBox &) = default; // actually deleted because of absl::FixedArray
    BBox(BBox &&) = default;
    BBox& operator=(BBox &&) = default; // actually deleted because of absl::FixedArray
    ~BBox() = default;

    Scalar squared_radius() const {
        VectorS v1 = box().row(1);
        VectorS v2 = this->center();
        return (v1-v2).squaredNorm();
    }
    [[nodiscard]]std::size_t dim() const {
        return _center.size();
    }

    absl::FixedArray<Scalar> span() const {
        absl::FixedArray<Scalar> ret(dim());
        for (std::size_t i = 0; i < dim(); ++i) ret[i] = span(i);
        return ret;
    }
    
    Scalar span(std::size_t i) const {
        return box()(1, i) - box()(0, i);
    }
    Scalar coord_min(std::size_t i) const {
        return box()(0, i);
    }
    Scalar coord_max(std::size_t i) const {
        return box()(1, i);
    }
    auto box() const {
        return Eigen::Map<const MatrixS>(_box.data(), 2, _box.size() / 2);
    }
    auto center() const {
        return Eigen::Map<const VectorS>(_center.data(), _center.size());
    }
    auto box_mut() {
        return Eigen::Map<MatrixS>(_box.data(), 2, _box.size() / 2);
    }
    auto center_mut() {
        return Eigen::Map<VectorS>(_center.data(), _center.size());
    }
    void set_box_from_points(
        Eigen::Ref<const Eigen::MatrixXd> points
    ) {
        std::size_t dim = points.rows();
        if(dim != this->dim()) {
            throw std::invalid_argument("Dimension mismatch in set_box_from_points.");
            return;
        }
        this->box_mut().row(0) = points.rowwise().minCoeff();
        this->box_mut().row(1) = points.rowwise().maxCoeff();
        this->set_center();
    }
    void set_box_from_boxes(
        std::size_t dim,
        std::span<BBox<Scalar> const> bboxes,
        std::span<std::size_t const> ids
    ) {
        if(dim != this->dim()) {
            throw std::invalid_argument("Dimension mismatch in set_box_from_boxes.");
            return;
        }
        this->box_mut().row(0).setConstant(std::numeric_limits<Scalar>::infinity());
        this->box_mut().row(1).setConstant(-std::numeric_limits<Scalar>::infinity());
        for(auto id : ids) {
            for(std::size_t d = 0; d < dim; ++d) {
                this->box_mut()(0, d) = std::min(this->box_mut()(0, d), bboxes[id].coord_min(d));
                this->box_mut()(1, d) = std::max(this->box_mut()(1, d), bboxes[id].coord_max(d));
            }
        }
        this->set_center();
    }
    void set_center() {
        center_mut() = (box().row(0) + box().row(1)) / 2;
    }

private:
    absl::FixedArray<Scalar> _box;
    absl::FixedArray<Scalar> _center;

};

} // namespace bottlefish::utils

#endif // BOTTLEFISH_TEMPLATES_UTILS_BBOX_HPP