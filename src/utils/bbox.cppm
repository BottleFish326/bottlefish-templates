module;
#include <Eigen/Core>
#include <absl/container/fixed_array.h>

export module bottlefish.utils.bbox;

namespace bottlefish::utils {
export template <typename Scalar = double>
struct BBox {
    using MatrixS = Eigen::Matrix<Scalar, 2, Eigen::Dynamic, Eigen::RowMajor>;
    using VectorS = Eigen::Vector<Scalar, Eigen::Dynamic>;
    BBox(std::size_t dim) : _box(dim * 2), _center(dim) {}
    BBox() : _box(0), _center(0) {}
    
    BBox(const BBox &) = default;
    BBox(BBox &&) = default;
    BBox& operator=(const BBox &) = delete;

    Scalar squared_radius() const {
        VectorS v1 = box().row(1);
        VectorS v2 = this->center();
        return (v1-v2).squaredNorm();
    }
    std::size_t dim() const {
        return _center.size();
    }

    auto span_vec() const {
        return box().row(1).transpose() - box().row(0);
    }
    absl::FixedArray<Scalar> span() const {
        absl::FixedArray<Scalar> ret(dim());
        for (std::size_t i = 0; i < dim(); ++i) ret[i] = span(i);
        return ret;
    }
    
    Scalar span(std::size_t i) const {
        return box()(1, i) - box(0, i);
    }
    Scalar coord_min(std::size_t i) const {
        return box()(0, i);
    }
    Scalar coord_max(std::size_t i) const {
        return box()(1, i);
    }
    auto box() const {
        return Eigen::Map<const MatrixS>(_box.data(), _box.size() / 2);
    }
    auto center() const {
        return Eigen::Map<const VectorS>(_center.data(), _center.size());
    }
    auto box_mut() {
        return Eigen::Map<MatrixS>(_box.data(), _box.size() / 2);
    }
    auto center_mut() {
        return Eigen::Map<VectorS>(_center.data(), _center.size());
    }
    void set_center() {
        center_mut() = (box().row(0) + box().row(1)) / 2;
    }

private:
    absl::FixedArray<Scalar> _box;
    absl::FixedArray<Scalar> _center;

};


} // namespace zjucad::dsdf::utils