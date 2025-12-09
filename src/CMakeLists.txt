add_library(bottlefish_templates INTERFACE)

set_property(TARGET bottlefish_templates PROPERTY CXX_SCAN_FOR_MODULES ON)

target_link_libraries(bottlefish_templates PUBLIC
    Eigen3::Eigen
    absl::base
    fmt::fmt
)

target_compile_features(bottlefish_templates PUBLIC cxx_std_23)

add_subdirectory(utils)