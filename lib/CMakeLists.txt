add_library(bottlefish STATIC
    bvh/aabb.cpp
)

target_compile_features(bottlefish PUBLIC cxx_std_20)

target_include_directories(bottlefish
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(bottlefish PUBLIC
    Eigen3::Eigen
    absl::base
    fmt::fmt
)