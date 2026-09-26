#pragma once

#include <shape_msgs/msg/mesh.hpp>
#include <string_view>

namespace rco_core::scene::detail {

[[nodiscard]] bool isStepMeshUri(std::string_view uri);

[[nodiscard]] shape_msgs::msg::Mesh makeStepMesh(std::string_view uri);

} // namespace rco_core::scene::detail
