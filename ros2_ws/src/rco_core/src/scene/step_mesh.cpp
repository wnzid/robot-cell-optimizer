#include "step_mesh.hpp"

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Reader.hxx>
#include <Standard_Handle.hxx>
#include <Standard_TypeDef.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <geometry_msgs/msg/point.hpp>
#include <limits>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace rco_core::scene::detail {
namespace {

constexpr std::string_view kFileUriPrefix{"file://"};
constexpr double kMillimetersPerMeter = 1000.0;
constexpr double kLinearDeflectionMeters = 0.001;
constexpr double kAngularDeflectionRadians = 0.5;

std::string lowercaseAscii(std::string value) {
  for (char& character : value) {
    if (character >= 'A' && character <= 'Z') {
      character = static_cast<char>(character - 'A' + 'a');
    }
  }
  return value;
}

std::filesystem::path localStepPath(std::string_view uri) {
  if (uri.starts_with(kFileUriPrefix)) {
    uri.remove_prefix(kFileUriPrefix.size());
  } else if (uri.find("://") != std::string_view::npos) {
    throw std::invalid_argument("STEP mesh URI must be a local path or file:// URI: '" +
                                std::string(uri) + "'.");
  }
  return {std::string(uri)};
}

std::uint32_t meshIndex(std::size_t vertex_offset, Standard_Integer one_based_index) {
  if (one_based_index < 1) {
    throw std::invalid_argument("STEP tessellation produced an invalid vertex index.");
  }
  return static_cast<std::uint32_t>(vertex_offset + static_cast<std::size_t>(one_based_index - 1));
}

void appendFace(shape_msgs::msg::Mesh& mesh, const TopoDS_Face& face) {
  TopLoc_Location location;
  const Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);
  if (triangulation.IsNull()) {
    return;
  }

  const Standard_Integer node_count = triangulation->NbNodes();
  if (node_count < 1) {
    return;
  }
  const std::size_t vertex_offset = mesh.vertices.size();
  const auto maximum_index = static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max());
  const auto added_vertices = static_cast<std::size_t>(node_count);
  if (vertex_offset > maximum_index || added_vertices > maximum_index - vertex_offset) {
    throw std::invalid_argument("STEP tessellation exceeds ROS mesh index capacity.");
  }

  mesh.vertices.reserve(vertex_offset + added_vertices);
  const auto transformation = location.Transformation();
  for (Standard_Integer index = 1; index <= node_count; ++index) {
    const auto point = triangulation->Node(index).Transformed(transformation);
    geometry_msgs::msg::Point vertex;
    vertex.x = point.X();
    vertex.y = point.Y();
    vertex.z = point.Z();
    mesh.vertices.push_back(vertex);
  }

  const Standard_Integer triangle_count = triangulation->NbTriangles();
  for (Standard_Integer index = 1; index <= triangle_count; ++index) {
    Standard_Integer first = 0;
    Standard_Integer second = 0;
    Standard_Integer third = 0;
    triangulation->Triangle(index).Get(first, second, third);
    if (face.Orientation() == TopAbs_REVERSED) {
      std::swap(second, third);
    }
    shape_msgs::msg::MeshTriangle triangle;
    triangle.vertex_indices = {meshIndex(vertex_offset, first), meshIndex(vertex_offset, second),
                               meshIndex(vertex_offset, third)};
    mesh.triangles.push_back(triangle);
  }
}

} // namespace

bool isStepMeshUri(std::string_view uri) {
  const auto extension =
      lowercaseAscii(std::filesystem::path(std::string(uri)).extension().string());
  return extension == ".step" || extension == ".stp";
}

shape_msgs::msg::Mesh makeStepMesh(std::string_view uri) {
  const std::filesystem::path path = localStepPath(uri);
  const std::string path_string = path.string();
  STEPControl_Reader reader;
  if (reader.ReadFile(path_string.c_str()) != IFSelect_RetDone) {
    throw std::invalid_argument("Failed to read STEP collision geometry from '" + std::string(uri) +
                                "'.");
  }

  reader.SetSystemLengthUnit(kMillimetersPerMeter);
  if (reader.TransferRoots() < 1) {
    throw std::invalid_argument("STEP collision geometry contains no transferable shapes: '" +
                                std::string(uri) + "'.");
  }
  const TopoDS_Shape shape = reader.OneShape();
  if (shape.IsNull()) {
    throw std::invalid_argument("STEP collision geometry produced an empty shape: '" +
                                std::string(uri) + "'.");
  }

  const BRepMesh_IncrementalMesh tessellator(shape, kLinearDeflectionMeters, false,
                                             kAngularDeflectionRadians, true);
  if (!tessellator.IsDone()) {
    throw std::invalid_argument("Failed to tessellate STEP collision geometry from '" +
                                std::string(uri) + "'.");
  }

  shape_msgs::msg::Mesh mesh;
  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
    appendFace(mesh, TopoDS::Face(explorer.Current()));
  }
  if (mesh.triangles.empty()) {
    throw std::invalid_argument("STEP collision geometry produced no triangles: '" +
                                std::string(uri) + "'.");
  }
  return mesh;
}

} // namespace rco_core::scene::detail
