#pragma once

#include "rco_core/domain/model.hpp"
#include "rco_core/domain/types.hpp"
#include "rco_core/domain/validation.hpp"

#include <optional>
#include <string>
#include <string_view>

namespace rco_core::domain {

// Parsing results are intentionally plain data records, consistent with the
// domain schema DTO design.
// NOLINTBEGIN(misc-non-private-member-variables-in-classes)

struct PoseYamlParseResult {
  std::optional<Pose> value;
  ValidationErrors errors;

  [[nodiscard]] bool ok() const noexcept {
    return value.has_value();
  }
};

struct GeometryYamlParseResult {
  std::optional<GeometryDefinition> value;
  ValidationErrors errors;

  [[nodiscard]] bool ok() const noexcept {
    return value.has_value();
  }
};

struct RobotYamlParseResult {
  std::optional<RobotDefinition> value;
  ValidationErrors errors;

  [[nodiscard]] bool ok() const noexcept {
    return value.has_value();
  }
};

struct ToolYamlParseResult {
  std::optional<ToolDefinition> value;
  ValidationErrors errors;

  [[nodiscard]] bool ok() const noexcept {
    return value.has_value();
  }
};

struct CellYamlParseResult {
  std::optional<CellDefinition> value;
  ValidationErrors errors;

  [[nodiscard]] bool ok() const noexcept {
    return value.has_value();
  }
};

struct TaskYamlParseResult {
  std::optional<TaskDefinition> value;
  ValidationErrors errors;

  [[nodiscard]] bool ok() const noexcept {
    return value.has_value();
  }
};

struct OptimizationYamlParseResult {
  std::optional<OptimizationProblem> value;
  ValidationErrors errors;

  [[nodiscard]] bool ok() const noexcept {
    return value.has_value();
  }
};

struct StudyYamlParseResult {
  std::optional<StudyDefinition> value;
  ValidationErrors errors;

  [[nodiscard]] bool ok() const noexcept {
    return value.has_value();
  }
};

// NOLINTEND(misc-non-private-member-variables-in-classes)

[[nodiscard]] PoseYamlParseResult parsePoseYaml(std::string_view yaml_text);

[[nodiscard]] GeometryYamlParseResult parseGeometryYaml(std::string_view yaml_text);

[[nodiscard]] RobotYamlParseResult parseRobotYaml(std::string_view yaml_text);

[[nodiscard]] ToolYamlParseResult parseToolYaml(std::string_view yaml_text);

[[nodiscard]] CellYamlParseResult parseCellYaml(std::string_view yaml_text);

[[nodiscard]] TaskYamlParseResult parseTaskYaml(std::string_view yaml_text);

[[nodiscard]] OptimizationYamlParseResult parseOptimizationYaml(std::string_view yaml_text);

[[nodiscard]] StudyYamlParseResult parseStudyYaml(std::string_view yaml_text);

// Keys are emitted in schema order and floating-point values use enough
// precision for an exact parse/serialize round trip.
[[nodiscard]] std::string serializePoseYaml(const Pose& pose);

[[nodiscard]] std::string serializeGeometryYaml(const GeometryDefinition& geometry);

[[nodiscard]] std::string serializeRobotYaml(const RobotDefinition& robot);

[[nodiscard]] std::string serializeToolYaml(const ToolDefinition& tool);

[[nodiscard]] std::string serializeCellYaml(const CellDefinition& cell);

[[nodiscard]] std::string serializeTaskYaml(const TaskDefinition& task);

[[nodiscard]] std::string serializeOptimizationYaml(const OptimizationProblem& optimization);

[[nodiscard]] std::string serializeStudyYaml(const StudyDefinition& study);

} // namespace rco_core::domain
