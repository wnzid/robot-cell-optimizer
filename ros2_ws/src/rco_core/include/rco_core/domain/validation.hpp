#pragma once

#include "rco_core/domain/model.hpp"

#include <string>
#include <vector>

namespace rco_core::domain {

// Validation results are intentionally plain data records.
// NOLINTBEGIN(misc-non-private-member-variables-in-classes)

struct ValidationError {
  std::string path;
  std::string message;

  bool operator==(const ValidationError&) const = default;
};

using ValidationErrors = std::vector<ValidationError>;

[[nodiscard]] ValidationErrors validate(const Pose& pose);

[[nodiscard]] ValidationErrors validate(const GeometryDefinition& geometry);

[[nodiscard]] ValidationErrors validate(const RobotDefinition& robot);

[[nodiscard]] ValidationErrors validate(const ToolDefinition& tool);

[[nodiscard]] ValidationErrors validate(const CellEntity& entity);

[[nodiscard]] ValidationErrors validate(const CellDefinition& cell);

[[nodiscard]] ValidationErrors validate(const TargetPose& target);

[[nodiscard]] ValidationErrors validate(const ProcessSegment& segment);

[[nodiscard]] ValidationErrors validate(const TaskDefinition& task);

[[nodiscard]] ValidationErrors validate(const OptimizationProblem& optimization);

[[nodiscard]] ValidationErrors validate(const StudyDefinition& study);

// NOLINTEND(misc-non-private-member-variables-in-classes)

} // namespace rco_core::domain
