#pragma once

#include "rco_core/domain/model.hpp"
#include "rco_core/domain/validation.hpp"

#include <optional>
#include <string_view>
#include <vector>

namespace rco_core::domain {

// CSV parsing results are intentionally plain data records, consistent with
// the domain schema DTO design.
// NOLINTBEGIN(misc-non-private-member-variables-in-classes)

struct TargetPathCsvParseResult {
  std::optional<std::vector<TargetPose>> value;
  ValidationErrors errors;

  [[nodiscard]] bool ok() const noexcept {
    return value.has_value();
  }
};

// NOLINTEND(misc-non-private-member-variables-in-classes)

// Expected header: frame_id,x_m,y_m,z_m,qx,qy,qz,qw
[[nodiscard]] TargetPathCsvParseResult parseTargetPathCsv(std::string_view csv_text);

} // namespace rco_core::domain
