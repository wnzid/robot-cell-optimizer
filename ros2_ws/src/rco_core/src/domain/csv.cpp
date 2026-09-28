#include "rco_core/domain/csv.hpp"

#include "rco_core/domain/model.hpp"
#include "rco_core/domain/validation.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iterator>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace rco_core::domain {
namespace {

constexpr std::array<std::string_view, 8> kColumns{
    "frame_id", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw",
};

bool isCsvWhitespace(char character) {
  return character == ' ' || character == '\t' || character == '\r';
}

std::string_view trimCsvWhitespace(std::string_view text) {
  while (!text.empty() && isCsvWhitespace(text.front())) {
    text.remove_prefix(1U);
  }
  while (!text.empty() && isCsvWhitespace(text.back())) {
    text.remove_suffix(1U);
  }
  return text;
}

std::vector<std::string_view> splitRecords(std::string_view text) {
  std::vector<std::string_view> records;
  while (true) {
    const std::size_t separator = text.find('\n');
    records.push_back(text.substr(0U, separator));
    if (separator == std::string_view::npos) {
      break;
    }
    text.remove_prefix(separator + 1U);
  }
  return records;
}

std::vector<std::string_view> splitRecord(std::string_view record) {
  std::vector<std::string_view> fields;
  while (true) {
    const std::size_t separator = record.find(',');
    fields.push_back(trimCsvWhitespace(record.substr(0U, separator)));
    if (separator == std::string_view::npos) {
      break;
    }
    record.remove_prefix(separator + 1U);
  }
  return fields;
}

bool hasExpectedHeader(std::string_view record) {
  const auto fields = splitRecord(record);
  return fields.size() == kColumns.size() && std::ranges::equal(fields, kColumns);
}

bool parseFiniteDouble(std::string_view text, double& output) {
  text = trimCsvWhitespace(text);
  if (text.empty()) {
    return false;
  }

  const char* const begin = text.data();
  const char* const end = begin + text.size();
  const auto result = std::from_chars(begin, end, output, std::chars_format::general);
  return result.ec == std::errc{} && result.ptr == end && std::isfinite(output);
}

void appendValidationErrors(ValidationErrors& destination, ValidationErrors source,
                            const std::string& prefix) {
  for (auto& error : source) {
    error.path = prefix + error.path;
    destination.push_back(std::move(error));
  }
}

bool parseTargetRow(std::string_view record, std::size_t row_index, TargetPose& target,
                    ValidationErrors& errors) {
  const std::string row_path = ".rows[" + std::to_string(row_index) + "]";
  const auto fields = splitRecord(record);
  if (fields.size() != kColumns.size()) {
    errors.push_back({row_path, "must contain exactly 8 columns"});
    return false;
  }

  target.pose.frame_id = std::string(fields.front());
  const std::array<double*, 7> outputs{
      &target.pose.position.x,    &target.pose.position.y,    &target.pose.position.z,
      &target.pose.orientation.x, &target.pose.orientation.y, &target.pose.orientation.z,
      &target.pose.orientation.w,
  };

  bool numbers_valid = true;
  for (std::size_t column_index = 1U; column_index < kColumns.size(); ++column_index) {
    if (!parseFiniteDouble(fields.at(column_index), *outputs.at(column_index - 1U))) {
      errors.push_back(
          {row_path + "." + std::string(kColumns.at(column_index)), "must be a finite number"});
      numbers_valid = false;
    }
  }
  if (!numbers_valid) {
    return false;
  }

  ValidationErrors validation_errors = validate(target);
  const bool target_valid = validation_errors.empty();
  appendValidationErrors(errors, std::move(validation_errors), row_path);
  return target_valid;
}

} // namespace

TargetPathCsvParseResult parseTargetPathCsv(std::string_view csv_text) {
  TargetPathCsvParseResult result;
  if (trimCsvWhitespace(csv_text).empty()) {
    result.errors.push_back({".header", "missing required header"});
    return result;
  }

  const auto records = splitRecords(csv_text);
  if (!hasExpectedHeader(records.front())) {
    result.errors.push_back({".header", "must equal: frame_id,x_m,y_m,z_m,qx,qy,qz,qw"});
    return result;
  }

  std::vector<TargetPose> targets;
  std::size_t row_index = 0U;
  for (std::size_t record_index = 1U; record_index < records.size(); ++record_index) {
    const std::string_view record = records.at(record_index);
    if (trimCsvWhitespace(record).empty()) {
      continue;
    }

    TargetPose target;
    if (parseTargetRow(record, row_index, target, result.errors)) {
      targets.push_back(std::move(target));
    }
    ++row_index;
  }

  if (row_index == 0U) {
    result.errors.push_back({".rows", "must contain at least one target"});
  }
  if (result.errors.empty()) {
    result.value = std::move(targets);
  }
  return result;
}

TargetPathCsvParseResult loadTargetPathCsv(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    TargetPathCsvParseResult result;
    result.errors.push_back(
        {".file", "could not open target path CSV file: '" + path.string() + "'"});
    return result;
  }

  const std::string contents{std::istreambuf_iterator<char>(input),
                             std::istreambuf_iterator<char>()};
  if (input.bad()) {
    TargetPathCsvParseResult result;
    result.errors.push_back(
        {".file", "could not read target path CSV file: '" + path.string() + "'"});
    return result;
  }
  return parseTargetPathCsv(contents);
}

} // namespace rco_core::domain
