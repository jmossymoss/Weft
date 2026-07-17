#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace cad_mesher::fixtures {

struct GenerationFailure final {
  std::string code;
  std::string message;
};

[[nodiscard]] const std::vector<std::string_view>& fixture_ids();

[[nodiscard]] bool generate_fixture(std::string_view fixture_id,
                                    const std::filesystem::path& output,
                                    GenerationFailure& failure);

[[nodiscard]] bool generate_all(const std::filesystem::path& output_directory,
                                GenerationFailure& failure);

[[nodiscard]] bool inspect_step(const std::filesystem::path& input,
                                const std::filesystem::path& output, GenerationFailure& failure);

} // namespace cad_mesher::fixtures
