#include "fixture_generator.hpp"

#include <iostream>
#include <optional>
#include <string>

namespace {

enum class Action { none, list, one, all, inspect };

struct Arguments final {
  Action action{Action::none};
  std::optional<std::string> fixture_id;
  std::optional<std::filesystem::path> input;
  std::optional<std::filesystem::path> output;
};

void print_usage(std::ostream& stream) {
  stream << "Usage:\n"
            "  cad_mesher_generate_fixtures --list\n"
            "  cad_mesher_generate_fixtures --fixture ID --output PATH\n"
            "  cad_mesher_generate_fixtures --all --output DIRECTORY\n"
            "  cad_mesher_generate_fixtures --inspect-step INPUT --output SUMMARY.json\n";
}

bool select_action(Arguments& arguments, const Action action, std::string& error) {
  if (arguments.action != Action::none) {
    error = "choose exactly one of --list, --fixture, --all, or --inspect-step";
    return false;
  }
  arguments.action = action;
  return true;
}

bool parse_arguments(const int argc, char* argv[], Arguments& arguments, std::string& error) {
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--list") {
      if (!select_action(arguments, Action::list, error)) {
        return false;
      }
      continue;
    }
    if (argument == "--all") {
      if (!select_action(arguments, Action::all, error)) {
        return false;
      }
      continue;
    }
    if (argument == "--fixture") {
      if (!select_action(arguments, Action::one, error)) {
        return false;
      }
      if (++index >= argc) {
        error = "--fixture requires an ID";
        return false;
      }
      arguments.fixture_id = argv[index];
      continue;
    }
    if (argument == "--inspect-step") {
      if (!select_action(arguments, Action::inspect, error)) {
        return false;
      }
      if (++index >= argc) {
        error = "--inspect-step requires an input path";
        return false;
      }
      arguments.input = std::filesystem::path(argv[index]);
      continue;
    }
    if (argument == "--output") {
      if (arguments.output.has_value()) {
        error = "--output may only be specified once";
        return false;
      }
      if (++index >= argc) {
        error = "--output requires a path";
        return false;
      }
      arguments.output = std::filesystem::path(argv[index]);
      continue;
    }
    error = "unknown argument: " + argument;
    return false;
  }

  if (arguments.action == Action::none) {
    error = "no action selected";
    return false;
  }
  if (arguments.action == Action::list) {
    if (arguments.output.has_value()) {
      error = "--list does not accept --output";
      return false;
    }
    return true;
  }
  if (!arguments.output.has_value()) {
    error = arguments.action == Action::inspect ? "--inspect-step requires --output"
                                                : "generation requires --output";
    return false;
  }
  return true;
}

} // namespace

int main(const int argc, char* argv[]) {
  Arguments arguments;
  std::string argument_error;
  if (!parse_arguments(argc, argv, arguments, argument_error)) {
    std::cerr << "error[arguments]: " << argument_error << '\n';
    print_usage(std::cerr);
    return 2;
  }

  if (arguments.action == Action::list) {
    for (const std::string_view fixture_id : cad_mesher::fixtures::fixture_ids()) {
      std::cout << fixture_id << '\n';
    }
    return 0;
  }

  cad_mesher::fixtures::GenerationFailure failure;
  bool succeeded = false;
  if (arguments.action == Action::all) {
    succeeded = cad_mesher::fixtures::generate_all(*arguments.output, failure);
  } else if (arguments.action == Action::inspect) {
    succeeded = cad_mesher::fixtures::inspect_step(*arguments.input, *arguments.output, failure);
  } else {
    succeeded =
        cad_mesher::fixtures::generate_fixture(*arguments.fixture_id, *arguments.output, failure);
  }
  if (!succeeded) {
    std::cerr << "error[" << failure.code << "]: " << failure.message << '\n';
    return 1;
  }
  return 0;
}
