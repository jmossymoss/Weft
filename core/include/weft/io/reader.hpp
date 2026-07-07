#pragma once

#include "weft/io/format.hpp"
#include "weft/io/params.hpp"
#include "weft/model.hpp"

#include <memory>
#include <span>
#include <string>

namespace weft::io {

// Two-phase like Mayo: readFile parses the file into reader state, transfer
// materializes a weft::Model. The seam between the two is where XCAF metadata
// extraction lives.
struct Reader {
    virtual ~Reader() = default;
    virtual bool readFile(const std::string& path) = 0;  // parse into reader state
    virtual Model transfer() = 0;                        // materialize -> Model
    virtual void applyParams(const ParamGroup&) {}       // no-op default (Stage A)
};

struct FactoryReader {
    virtual ~FactoryReader() = default;
    virtual std::span<const Format> formats() const = 0;  // span over static Format[]
    virtual std::unique_ptr<Reader> create(Format) const = 0;
    virtual ParamGroup createParams(Format) const { return {}; }
};

}  // namespace weft::io
