#pragma once

#include "weft/io/format.hpp"
#include "weft/io/params.hpp"
#include "weft/mesh.hpp"

#include <memory>
#include <span>
#include <string>
#include <vector>

namespace weft {
struct Model;
}

namespace weft::io {

// What a writer may consume. Mesh writers read `mesh` (+`model`, `solidFaces`);
// B-rep writers read `model->shape`. Each writer takes what it needs. Stage A
// defines the interfaces only — no writer factories are registered yet.
struct WriteInput {
    const PolyMesh* mesh = nullptr;                          // retopo result
    const Model* model = nullptr;                            // shape + metadata + normals
    const std::vector<std::vector<int>>* solidFaces = nullptr;  // per-solid face-id groups
};

struct Writer {
    virtual ~Writer() = default;
    virtual bool transfer(const WriteInput&) = 0;  // stash + validate pairing
    virtual bool writeFile(const std::string& path) = 0;
    virtual void applyParams(const ParamGroup&) {}
};

struct FactoryWriter {
    virtual ~FactoryWriter() = default;
    virtual std::span<const Format> formats() const = 0;
    virtual std::unique_ptr<Writer> create(Format) const = 0;
    virtual ParamGroup createParams(Format) const { return {}; }
};

}  // namespace weft::io
