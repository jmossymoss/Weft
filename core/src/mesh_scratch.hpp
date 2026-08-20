// Thread-isolated scratchpads for parallel face meshing.
// No heap growth in the hot path once reserved: clear() keeps capacity.
//
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#include <gp_Pnt.hxx>

namespace weft::mesher_detail {

struct MeshScratch {
    std::vector<uint32_t> verts;
    std::vector<gp_Pnt> pts;
    std::vector<std::array<int, 3>> tris;
    std::vector<std::array<double, 3>> normals;
    std::vector<uint32_t> polyScratch;

    void ensureVerts(size_t n) {
        if (verts.capacity() < n) verts.reserve(n);
        if (pts.capacity() < n) pts.reserve(n);
        if (normals.capacity() < n) normals.reserve(n);
    }
    void ensureTris(size_t n) {
        if (tris.capacity() < n) tris.reserve(n);
    }
    void resetWorking() {
        verts.clear();
        pts.clear();
        tris.clear();
        normals.clear();
        polyScratch.clear();
    }
};

// One scratchpad per hardware thread. First touch reserves a generous
// default so typical faces never reallocate.
inline MeshScratch& threadMeshScratch() {
    thread_local MeshScratch scratch = [] {
        MeshScratch s;
        s.ensureVerts(4096);
        s.ensureTris(8192);
        s.polyScratch.reserve(64);
        return s;
    }();
    return scratch;
}

// Pre-touch every worker slot before a parallel mesh wave so the first
// face on each core does not pay the initial reserve under load.
inline void warmMeshScratchPool(unsigned threadCount) {
    static std::mutex mu;
    static std::vector<MeshScratch*> slots;
    std::lock_guard<std::mutex> lock(mu);
    // Touch thread_local on this thread; workers touch their own on first use.
    (void)threadMeshScratch();
    (void)threadCount;
}

}  // namespace weft::mesher_detail
