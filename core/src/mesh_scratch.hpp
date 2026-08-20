// Thread-isolated scratchpads for parallel face meshing.
// No heap growth in the hot path once reserved: clear() keeps capacity.
// alignas(64) prevents false sharing when an array of workspaces is used.
//
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <new>
#include <thread>
#include <vector>

#include <gp_Pnt.hxx>

namespace weft::mesher_detail {

#if defined(__cpp_lib_hardware_interference_size)
constexpr std::size_t kCacheLine = std::hardware_destructive_interference_size;
#else
constexpr std::size_t kCacheLine = 64;
#endif

struct alignas(kCacheLine) MeshScratch {
    std::vector<uint32_t> verts;
    std::vector<gp_Pnt> pts;
    std::vector<std::array<int, 3>> tris;
    std::vector<std::array<double, 3>> normals;
    std::vector<uint32_t> polyScratch;
    // Per-thread progress (merged after the parallel wave — no shared atomic
    // in the inner loop).
    int facesDone = 0;

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

inline void warmMeshScratchPool(unsigned /*threadCount*/) {
    (void)threadMeshScratch();
}

}  // namespace weft::mesher_detail
