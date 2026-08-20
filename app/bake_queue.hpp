// Per-face bake queue for weft_app (Phase 1).
//
// Scheduling layer only: the single worker always calls weft::generate()
// with a frozen GenerationSettings snapshot (AD-1). Face ids drive
// fidelity overlays and latest-wins dedupe so rapid artist edits do not
// stack stale full regenerates.
//
#pragma once

#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/mesh.hpp"
#include "weft/model.hpp"
#include "weft/recipe.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace weft_app {

enum class FaceFidelity : uint8_t {
    LowPolyProxy = 0,  // showing last stable mesh / proxy while dirty
    Queued,            // pending bake with newer params
    Baking,            // worker is running generate() that includes this face
    HighFidelity,      // last adopt landed for this face
};

struct FaceBakeResult {
    uint64_t generation = 0;
    std::vector<uint32_t> triggerFaces;  // faces that prompted this run
    weft::PolyMesh mesh;
    weft::GenerationReport report;
    int opsApplied = 0;
    int opsFailed = 0;
    bool finalizeMesh = false;
    std::string error;
};

// Thread-safe FIFO with per-faceId pending overwrite + single worker.
class FaceBakeQueue {
public:
    FaceBakeQueue() = default;
    FaceBakeQueue(const FaceBakeQueue&) = delete;
    FaceBakeQueue& operator=(const FaceBakeQueue&) = delete;
    ~FaceBakeQueue() { stop(); }

    // Bind the live document. Model/Analysis must outlive the worker and
    // must not be replaced while busy() (join first on reload).
    void bind(const weft::Model* model, const weft::Analysis* analysis,
              weft::GenerationCache* cache);

    void start();
    void stop();  // join worker; discard pending; keep completed for poll

    // Snapshot full settings (+ optional ops). faceId 0 = model-wide bake
    // (defaults / load). Same faceId pending → overwrite with latest.
    void enqueue(uint32_t faceId, const weft::GenerationSettings& settings,
                 const std::vector<weft::ManualOp>& ops = {});

    // UI-thread only: mark a face Queued/LowPolyProxy without starting work
    // (used while a gesture is still debouncing).
    void noteQueued(uint32_t faceId);

    // Drop pending work (model reload). Does not stop the worker.
    void clearPending();

    // Non-blocking drain of finished runs (UI thread).
    std::vector<FaceBakeResult> pollCompleted();

    FaceFidelity fidelity(uint32_t faceId) const;
    std::vector<uint32_t> facesInState(FaceFidelity f) const;

    bool busy() const { return busy_.load(std::memory_order_acquire); }
    bool hasPending() const;
    int pendingDepth() const;
    uint64_t generation() const {
        return generation_.load(std::memory_order_relaxed);
    }

    // Progress mirrors GenerationSettings progress atomics for the overlay.
    std::atomic<int> progressFaces{0};
    std::atomic<int> progressTotal{-1};

private:
    struct Pending {
        uint32_t faceId = 0;
        uint64_t generation = 0;
        weft::GenerationSettings settings;
        std::vector<weft::ManualOp> ops;
    };

    void workerMain();
    bool waitForWork(Pending& out);  // false → shutdown
    void markFidelityLocked(uint32_t faceId, FaceFidelity f);
    void publish(FaceBakeResult&& r);

    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::thread worker_;
    bool running_ = false;
    bool stop_ = false;

    const weft::Model* model_ = nullptr;
    const weft::Analysis* analysis_ = nullptr;
    weft::GenerationCache* cache_ = nullptr;

    // Latest-wins map: at most one pending entry per faceId.
    std::unordered_map<uint32_t, Pending> pendingByFace_;
    // FIFO of faceIds present in pendingByFace_ (stable order for overlay).
    std::deque<uint32_t> pendingOrder_;

    std::deque<FaceBakeResult> completed_;
    std::unordered_map<uint32_t, FaceFidelity> fidelity_;
    std::unordered_set<uint32_t> bakingFaces_;

    std::atomic<bool> busy_{false};
    std::atomic<uint64_t> generation_{0};
};

}  // namespace weft_app
