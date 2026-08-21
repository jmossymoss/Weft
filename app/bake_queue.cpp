#include "bake_queue.hpp"

#include "weft/edit.hpp"
#include "weft/independent_mesh.hpp"

#include <algorithm>
#include <utility>

namespace weft_app {

void FaceBakeQueue::bind(const weft::Model* model,
                         const weft::Analysis* analysis,
                         weft::GenerationCache* cache) {
    std::lock_guard<std::mutex> lock(mu_);
    model_ = model;
    analysis_ = analysis;
    cache_ = cache;
}

void FaceBakeQueue::start() {
    std::lock_guard<std::mutex> lock(mu_);
    if (running_) return;
    stop_ = false;
    running_ = true;
    worker_ = std::thread([this] { workerMain(); });
}

void FaceBakeQueue::stop() {
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (!running_) return;
        stop_ = true;
        pendingByFace_.clear();
        pendingOrder_.clear();
        bakingFaces_.clear();
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    {
        std::lock_guard<std::mutex> lock(mu_);
        running_ = false;
        busy_.store(false, std::memory_order_release);
    }
}

void FaceBakeQueue::noteQueued(uint32_t faceId) {
    if (faceId == 0) return;
    std::lock_guard<std::mutex> lock(mu_);
    auto it = fidelity_.find(faceId);
    if (it != fidelity_.end() && it->second == FaceFidelity::Baking) {
        return;  // keep Baking until publish
    }
    markFidelityLocked(faceId, FaceFidelity::Queued);
}

void FaceBakeQueue::clearPending() {
    std::lock_guard<std::mutex> lock(mu_);
    pendingByFace_.clear();
    pendingOrder_.clear();
}

void FaceBakeQueue::enqueue(uint32_t faceId,
                            const weft::GenerationSettings& settings,
                            const std::vector<weft::ManualOp>& ops) {
    const uint64_t gen =
        generation_.fetch_add(1, std::memory_order_relaxed) + 1;
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (pendingByFace_.count(faceId) == 0) {
            pendingOrder_.push_back(faceId);
        }
        Pending p;
        p.faceId = faceId;
        p.generation = gen;
        p.settings = settings;
        p.ops = ops;
        pendingByFace_[faceId] = std::move(p);
        if (faceId != 0) {
            markFidelityLocked(faceId, FaceFidelity::Queued);
            // Stomp LowPolyProxy visual for the edited face immediately.
        }
        // Any in-flight bake is allowed to finish; the next waitForWork
        // pull will see the overwritten pending entry (latest params).
    }
    cv_.notify_one();
}

bool FaceBakeQueue::hasPending() const {
    std::lock_guard<std::mutex> lock(mu_);
    return !pendingByFace_.empty();
}

int FaceBakeQueue::pendingDepth() const {
    std::lock_guard<std::mutex> lock(mu_);
    return int(pendingByFace_.size());
}

std::vector<FaceBakeResult> FaceBakeQueue::pollCompleted() {
    std::vector<FaceBakeResult> out;
    std::lock_guard<std::mutex> lock(mu_);
    while (!completed_.empty()) {
        out.push_back(std::move(completed_.front()));
        completed_.pop_front();
    }
    return out;
}

FaceFidelity FaceBakeQueue::fidelity(uint32_t faceId) const {
    std::lock_guard<std::mutex> lock(mu_);
    auto it = fidelity_.find(faceId);
    if (it == fidelity_.end()) return FaceFidelity::HighFidelity;
    return it->second;
}

std::vector<uint32_t> FaceBakeQueue::facesInState(FaceFidelity f) const {
    std::lock_guard<std::mutex> lock(mu_);
    std::vector<uint32_t> out;
    for (const auto& [fid, state] : fidelity_) {
        if (state == f) out.push_back(fid);
    }
    std::sort(out.begin(), out.end());
    return out;
}

void FaceBakeQueue::markFidelityLocked(uint32_t faceId, FaceFidelity f) {
    fidelity_[faceId] = f;
}

void FaceBakeQueue::publish(FaceBakeResult&& r) {
    std::lock_guard<std::mutex> lock(mu_);
    for (uint32_t fid : bakingFaces_) {
        markFidelityLocked(fid, FaceFidelity::HighFidelity);
    }
    bakingFaces_.clear();
    // Triggers that were not remeshed still go HighFidelity if no error;
    // remeshedFaces from the report get the same treatment via triggers.
    if (r.error.empty()) {
        for (uint32_t fid : r.triggerFaces) {
            if (fid != 0) markFidelityLocked(fid, FaceFidelity::HighFidelity);
        }
        for (int fid : r.report.remeshedFaces) {
            if (fid > 0) {
                markFidelityLocked(uint32_t(fid), FaceFidelity::HighFidelity);
            }
        }
    }
    completed_.push_back(std::move(r));
}

bool FaceBakeQueue::waitForWork(Pending& out) {
    std::unique_lock<std::mutex> lock(mu_);
    cv_.wait(lock, [&] {
        return stop_ || !pendingByFace_.empty();
    });
    if (stop_) return false;
    // Coalesce: take the newest pending snapshot (highest generation)
    // and union all trigger face ids. One bake applies the latest
    // cumulative recipe; older pending entries for other faces are
    // absorbed so we never stack latent full regenerates.
    uint64_t bestGen = 0;
    uint32_t bestFace = 0;
    for (const auto& [fid, p] : pendingByFace_) {
        if (p.generation >= bestGen) {
            bestGen = p.generation;
            bestFace = fid;
        }
    }
    out = pendingByFace_[bestFace];
    std::vector<uint32_t> triggers;
    triggers.reserve(pendingByFace_.size());
    for (const auto& [fid, p] : pendingByFace_) {
        if (fid != 0) {
            triggers.push_back(fid);
            markFidelityLocked(fid, FaceFidelity::Baking);
            bakingFaces_.insert(fid);
        }
        (void)p;
    }
    // Stash triggers on settings via a side channel: encode in out by
    // replacing faceId list — we return them via bakingFaces_ already.
    // Clear the entire pending map (coalesced into this one run).
    pendingByFace_.clear();
    pendingOrder_.clear();
    // Keep out.faceId as the newest trigger for logging; triggers live in
    // bakingFaces_.
    (void)triggers;
    return true;
}

void FaceBakeQueue::workerMain() {
    while (true) {
        Pending job;
        if (!waitForWork(job)) break;

        busy_.store(true, std::memory_order_release);
        progressFaces.store(0, std::memory_order_relaxed);
        progressTotal.store(-1, std::memory_order_relaxed);

        FaceBakeResult result;
        result.generation = job.generation;
        {
            std::lock_guard<std::mutex> lock(mu_);
            result.triggerFaces.assign(bakingFaces_.begin(),
                                       bakingFaces_.end());
            std::sort(result.triggerFaces.begin(), result.triggerFaces.end());
        }

        const weft::Model* model = nullptr;
        const weft::Analysis* analysis = nullptr;
        weft::GenerationCache* cache = nullptr;
        {
            std::lock_guard<std::mutex> lock(mu_);
            model = model_;
            analysis = analysis_;
            cache = cache_;
        }

        try {
            if (!model || !analysis) {
                result.error = "bake queue: model not bound";
            } else {
                weft::GenerationSettings settings = job.settings;
                settings.progressFaces = &progressFaces;
                settings.progressTotal = &progressTotal;
                weft::GenerationReport report;
                weft::PolyMesh mesh =
                    settings.independentMesh
                        ? weft::meshIndependent(*model, *analysis, settings,
                                                &report)
                        : weft::generate(*model, *analysis, settings, &report,
                                         cache);
                weft::ApplyOpsReport ops =
                    weft::applyOps(mesh, *model, job.ops);
                result.opsApplied = ops.applied;
                result.opsFailed = ops.failed;
                result.finalizeMesh = settings.finalizeMesh;
                result.mesh = std::move(mesh);
                result.report = std::move(report);
            }
        } catch (const std::exception& e) {
            result.error = e.what();
        } catch (...) {
            result.error = "unknown exception";
        }

        // If newer work arrived while we ran, the result is still useful
        // as a stable intermediate (full-mesh adopt); pending will fire
        // another bake with the latest snapshot immediately after.
        publish(std::move(result));
        busy_.store(false, std::memory_order_release);
        // Wake in case stop() is waiting, and so UI can notice hasPending.
        cv_.notify_all();
    }
}

}  // namespace weft_app
