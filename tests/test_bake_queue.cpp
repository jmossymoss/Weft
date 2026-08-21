// Bake-queue: latest-wins dedupe, and pending faces stay Queued across
// an in-flight publish so GPU preview does not vanish mid-edit.
#include "../app/bake_queue.hpp"

#include "weft/analysis.hpp"
#include "weft/fixture.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <thread>

static int gFails = 0;
#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,     \
                         #cond);                                             \
            ++gFails;                                                        \
        }                                                                    \
    } while (0)

int main() {
    weft_app::FaceBakeQueue q;
    // Do not start the worker — we only exercise pending dedupe.
    weft::GenerationSettings s1, s2, s3;
    s1.defaults.radial = 8;
    s2.defaults.radial = 12;
    s3.defaults.radial = 16;

    q.enqueue(5, s1);
    CHECK(q.pendingDepth() == 1);
    q.enqueue(12, s2);
    CHECK(q.pendingDepth() == 2);
    q.enqueue(5, s3);  // stomp face 5
    CHECK(q.pendingDepth() == 2);

    q.noteQueued(7);
    CHECK(q.fidelity(7) == weft_app::FaceFidelity::Queued);
    CHECK(q.fidelity(5) == weft_app::FaceFidelity::Queued);

    q.clearPending();
    CHECK(q.pendingDepth() == 0);
    CHECK(!q.hasPending());

    // Worker path: enqueue a newer snapshot while the first generate() is
    // in flight. Between jobs (busy=false, pending=true) fidelity must
    // remain Queued — HighFidelity here is what hid the GPU overlay.
    {
        const std::string path =
            (std::filesystem::temp_directory_path() / "weft_bake_q.step")
                .string();
        weft::writeStep(weft::makeFixture("cylinder"), path);
        weft::Model model = weft::loadStep(path);
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationCache cache;
        weft_app::FaceBakeQueue live;
        live.bind(&model, &analysis, &cache);
        live.start();
        weft::GenerationSettings s1, s2;
        s1.finalizeMesh = false;
        s2.finalizeMesh = false;
        s1.defaults.radial = 8;
        s2.defaults.radial = 20;
        live.enqueue(1, s1);
        const auto t0 = std::chrono::steady_clock::now();
        while (!live.busy() && !live.hasPending()) {
            if (std::chrono::steady_clock::now() - t0 >
                std::chrono::seconds(5)) {
                std::fprintf(stderr, "FAIL bake queue never started\n");
                ++gFails;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        live.enqueue(1, s2);
        bool sawQueuedBetweenJobs = false;
        const auto t1 = std::chrono::steady_clock::now();
        while (live.busy() || live.hasPending()) {
            if (!live.busy() && live.hasPending()) {
                CHECK(live.fidelity(1) == weft_app::FaceFidelity::Queued);
                sawQueuedBetweenJobs = true;
            }
            if (std::chrono::steady_clock::now() - t1 >
                std::chrono::seconds(15)) {
                std::fprintf(stderr, "FAIL bake queue never drained\n");
                ++gFails;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        (void)sawQueuedBetweenJobs;
        auto done = live.pollCompleted();
        CHECK(!done.empty());
        CHECK(live.fidelity(1) == weft_app::FaceFidelity::HighFidelity);
        live.stop();
    }

    if (gFails) {
        std::fprintf(stderr, "%d FAILURE(S)\n", gFails);
        return 1;
    }
    std::printf("bake_queue dedupe: ok\n");
    return 0;
}
