// Bake-queue unit test: latest-wins dedupe without running generate().
#include "../app/bake_queue.hpp"

#include <cstdio>
#include <cstdlib>

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

    if (gFails) {
        std::fprintf(stderr, "%d FAILURE(S)\n", gFails);
        return 1;
    }
    std::printf("bake_queue dedupe: ok\n");
    return 0;
}
