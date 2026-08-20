#include "weft/analysis.hpp"
#include "weft/fixture.hpp"
#include "weft/geometry_pool.hpp"
#include "weft/model.hpp"

#include <ElSLib.hxx>
#include <cstdio>
#include <cmath>

static int gFails = 0;
#define CHECK(c)                                                             \
    do {                                                                     \
        if (!(c)) {                                                          \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); \
            ++gFails;                                                        \
        }                                                                    \
    } while (0)

int main() {
    const std::string path = "weft_geom_pool_test.step";
    weft::writeStep(weft::makeFixture("cylinder"), path);
    weft::Model model = weft::loadStep(path);
    weft::Analysis a = weft::analyze(model);
    CHECK(!a.geometry.empty());
    CHECK(a.geometry.byFace.size() == size_t(model.faceCount()) + 1);
    // Cylinder fixture: expect at least one cylinder primitive.
    CHECK(!a.geometry.cylinders.empty());
    const auto& cyl = a.geometry.cylinders.front();
    gp_Pnt pOcct, pFast;
    // Sample mid of cylinder UV via pool.
    CHECK(a.geometry.value(cyl.faceId, 0.5 * (cyl.u0 + cyl.u1),
                            0.5 * (cyl.v0 + cyl.v1), pFast));
    // Cross-check ElSLib directly.
    ElSLib::CylinderD0(0.5 * (cyl.u0 + cyl.u1), 0.5 * (cyl.v0 + cyl.v1),
                       cyl.frame, cyl.radius, pOcct);
    CHECK(pFast.Distance(pOcct) < 1e-9);
    if (gFails) {
        std::fprintf(stderr, "%d FAILURE(S)\n", gFails);
        return 1;
    }
    std::printf("geometry_pool: ok (planes=%zu cyl=%zu nurbs=%zu)\n",
                a.geometry.planes.size(), a.geometry.cylinders.size(),
                a.geometry.nurbs.size());
    return 0;
}
