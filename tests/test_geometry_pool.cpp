#include "weft/analysis.hpp"
#include "weft/fixture.hpp"
#include "weft/geometry_pool.hpp"
#include "weft/model.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <ElSLib.hxx>
#include <TopoDS.hxx>
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
    {
        const std::string path = "weft_geom_pool_cyl.step";
        weft::writeStep(weft::makeFixture("cylinder"), path);
        weft::Model model = weft::loadStep(path);
        weft::Analysis a = weft::analyze(model);
        CHECK(!a.geometry.empty());
        CHECK(!a.geometry.cylinders.empty());
        const auto& cyl = a.geometry.cylinders.front();
        gp_Pnt pFast, pRef;
        CHECK(a.geometry.value(cyl.faceId, 0.5 * (cyl.u0 + cyl.u1),
                                0.5 * (cyl.v0 + cyl.v1), pFast));
        ElSLib::CylinderD0(0.5 * (cyl.u0 + cyl.u1), 0.5 * (cyl.v0 + cyl.v1),
                           cyl.frame, cyl.radius, pRef);
        CHECK(pFast.Distance(pRef) < 1e-9);
    }
    {
        // BSpline slab — native de Boor must match OCCT adaptor.
        const std::string path = "weft_geom_pool_nurb.step";
        weft::writeStep(weft::makeFixture("bspline_slab"), path);
        weft::Model model = weft::loadStep(path);
        weft::Analysis a = weft::analyze(model);
        CHECK(!a.geometry.nurbs.empty());
        int checked = 0;
        for (const auto& n : a.geometry.nurbs) {
            BRepAdaptor_Surface surf(TopoDS::Face(model.faces(int(n.faceId))),
                                     Standard_True);
            for (int i = 0; i <= 4; ++i) {
                for (int j = 0; j <= 4; ++j) {
                    const double u =
                        n.u0 + (n.u1 - n.u0) * double(i) / 4.0;
                    const double v =
                        n.v0 + (n.v1 - n.v0) * double(j) / 4.0;
                    gp_Pnt pFast, pOcct = surf.Value(u, v);
                    CHECK(a.geometry.value(n.faceId, u, v, pFast));
                    const double d = pFast.Distance(pOcct);
                    if (d >= 1e-4) {
                        std::fprintf(stderr,
                                     "nurb mismatch f%d uv=(%.4f,%.4f) "
                                     "d=%.6g\n",
                                     n.faceId, u, v, d);
                    }
                    CHECK(d < 1e-4);
                    ++checked;
                }
            }
        }
        CHECK(checked >= 25);
        std::printf("geometry_pool: ok (cyl + nurb de Boor, samples=%d)\n",
                    checked);
    }
    if (gFails) {
        std::fprintf(stderr, "%d FAILURE(S)\n", gFails);
        return 1;
    }
    return 0;
}
