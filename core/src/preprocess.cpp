#include "weft/preprocess.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GeomAbs_Shape.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Geom_Surface.hxx>
#include <Precision.hxx>
#include <ShapeBuild_ReShape.hxx>
#include <ShapeCustom.hxx>
#include <ShapeCustom_RestrictionParameters.hxx>
#include <ShapeCustom_Surface.hxx>
#include <ShapeFix_Face.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace weft {

PreprocessSettings preprocessSettingsFromEnv() {
    PreprocessSettings s;
    // Opt-in analytic recognition (default off — preserves watertightness).
    if (const char* v = std::getenv("WEFT_ANALYTIC_FIT_REL")) {
        const double r = std::atof(v);
        if (r > 0.0) {
            s.convertToAnalytical = true;
            s.analyticFitTolRel = r;
        }
    }
    if (const char* v = std::getenv("WEFT_ANALYTIC_FIT")) {
        const double a = std::atof(v);
        if (a > 0.0) {
            s.convertToAnalytical = true;
            s.analyticFitTol = a;
        }
    }
    if (std::getenv("WEFT_BSPLINE_RESTRICT") != nullptr) {
        s.restrictBSplines = true;
    }
    if (std::getenv("WEFT_FULL_HEAL") != nullptr) {
        s.forceFullHeal = true;
    }
    if (std::getenv("WEFT_NO_ANALYTIC_FIT") != nullptr) {
        s.convertToAnalytical = false;
    }
    return s;
}

static double modelDiagonalOf(const TopoDS_Shape& shape) {
    Bnd_Box bb;
    try {
        BRepBndLib::Add(shape, bb);
        if (bb.IsVoid()) return 0.0;
        double x0, y0, z0, x1, y1, z1;
        bb.Get(x0, y0, z0, x1, y1, z1);
        return gp_Pnt(x0, y0, z0).Distance(gp_Pnt(x1, y1, z1));
    } catch (const Standard_Failure&) {
        return 0.0;
    }
}

TopoDS_Shape applyAnalyticPreprocess(const TopoDS_Shape& shape,
                                     const PreprocessSettings& settings,
                                     Handle(BRepTools_History) outHist) {
    TopoDS_Shape result = shape;
    const bool profile = std::getenv("WEFT_PROFILE_IMPORT") != nullptr ||
                         std::getenv("WEFT_TIMINGS") != nullptr;
    auto t0 = std::chrono::steady_clock::now();
    auto mark = [&](const char* stage, int n = -1) {
        if (!profile) return;
        const auto now = std::chrono::steady_clock::now();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - t0)
                            .count();
        if (n >= 0) {
            std::fprintf(stderr, "preprocess: %-24s %6lld ms  (n=%d)\n", stage,
                         static_cast<long long>(ms), n);
        } else {
            std::fprintf(stderr, "preprocess: %-24s %6lld ms\n", stage,
                         static_cast<long long>(ms));
        }
        t0 = now;
    };

    if (settings.convertToAnalytical) {
        double tol = settings.analyticFitTol;
        if (tol <= 0.0) {
            const double diag = modelDiagonalOf(result);
            tol = std::max(1e-9, settings.analyticFitTolRel *
                                     (diag > 1e-9 ? diag : 1.0));
        }
        Handle(ShapeBuild_ReShape) context = new ShapeBuild_ReShape();
        int converted = 0, considered = 0;
        BRep_Builder builder;
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(result, TopAbs_FACE, faces);
        for (int i = 1; i <= faces.Extent(); ++i) {
            const TopoDS_Face face = TopoDS::Face(faces(i));
            try {
                BRepAdaptor_Surface ads(face, Standard_True);
                const GeomAbs_SurfaceType st = ads.GetType();
                if (st != GeomAbs_BSplineSurface &&
                    st != GeomAbs_BezierSurface) {
                    continue;
                }
                ++considered;
                TopLoc_Location loc;
                Handle(Geom_Surface) gs = BRep_Tool::Surface(face, loc);
                if (gs.IsNull()) continue;
                ShapeCustom_Surface scs(gs);
                Handle(Geom_Surface) analytic =
                    scs.ConvertToAnalytical(tol, Standard_False);
                if (analytic.IsNull() || scs.Gap() > tol) continue;

                // Copy face topology, swap surface, rebuild pcurves.
                BRepBuilderAPI_Copy copier(face, Standard_True, Standard_True);
                TopoDS_Face nf = TopoDS::Face(copier.Shape());
                builder.UpdateFace(nf, analytic, loc,
                                   BRep_Tool::Tolerance(face));
                try {
                    ShapeFix_Face fix(nf);
                    fix.SetPrecision(tol);
                    fix.Perform();
                    if (!fix.Face().IsNull()) nf = fix.Face();
                } catch (const Standard_Failure&) {
                    // Keep UpdateFace result; generate() may still demote.
                }
                if (!nf.IsNull() && !nf.IsSame(face)) {
                    context->Replace(face, nf);
                    ++converted;
                }
            } catch (const Standard_Failure&) {
                continue;
            }
        }
        if (converted > 0) {
            try {
                TopoDS_Shape applied = context->Apply(result);
                if (!applied.IsNull()) {
                    result = applied;
                    // ReShape history is not a full BRepTools_History merge
                    // for all OCCT versions; face ids are re-indexed at
                    // indexShape() anyway.
                }
            } catch (const Standard_Failure&) {
            }
        }
        mark("analytic convert", converted);
        if (profile) {
            std::fprintf(stderr,
                         "preprocess: analytic considered=%d converted=%d "
                         "tol=%.6g\n",
                         considered, converted, tol);
        }
        (void)outHist;
    }

    if (settings.restrictBSplines) {
        try {
            Handle(ShapeCustom_RestrictionParameters) params =
                new ShapeCustom_RestrictionParameters();
            TopoDS_Shape restricted = ShapeCustom::BSplineRestriction(
                result, settings.bsplineTol3d, settings.bsplineTol2d,
                settings.maxBSplineDegree, settings.maxBSplineSegments,
                GeomAbs_C1, GeomAbs_C1, Standard_True, Standard_True, params);
            if (!restricted.IsNull()) result = restricted;
        } catch (const Standard_Failure&) {
        }
        mark("bspline restrict");
    }

    return result;
}

}  // namespace weft
