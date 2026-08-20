// Optional STEP preprocess: heal/sew (already in healWithHistory) plus
// tolerance-gated analytic recognition and BSpline restriction.
//
// Default: both experimental stages OFF (watertightness preserved).
// Opt-in via environment:
//   WEFT_ANALYTIC_FIT_REL=0.001   — ConvertToAnalytical (rel. to model diag)
//   WEFT_ANALYTIC_FIT=0.1         — absolute mm tolerance
//   WEFT_BSPLINE_RESTRICT=1       — degree/segment clamp on freeform
//   WEFT_FULL_HEAL=1              — ShapeFix even on large assemblies
//   WEFT_PROFILE_IMPORT=1         — print preprocess stage timings
//
// A/B on mp9_Edited (indicative): BSpline restrict can cut cold generate
// ~16s→~4.5s but currently fails watertight/floor gates — keep opt-in
// until pcurve rebuild + corpus gates are green.
//
#pragma once

#include <TopoDS_Shape.hxx>
#include <Standard_Handle.hxx>
#include <BRepTools_History.hxx>

namespace weft {

struct PreprocessSettings {
    // Opt-in: ConvertToAnalytical on BSpline/Bezier faces. Default off —
    // OCCT warns PCurves must be redone; aggressive fits reopen seams on
    // mp9. Enable via WEFT_ANALYTIC_FIT_REL / WEFT_ANALYTIC_FIT.
    bool convertToAnalytical = false;
    double analyticFitTol = 0.0;      // mm; 0 → use relative
    double analyticFitTolRel = 1e-3;   // fraction of model diagonal

    // Optional degree/segment clamp on remaining freeform BSplines.
    bool restrictBSplines = false;
    double bsplineTol3d = 1e-3;
    double bsplineTol2d = 1e-4;
    int maxBSplineDegree = 5;
    int maxBSplineSegments = 64;

    // Force ShapeFix on large models (otherwise budget-skipped above 2000 faces).
    bool forceFullHeal = false;
};

// Defaults from env (optional overrides):
//   WEFT_ANALYTIC_FIT_REL=0.001
//   WEFT_BSPLINE_RESTRICT=1
//   WEFT_FULL_HEAL=1
PreprocessSettings preprocessSettingsFromEnv();

// Applies analytic conversion (+ optional BSpline restriction) to `shape`.
// Merges history into outHist when non-null. Best-effort: failures keep
// the prior shape.
TopoDS_Shape applyAnalyticPreprocess(const TopoDS_Shape& shape,
                                     const PreprocessSettings& settings,
                                     Handle(BRepTools_History) outHist);

}  // namespace weft
