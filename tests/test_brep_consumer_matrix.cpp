// Scaffold for B-rep family×subclass matrix ctests.
// Wave A–F agents replace presence checks with fail-closed mesh/refuse
// assertions under the WEFT_*_MATRIX markers documented in
// docs/governance/brep-consumer-matrix.md. This binary must not call
// generateSecureMesh or implement consumer paths.

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace {

int failures = 0;

std::filesystem::path repoRoot() {
    return std::filesystem::path(__FILE__).parent_path().parent_path();
}

std::filesystem::path extractPath(const char* name) {
    return repoRoot() / "tests" / "fixtures" / "mp9_extracts" / name;
}

void requireExtract(const char* wave, const char* marker, const char* name) {
    const std::filesystem::path path = extractPath(name);
    if (!std::filesystem::exists(path)) {
        std::printf("FAIL %s %s missing extract %s\n", wave, marker, name);
        ++failures;
        return;
    }
    std::printf("%s extract=%s present=1\n", marker, name);
}

void scaffoldWaveA() {
    const char* marker = "WEFT_PLANE_MATRIX";
    std::printf("%s scaffold=presence host=planar_trim_assembly "
                "entry=testPlaneMatrix "
                "also=testG1Plane2732DigonRecovery,"
                "testG1Plane3605PerforatedRecovery,"
                "testG1Plane3821EllipseDensifyRecovery "
                "contract=allowCurvedUv=0\n",
                marker);
    for (const char* name :
         {"plane_multi.step", "plane_2732.step", "plane_3605.step",
          "plane_3821.step", "plane_1793.step"}) {
        requireExtract("A", marker, name);
    }
}

void scaffoldWaveB() {
    const char* marker = "WEFT_CYLINDER_MATRIX";
    std::printf("%s scaffold=presence host=cylinder_template "
                "entry=testCylinderMatrix "
                "also=testCyl24SplitRimExtract "
                "contract=relax=0\n",
                marker);
    for (const char* name :
         {"cylinder_band.step", "cylinder_complex.step",
          "cylinder_ellipse.step", "cylinder_ellipse_band.step", "cyl_24.step",
          "cyl_24_from_mp9.step", "cyl_filleted_slot_bore.step",
          "cyl_441.step"}) {
        requireExtract("B", marker, name);
    }
}

void scaffoldWaveC() {
    std::printf("WEFT_CONE_MATRIX scaffold=presence host=cone_template "
                "entry=testConeMatrix\n");
    for (const char* name : {"cone_apex.step", "cone_frustum.step"}) {
        requireExtract("C", "WEFT_CONE_MATRIX", name);
    }

    std::printf("WEFT_SPHERE_MATRIX scaffold=presence host=sphere_template "
                "entry=testSphereMatrix "
                "also=testSphericalCapWallHardCertification "
                "contract=capwall_refuse_not_drop\n");
    for (const char* name :
         {"sphere_cap_778.step", "sphere_cap_complex.step"}) {
        requireExtract("C", "WEFT_SPHERE_MATRIX", name);
    }

    std::printf("WEFT_TORUS_MATRIX scaffold=presence host=torus_template "
                "entry=testTorusMatrix "
                "also=testFullTorusBody,testTorusPreviewLodChordValid "
                "contract=face_116_in_matrix,face_115_orient_densify\n");
    requireExtract("C", "WEFT_TORUS_MATRIX", "face_116_torus.step");
    requireExtract("C", "WEFT_TORUS_MATRIX", "face_115_torus.step");
}

void scaffoldWaveD() {
    const char* mapped = "WEFT_MAPPED_MATRIX";
    const char* freeform = "WEFT_FREEFORM_MATRIX";
    std::printf("%s host=mapped_template "
                "entry=testMappedFreeformMatrix "
                "orient=face_33,face_103,face_107 "
                "extrusion=extrusion_quad fixture "
                "offset=offset_quad.step\n",
                mapped);
    std::printf("%s host=mapped_template "
                "entry=testMappedFreeformMatrix,testG3Mp9FreeformExtracts,"
                "testG3Mp9CrashFace136NoAv "
                "revolution_n_ne_4=uv_promote "
                "fixtures=extrusion_quad,revolution_ngon\n",
                freeform);
    for (const char* name :
         {"freeform_pent.step", "freeform_hex.step",
          "freeform_mapped_fallback.step", "bspline_139.step",
          "offset_quad.step", "face_33_orient.step", "face_103_orient.step",
          "face_107_orient.step", "crash_face_136_r0.step",
          "freeform_135.step"}) {
        requireExtract("D", freeform, name);
    }
}

void scaffoldWaveE() {
    std::printf("WEFT_ASSEMBLY_MATRIX scaffold=host_secure_meshing "
                "entry=testG5HardCertifiedAssembly,"
                "testAssemblyMatrixMultiComponent "
                "adversaries=certified_mesh "
                "track_m=inter_solid_vs_leak_light\n");
}

void scaffoldWaveF() {
    const char* marker = "WEFT_UNSUPPORTED_MATRIX";
    std::printf("%s scaffold=presence host=secure_meshing "
                "entry=testUnsupportedFamilyMatrix\n",
                marker);
    const std::filesystem::path root = repoRoot();
    const std::filesystem::path dir =
        root / "tests" / "fixtures" / "unsupported";
    for (const char* name :
         {"curve_hyperbola.step", "curve_parabola.step", "curve_offset.brep",
          "surface_kernel_specific.construction"}) {
        const std::filesystem::path path = dir / name;
        if (!std::filesystem::exists(path)) {
            std::printf("FAIL F %s missing fixture %s\n", marker, name);
            ++failures;
            continue;
        }
        std::printf("%s fixture=%s present=1\n", marker, name);
    }
}

void noteObsoleteDeferredPicker() {
    std::printf(
        "WEFT_MATRIX_NOTE obsolete_deferred_picker=ADR-0014_*_deferred "
        "vs recon UV-promote=freeform.uv_trim_candidate,"
        "sphere.uv_trim_candidate,mapped.four_sided_candidate; "
        "authority=docs/governance/brep-consumer-matrix.md\n");
}

}  // namespace

int main() {
    noteObsoleteDeferredPicker();
    scaffoldWaveA();
    scaffoldWaveB();
    scaffoldWaveC();
    scaffoldWaveD();
    scaffoldWaveE();
    scaffoldWaveF();
    if (failures == 0) {
        std::printf("brep consumer matrix scaffold checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d brep consumer matrix scaffold failure(s)\n", failures);
    return EXIT_FAILURE;
}
