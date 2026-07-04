// weft — Phase 0 spike CLI.
// Proves the loop: STEP in → B-rep analysis → per-face controllable
// topology generation → OBJ out. See docs/PLAN.md §9.

#include "weft/analysis.hpp"
#include "weft/edit.hpp"
#include "weft/fixture.hpp"
#include "weft/mesh.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/export_gltf.hpp"
#include "weft/recipe.hpp"
#include "weft/validate.hpp"

#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void usage() {
    std::printf(
        "weft — B-rep retopology core (phase 0)\n"
        "\n"
        "usage:\n"
        "  weft fixture <out.step> [--shape cylinder|box|cone|sphere|torus|\n"
        "                                    fillet|hole|notch|demo|boss]\n"
        "      generate a test STEP file from OCCT primitives\n"
        "\n"
        "  weft inspect <in.step>\n"
        "      list B-rep faces (type, radius, neighbors) and edges\n"
        "      (convexity, dihedral angle)\n"
        "\n"
        "  weft validate <in.step> [mesh options]\n"
        "      generate topology with the same options as `mesh` and run\n"
        "      bake-ready checks: watertightness (open/non-manifold edges),\n"
        "      winding consistency, degenerate/sliver polygons, and chord\n"
        "      deviation vs. the live B-rep. Exits 1 if the mesh leaks.\n"
        "\n"
        "  weft mesh <in.step> -o <out.obj> [options]\n"
        "      generate topology and export by extension: .obj (groups carry\n"
        "      face IDs) or .glb (binary glTF, _WEFT_FACE_ID attribute)\n"
        "    --validate        run the bake-ready checks after meshing\n"
        "    --no-normals      skip exact CAD vertex normals in the OBJ\n"
        "                      (default: every corner carries its face's\n"
        "                      true surface normal — sharp edges split,\n"
        "                      fillets shade smooth, ready for baking)\n"
        "    --radial N        divisions around cylinders/caps (default 16)\n"
        "    --axial N         divisions along cylinder axes  (default 4)\n"
        "    --grid NxM        planar/parametric grid divisions (default 4x4)\n"
        "    --cap ngon|fan    cylinder cap style (default ngon)\n"
        "    --loops N         support loops across fillet/blend faces (default 3)\n"
        "    --hold F          cluster fillet loops toward the creases, 0..0.95\n"
        "    --rings N         concentric quad loops around holes/bosses in\n"
        "                      planar faces (default 2)\n"
        "    --refine          EXPERIMENTAL: refine triangulated interiors toward\n"
        "                      the border spacing before quad pairing\n"
        "    --pure-tris       disable quad pairing on fallback-triangulated\n"
        "                      faces (default: quad-dominant)\n"
        "    --chord T         fallback triangulation tolerance (default 0.1)\n"
        "    --face ID:k=v[,k=v...]\n"
        "                      per-face override, e.g. --face 1:radial=24,axial=2\n"
        "                      keys: radial, axial, gridu, gridv, cap, chord\n"
        "    --edge ID:N       pin an edge (and its density-matched group) to\n"
        "                      exactly N subdivisions\n"
        "    --op-loop ID:u,v,t\n"
        "                      manual edit: insert an edge loop crossing the\n"
        "                      mesh edge nearest (u,v) on B-rep face ID, at\n"
        "                      fraction t; anchored to the CAD, so it replays\n"
        "                      after density changes\n"
        "    --recipe FILE     load settings + manual ops from a saved recipe\n"
        "                      (flags given after it override)\n"
        "    --save-recipe FILE\n"
        "                      persist settings AND manual ops, keyed to CAD IDs\n"
        "\n"
        "  Divisions are density-matched: edges shared between parametric\n"
        "  faces resolve to one count (max of the faces' proposals), so\n"
        "  neighbours meet vertex-for-vertex.\n");
}

// "ID:k=v,k=v" → per-face override starting from the current defaults.
void parseFaceOverride(weft::GenerationSettings& gs, const std::string& spec) {
    size_t colon = spec.find(':');
    if (colon == std::string::npos) {
        throw std::runtime_error("--face expects ID:key=val[,key=val], got " + spec);
    }
    int faceId = std::stoi(spec.substr(0, colon));
    weft::FaceMeshSettings s = gs.defaults;
    weft::applySettingsList(s, spec.substr(colon + 1));
    gs.perFace[faceId] = s;
}

int cmdFixture(const std::vector<std::string>& args) {
    if (args.empty()) { usage(); return 2; }
    std::string shape = "demo";
    for (size_t i = 1; i < args.size(); ++i) {
        if (args[i] == "--shape" && i + 1 < args.size()) shape = args[++i];
    }
    weft::writeStep(weft::makeFixture(shape), args[0]);
    std::printf("wrote %s (%s)\n", args[0].c_str(), shape.c_str());
    return 0;
}

int cmdInspect(const std::vector<std::string>& args) {
    if (args.empty()) { usage(); return 2; }
    weft::Model model = weft::loadStep(args[0]);
    weft::Analysis a = weft::analyze(model);

    std::printf("%s: %d faces, %d edges\n\n", args[0].c_str(),
                model.faceCount(), model.edgeCount());
    std::printf("faces:\n");
    for (const weft::FaceInfo& f : a.faces) {
        std::printf("  #%-3d %-10s", f.id, weft::surfaceTypeName(f.type));
        if (f.radius > 0) std::printf(" r=%-8.3f", f.radius);
        else std::printf("           ");
        std::printf("%s", f.isFillet ? " fillet " : f.isHole ? " hole   "
                                                            : "        ");
        std::printf(" edges=%zu neighbors=[", f.edgeIds.size());
        for (size_t i = 0; i < f.neighborFaceIds.size(); ++i) {
            std::printf("%s%d", i ? "," : "", f.neighborFaceIds[i]);
        }
        std::printf("]\n");
    }
    std::printf("\nedges:\n");
    for (const weft::EdgeInfo& e : a.edges) {
        std::printf("  #%-3d %-9s dihedral=%6.2f deg  faces=[", e.id,
                    weft::edgeConvexityName(e.convexity), e.dihedralDeg);
        for (size_t i = 0; i < e.faceIds.size(); ++i) {
            std::printf("%s%d", i ? "," : "", e.faceIds[i]);
        }
        std::printf("]\n");
    }
    return 0;
}

int cmdMesh(const std::vector<std::string>& args, bool validateOnly = false) {
    if (args.empty()) { usage(); return 2; }
    std::string input = args[0];
    std::string output;
    std::string recipeOut;
    bool validate = validateOnly;
    bool noNormals = false;
    weft::Recipe recipe;
    weft::GenerationSettings& gs = recipe.settings;
    std::vector<std::string> faceSpecs;

    for (size_t i = 1; i < args.size(); ++i) {
        const std::string& a = args[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= args.size()) throw std::runtime_error(a + " needs a value");
            return args[++i];
        };
        if (a == "-o" || a == "--output") output = next();
        else if (a == "--radial") gs.defaults.radial = std::stoi(next());
        else if (a == "--axial") gs.defaults.axial = std::stoi(next());
        else if (a == "--chord") gs.defaults.chordTolerance = std::stod(next());
        else if (a == "--cap") weft::applySetting(gs.defaults, "cap", next());
        else if (a == "--loops") gs.defaults.filletLoops = std::stoi(next());
        else if (a == "--hold") gs.defaults.filletHold = std::stod(next());
        else if (a == "--rings") gs.defaults.junctionRings = std::stoi(next());
        else if (a == "--pure-tris") gs.defaults.quadDominant = false;
        else if (a == "--refine") gs.defaults.interiorRefine = true;
        else if (a == "--validate") validate = true;
        else if (a == "--no-normals") noNormals = true;
        else if (a == "--recipe") recipe = weft::loadRecipe(next());
        else if (a == "--save-recipe") recipeOut = next();
        else if (a == "--op-loop") {
            // faceId:u,v,t — insert a loop crossing the mesh edge nearest
            // to (u,v) on that B-rep face, at fraction t along the edge.
            std::string spec = next();
            size_t colon = spec.find(':');
            if (colon == std::string::npos) {
                throw std::runtime_error("--op-loop expects ID:u,v,t, got " +
                                         spec);
            }
            weft::ManualOp op;
            op.faceId = std::stoi(spec.substr(0, colon));
            std::string rest = spec.substr(colon + 1);
            size_t c1 = rest.find(',');
            size_t c2 = rest.find(',', c1 + 1);
            if (c1 == std::string::npos || c2 == std::string::npos) {
                throw std::runtime_error("--op-loop expects ID:u,v,t, got " +
                                         spec);
            }
            op.u = std::stod(rest.substr(0, c1));
            op.v = std::stod(rest.substr(c1 + 1, c2 - c1 - 1));
            op.t = std::stod(rest.substr(c2 + 1));
            recipe.ops.push_back(op);
        }
        else if (a == "--grid") {
            std::string g = next();
            size_t x = g.find('x');
            gs.defaults.gridU = std::stoi(g.substr(0, x));
            gs.defaults.gridV =
                x == std::string::npos ? gs.defaults.gridU
                                       : std::stoi(g.substr(x + 1));
        } else if (a == "--face") {
            faceSpecs.push_back(next());  // parsed after defaults are final
        } else if (a == "--edge") {
            std::string spec = next();
            size_t colon = spec.find(':');
            if (colon == std::string::npos) {
                throw std::runtime_error("--edge expects ID:N, got " + spec);
            }
            gs.perEdge[std::stoi(spec.substr(0, colon))] =
                std::stoi(spec.substr(colon + 1));
        } else {
            throw std::runtime_error("unknown option: " + a);
        }
    }
    if (output.empty() && !validateOnly) {
        throw std::runtime_error("missing -o <out.obj>");
    }
    for (const std::string& spec : faceSpecs) parseFaceOverride(gs, spec);
    if (!recipeOut.empty()) {
        weft::saveRecipe(recipe, recipeOut);
        std::printf("saved recipe %s\n", recipeOut.c_str());
    }

    weft::Model model = weft::loadStep(input);
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    weft::applyOps(mesh, model, recipe.ops);
    if (!output.empty()) {
        auto endsWith = [&](const char* suffix) {
            size_t n = std::strlen(suffix);
            return output.size() >= n &&
                   output.compare(output.size() - n, n, suffix) == 0;
        };
        if (endsWith(".glb") || endsWith(".gltf")) {
            weft::writeGlb(mesh, output, noNormals ? nullptr : &model);
        } else {
            weft::writeObj(mesh, output, noNormals ? nullptr : &model);
        }
        std::printf("%s -> %s\n", input.c_str(), output.c_str());
    } else {
        std::printf("%s\n", input.c_str());
    }
    std::printf("  %zu vertices, %zu polygons (%zu quads, %zu tris, %zu n-gons)\n",
                mesh.vertexCount(), mesh.polygonCount(), mesh.countQuads(),
                mesh.countTris(), mesh.countNgons());
    if (validate) {
        weft::ValidationReport vr = weft::validateMesh(mesh, &model);
        std::printf("%s", weft::formatReport(vr).c_str());
        if (!vr.watertight()) return 1;
    }
    if (report.faceMesher.size() <= 48) {
        for (const auto& [fid, kind] : report.faceMesher) {
            std::printf("  face #%-3d %s\n", fid, weft::mesherKindName(kind));
        }
    } else {  // dense assemblies: per-strategy totals instead of a face list
        std::map<std::string, int> byKind;
        for (const auto& [fid, kind] : report.faceMesher) {
            ++byKind[weft::mesherKindName(kind)];
        }
        for (const auto& [name, count] : byKind) {
            std::printf("  %6d x %s\n", count, name.c_str());
        }
    }
    if (!report.edgeDivisions.empty() && report.edgeDivisions.size() <= 64) {
        std::printf("  density-matched edges:");
        for (const auto& [eid, div] : report.edgeDivisions) {
            std::printf(" #%d=%d", eid, div);
        }
        std::printf("\n");
    } else if (!report.edgeDivisions.empty()) {
        std::printf("  density-matched edges: %zu\n", report.edgeDivisions.size());
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) { usage(); return 2; }
    std::string cmd = argv[1];
    std::vector<std::string> args(argv + 2, argv + argc);
    try {
        if (cmd == "fixture") return cmdFixture(args);
        if (cmd == "inspect") return cmdInspect(args);
        if (cmd == "mesh") return cmdMesh(args);
        if (cmd == "validate") return cmdMesh(args, /*validateOnly=*/true);
        usage();
        return 2;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
