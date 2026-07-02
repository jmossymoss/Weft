// weft — Phase 0 spike CLI.
// Proves the loop: STEP in → B-rep analysis → per-face controllable
// topology generation → OBJ out. See docs/PLAN.md §9.

#include "weft/analysis.hpp"
#include "weft/fixture.hpp"
#include "weft/mesh.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/recipe.hpp"

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
        "  weft fixture <out.step> [--shape cylinder|box|cone|sphere|torus|demo|boss]\n"
        "      generate a test STEP file from OCCT primitives\n"
        "\n"
        "  weft inspect <in.step>\n"
        "      list B-rep faces (type, radius, neighbors) and edges\n"
        "      (convexity, dihedral angle)\n"
        "\n"
        "  weft mesh <in.step> -o <out.obj> [options]\n"
        "      generate topology and export OBJ (groups carry face IDs)\n"
        "    --radial N        divisions around cylinders/caps (default 16)\n"
        "    --axial N         divisions along cylinder axes  (default 4)\n"
        "    --grid NxM        planar/parametric grid divisions (default 4x4)\n"
        "    --cap ngon|fan    cylinder cap style (default ngon)\n"
        "    --chord T         fallback triangulation tolerance (default 0.1)\n"
        "    --face ID:k=v[,k=v...]\n"
        "                      per-face override, e.g. --face 1:radial=24,axial=2\n"
        "                      keys: radial, axial, gridu, gridv, cap, chord\n"
        "    --edge ID:N       pin an edge (and its density-matched group) to\n"
        "                      exactly N subdivisions\n"
        "    --recipe FILE     load settings from a saved recipe (flags given\n"
        "                      after it override; --face/--edge always apply)\n"
        "    --save-recipe FILE\n"
        "                      persist the final settings, keyed to CAD IDs\n"
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

int cmdMesh(const std::vector<std::string>& args) {
    if (args.empty()) { usage(); return 2; }
    std::string input = args[0];
    std::string output;
    std::string recipeOut;
    weft::GenerationSettings gs;
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
        else if (a == "--recipe") gs = weft::loadRecipe(next());
        else if (a == "--save-recipe") recipeOut = next();
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
    if (output.empty()) throw std::runtime_error("missing -o <out.obj>");
    for (const std::string& spec : faceSpecs) parseFaceOverride(gs, spec);
    if (!recipeOut.empty()) {
        weft::saveRecipe(gs, recipeOut);
        std::printf("saved recipe %s\n", recipeOut.c_str());
    }

    weft::Model model = weft::loadStep(input);
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    weft::writeObj(mesh, output);

    std::printf("%s -> %s\n", input.c_str(), output.c_str());
    std::printf("  %zu vertices, %zu polygons (%zu quads, %zu tris, %zu n-gons)\n",
                mesh.vertexCount(), mesh.polygonCount(), mesh.countQuads(),
                mesh.countTris(), mesh.countNgons());
    for (const auto& [fid, kind] : report.faceMesher) {
        std::printf("  face #%-3d %s\n", fid, weft::mesherKindName(kind));
    }
    if (!report.edgeDivisions.empty()) {
        std::printf("  density-matched edges:");
        for (const auto& [eid, div] : report.edgeDivisions) {
            std::printf(" #%d=%d", eid, div);
        }
        std::printf("\n");
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
        usage();
        return 2;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
