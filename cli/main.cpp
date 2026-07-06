// weft — Phase 0 spike CLI.
// Proves the loop: STEP in → B-rep analysis → per-face controllable
// topology generation → OBJ out. See docs/PLAN.md §9.

#include "weft/analysis.hpp"
#include "weft/edit.hpp"
#include "weft/fixture.hpp"
#include "weft/mesh.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/export_fbx.hpp"
#include "weft/export_gltf.hpp"
#include "weft/recipe.hpp"
#include "weft/validate.hpp"

#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <map>
#include <string>
#include <vector>

namespace {

void usage() {
    std::printf(
        "weft — B-rep retopology core (phase 0)\n"
        "\n"
        "usage:\n"
        "  weft fixture <out.step> [--shape cylinder|box|cone|sphere|torus|\n"
        "                                    fillet|hole|demo|boss|notched|\n"
        "                                    slotted|barrel|bossfillet|ribbon|\n"
        "                                    ribbonnotch]\n"
        "      generate a test STEP file from OCCT primitives\n"
        "\n"
        "  weft inspect <in.step>\n"
        "      list B-rep faces (type, radius, neighbors) and edges\n"
        "      (convexity, dihedral angle)\n"
        "\n"
        "  weft validate <in.step> [mesh options]\n"
        "      bake-ready checks: watertightness, winding, degenerates,\n"
        "      chord deviation vs the live B-rep; exits 1 on leaks\n"
        "\n"
        "  weft mesh <in.step> -o <out.obj|out.glb> [options]\n"
        "      generate topology and export OBJ (groups carry face IDs)\n"
        "    --radial N        divisions around cylinders/caps (default 16)\n"
        "    --axial N         divisions along cylinder axes  (default 4)\n"
        "    --grid NxM        planar/parametric grid divisions (default 4x4)\n"
        "    --cap ngon|fan    cylinder cap style (default ngon)\n"
        "    --loops N         support loops across fillet/blend faces (default 3)\n"
        "    --hold F          cluster fillet loops toward the creases, 0..0.95\n"
        "    --rings N         concentric quad loops around holes/bosses in\n"
        "                      planar faces (default 2)\n"
        "    --validate        run bake-ready checks after meshing\n"
        "    --no-normals      skip exact CAD vertex normals in exports\n"
        "    --triangulate     ear-clip everything to triangles on export\n"
        "    --yup / --scale F Y-up + unit scale (engine spaces)\n"
        "    --lods F1,F2,...  one export per density factor (_lod0..),\n"
        "                      manual ops replay into every tier\n"
        "    --profile cad     the CAD n-gon profile: minimal flats + adaptive\n"
        "                      curvature counts + natural strips ('dense'\n"
        "                      restores grid flats)\n"
        "    --adaptive        curvature-driven border counts (the CAD profile;\n"
        "                      big arcs get more segments, straights get 1)\n"
"    --flat-quads      dense grids on flat faces too (default: flats\n"
        "                      are boundary n-gons/webs; quads go to curves)\n"
        "    --quads           route flat plates through the quad-fill grid\n"
        "    --pure-tris       keep fallback floors as raw triangles\n"
        "                      (they pair into quads by default)\n"
        "                      where quality allows (default: pure tris)\n"
        "    --chord T         fallback triangulation tolerance (default 0.1)\n"
        "    --weld MM         global weld tolerance in mm (default 1e-6);\n"
        "                      raise to close seams on sloppy CAD / off-curve\n"
        "                      fallback borders (per-face: --face ID:weld=MM)\n"
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
    std::vector<double> lods;
    weft::ObjExportOptions objOpts;
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
        else if (a == "--pure-tris") {
            gs.defaults.quadDominant = false;
            gs.defaults.pureTriFloor = true;
        }
        else if (a == "--quads") gs.defaults.quadDominant = true;
        else if (a == "--flat-quads") gs.defaults.minimal = false;
        else if (a == "--adaptive") gs.defaults.adaptive = true;
        else if (a == "--density") {
            // One dial re-budgets the whole model: scales every
            // solved count (adaptive ones too) before the group
            // solve, and fallback tolerances to match.
            gs.densityScale = std::stod(next());
        }
        else if (a == "--weld") {
            // Global weld tolerance (mm): how far apart coincident border
            // verts may sit and still fuse. Loosening it closes seams on
            // sloppy CAD / off-curve fallback borders.
            gs.weldTolerance = std::stod(next());
        }
        else if (a == "--profile") {
            std::string prof = next();
            if (prof == "cad") {
                // Handoff step 3: minimal (default) + adaptive + strips.
                gs.defaults.minimal = true;
                gs.defaults.adaptive = true;
                // Game topology: deviation relative to feature size, so
                // ring counts follow the ANGLE criterion at every scale.
                gs.defaults.relativeDeviation = true;
            } else if (prof == "dense") {
                gs.defaults.minimal = false;
            } else {
                throw std::runtime_error("unknown profile: " + prof);
            }
        }
        else if (a == "--validate") validate = true;
        else if (a == "--debug") weft::setGenerateDebugLog(stderr);
        else if (a == "--no-normals") noNormals = true;
        else if (a == "--triangulate") objOpts.triangulate = true;
        else if (a == "--yup") objOpts.yUp = true;
        else if (a == "--scale") objOpts.scale = std::stod(next());
        else if (a == "--lods") {
            std::string spec = next();
            size_t pos = 0;
            while (pos != std::string::npos) {
                size_t comma = spec.find(',', pos);
                lods.push_back(std::stod(spec.substr(pos, comma - pos)));
                pos = comma == std::string::npos ? comma : comma + 1;
            }
        }
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
    objOpts.objectNames = &model.solidNames;
    if (!noNormals) objOpts.model = &model;
    auto isGlb = [](const std::string& s2) {
        return (s2.size() > 4 && s2.compare(s2.size() - 4, 4, ".glb") == 0) ||
               (s2.size() > 5 && s2.compare(s2.size() - 5, 5, ".gltf") == 0);
    };
    auto isFbx = [](const std::string& s2) {
        return s2.size() > 4 && s2.compare(s2.size() - 4, 4, ".fbx") == 0;
    };
    auto exportMesh = [&](const weft::PolyMesh& m, const std::string& path) {
        if (isGlb(path)) {
            weft::writeGlb(m, path, noNormals ? nullptr : &model,
                           &analysis.solidFaces);
        } else if (isFbx(path)) {
            weft::FbxExportOptions fo;
            fo.triangulate = objOpts.triangulate;
            fo.yUp = objOpts.yUp;
            fo.scale = objOpts.scale;
            weft::writeFbx(m, path, fo);
        } else {
            weft::writeObj(m, path, &analysis.solidFaces, &objOpts);
        }
    };

    // LOD tiers: one control setup, one export per density factor.
    // Divisions scale with the factor, chord tolerance with 1/f^2, and
    // manual ops replay into every tier (they anchor to the CAD).
    if (!lods.empty() && !output.empty()) {
        int rc = 0;
        for (size_t li = 0; li < lods.size(); ++li) {
            const double f = std::max(0.05, lods[li]);
            weft::GenerationSettings scaled = gs;
            auto scaleSet = [&](weft::FaceMeshSettings& fs) {
                fs.radial = std::max(3, (int)std::lround(fs.radial * f));
                fs.axial = std::max(1, (int)std::lround(fs.axial * f));
                fs.gridU = std::max(1, (int)std::lround(fs.gridU * f));
                fs.gridV = std::max(1, (int)std::lround(fs.gridV * f));
                fs.filletLoops =
                    std::max(1, (int)std::lround(fs.filletLoops * f));
                fs.junctionRings =
                    std::max(1, (int)std::lround(fs.junctionRings * f));
                fs.chordTolerance /= f * f;
            };
            scaleSet(scaled.defaults);
            for (auto& [fid, fs] : scaled.perFace) scaleSet(fs);
            weft::PolyMesh lod = weft::generate(model, analysis, scaled);
            weft::applyOps(lod, model, recipe.ops);
            size_t dot = output.rfind('.');
            std::string lodPath =
                dot == std::string::npos
                    ? output + "_lod" + std::to_string(li)
                    : output.substr(0, dot) + "_lod" + std::to_string(li) +
                          output.substr(dot);
            exportMesh(lod, lodPath);
            std::printf("  lod%zu (x%.3g): %s — %zu polygons\n", li, f,
                        lodPath.c_str(), lod.polygonCount());
            if (validate) {
                weft::ValidationReport vr = weft::validateMesh(lod, &model);
                std::printf("%s", weft::formatReport(vr).c_str());
                if (!vr.watertight()) rc = 1;
            }
        }
        return rc;
    }

    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    weft::applyOps(mesh, model, recipe.ops);
    if (!output.empty()) {
        exportMesh(mesh, output);
        std::printf("%s -> %s\n", input.c_str(), output.c_str());
    } else {
        std::printf("%s\n", input.c_str());
    }
    std::printf("  %zu vertices, %zu polygons (%zu quads, %zu tris, %zu n-gons)\n",
                mesh.vertexCount(), mesh.polygonCount(), mesh.countQuads(),
                mesh.countTris(), mesh.countNgons());
    {
        const auto folded = weft::foldedPolys(model, mesh);
        size_t nf = 0;
        std::map<int, int> perFace;
        for (size_t p = 0; p < folded.size(); ++p) {
            if (!folded[p]) continue;
            ++nf;
            if (p < mesh.polygonFaceId.size()) {
                ++perFace[mesh.polygonFaceId[p]];
            }
        }
        if (nf) {
            std::printf("  %zu folded polygon(s); worst faces:", nf);
            std::vector<std::pair<int, int>> top(perFace.begin(),
                                                 perFace.end());
            std::sort(top.begin(), top.end(),
                      [](auto& a, auto& b) { return a.second > b.second; });
            for (size_t i = 0; i < top.size() && i < 6; ++i) {
                std::printf(" #%d(%d)", top[i].first, top[i].second);
            }
            std::printf("\n");
        }
    }
    if (validate) {
        weft::ValidationReport vr = weft::validateMesh(mesh, &model);
        std::printf("%s", weft::formatReport(vr).c_str());
        if (!vr.watertight()) return 1;
    }
    if (report.faceMesher.size() <= 48) {
        for (const auto& [fid, kind] : report.faceMesher) {
            std::printf("  face #%-3d %s\n", fid, weft::mesherKindName(kind));
        }
    } else {
        std::map<std::string, int> byKind;
        for (const auto& [fid, kind] : report.faceMesher) {
            ++byKind[weft::mesherKindName(kind)];
        }
        for (const auto& [name, count] : byKind) {
            std::printf("  %6d x %s\n", count, name.c_str());
        }
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
        if (cmd == "validate") return cmdMesh(args, /*validateOnly=*/true);
        usage();
        return 2;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
