// weft — Phase 0 spike CLI.
// Proves the loop: STEP in → B-rep analysis → per-face controllable
// topology generation → OBJ out. See docs/PLAN.md §9.

#include "weft/analysis.hpp"
#include "weft/compiler.hpp"
#include "weft/edit.hpp"
#include "weft/fixture.hpp"
#include "weft/mesh.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/export_fbx.hpp"
#include "weft/export_gltf.hpp"
#include "weft/io/system.hpp"
#include "weft/recipe.hpp"
#include "weft/secure_meshing.hpp"
#include "weft/secure_recipe.hpp"
#include "weft/secure_reconnaissance.hpp"
#include "weft/validate.hpp"

#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <set>
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
        "                                    slotted|barrel|drilled|bossfillet|ribbon|\n"
        "                                    ribbonnotch|hairline|canrev|slitdrill|\n"
        "                                    microedge|filletslot|torture]\n"
        "      generate a test STEP file from OCCT primitives\n"
        "\n"
        "  weft inspect <in.step>\n"
        "      list B-rep faces (type, radius, neighbors) and edges\n"
        "      (convexity, dihedral angle)\n"
        "\n"
        "  weft inventory <in.step> [options]\n"
        "      secure import + reconnaissance gap report for large models\n"
        "      (MP9). Prints family/support/strategy/trim blocker buckets and\n"
        "      optionally probes per-face meshing refusals.\n"
        "    --repair conservative|compatibility\n"
        "    --gap-tsv FILE    write one row per face/unsupported curve\n"
        "    --probe-limit N   mesh up to N sample faces per surface family\n"
        "                      (default 0 = no probes); logs WEFT_PROBE lines\n"
        "\n"
        "  weft extract <in.step> --faces ID[,ID...] --rings N -o <out.step>\n"
        "      write selected source faces plus N adjacency rings as a small\n"
        "      regression STEP file (default: one neighbor ring)\n"
        "\n"
        "  weft validate <in.step> [mesh options]\n"
        "      bake-ready checks: watertightness, winding, degenerates,\n"
        "      chord deviation vs the live B-rep; exits 1 on leaks\n"
        "\n"
        "  weft convert <in> -o <out>\n"
        "      import then export with no retopo: B-rep->B-rep serializes the\n"
        "      shape; STEP->mesh delegates to the certified secure pipeline\n"
        "\n"
        "  weft sweep <in.step> [--profile cad] [--radials 8,16,24,...]\n"
        "      secure global density/property harness: every count must produce\n"
        "      a complete deterministic certified result; exits 1 on failure\n"
        "\n"
        "  weft cache-check <in.step> --face ID:key=value[,key=value...] [--preview]\n"
        "      reserved for recipe-v2 secure dependency-cache verification;\n"
        "      currently refuses by name instead of using the legacy cache\n"
        "\n"
        "  weft mesh <in.step> -o <out.obj|out.glb|out.stl|out.fbx> [options]\n"
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
        "    --preview         export the connected interactive mesh before\n"
        "                      border/stitch/final cleanup passes\n"
        "    --no-normals      skip exact CAD vertex normals in exports\n"
        "    --triangulate     ear-clip everything to triangles on export\n"
        "    --yup / --scale F Y-up + unit scale (engine spaces)\n"
        "    --lods F1,F2,...  one export per density factor (_lod0..),\n"
        "                      manual ops replay into every tier\n"
        "    --profile cad     the CAD n-gon profile: minimal flats + adaptive\n"
        "                      curvature counts + natural strips ('dense'\n"
        "                      restores grid flats)\n"
        "    --repair conservative|compatibility\n"
        "                      audited working-copy repair profile\n"
        "    --mesh-report FILE write the complete secure validation coverage\n"
        "    --pipeline legacy|compiler\n"
        "                      accepted as ignored migration input with warning\n"
        "    --adaptive        curvature-driven border counts (the CAD profile;\n"
        "                      big arcs get more segments, straights get 1)\n"
"    --flat-quads      dense grids on flat faces too (default: flats\n"
        "                      are boundary n-gons/webs; quads go to curves)\n"
        "    --quads           pair exact-border fallback triangles into quads\n"
        "    --pure-tris       keep fallback floors as raw triangles\n"
        "                      (they pair into quads by default)\n"
        "                      where quality allows (default: pure tris)\n"
        "    --chord T         fallback triangulation tolerance (default 0.1)\n"
        "    --progress        stderr stage/face progress (WEFT_PROGRESS ...)\n"
        "    --inventory-refusals\n"
        "                      aggregate unsupported curve/surface families\n"
        "                      from reconnaissance and refuse (no mesh)\n"
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
        "    --recipe FILE     load recipe v2, or migrate v1 with conflicts\n"
        "                      blocking export; later flags still override\n"
        "    --save-recipe FILE\n"
        "                      save only recipe v2: source refs + fingerprints\n"
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
    int compilerFace = 0;
    for (size_t i = 1; i + 1 < args.size(); ++i) {
        if (args[i] == "--compiler-face") {
            compilerFace = std::stoi(args[++i]);
        }
    }
    const weft::ImportedModel imported = weft::importStepSecure(args[0]);
    const weft::Model& model = imported.source->snapshot.model;
    weft::Analysis a = weft::analyze(model);

    if (compilerFace > 0) {
        throw std::runtime_error(
            "secure inspect retired --compiler-face; use source topology "
            "and reconnaissance reports instead");
    }
    if (compilerFace > 0) {
        if (compilerFace > model.faceCount()) {
            throw std::runtime_error("compiler face id is out of range");
        }
        const weft::CompilerPlan plan = weft::planPrimitiveAware(model, a);
        const weft::BrepFaceNode& face =
            plan.graph.faces[compilerFace - 1];
        const weft::PatchPlan& patch = plan.patches[compilerFace - 1];
        std::printf("compiler face #%d: %s, patch=%s, fallback=%s, wires=%zu\n",
                    compilerFace, weft::surfaceTypeName(face.surface.type),
                    weft::patchKindName(patch.kind),
                    weft::patchFallbackReasonName(patch.fallbackReason),
                    face.wires.size());
        for (size_t wi = 0; wi < face.wires.size(); ++wi) {
            std::printf("  wire %zu: %zu coedges\n", wi,
                        face.wires[wi].size());
            for (int coedgeId : face.wires[wi]) {
                const weft::BrepCoedgeNode& coedge =
                    plan.graph.coedges[coedgeId - 1];
                const weft::BrepEdgeNode& edge =
                    plan.graph.edges[coedge.edgeId - 1];
                double u0 = std::numeric_limits<double>::infinity();
                double u1 = -u0, v0 = u0, v1 = -u0;
                for (const auto& uv : coedge.uv) {
                    if (!std::isfinite(uv[0]) || !std::isfinite(uv[1])) continue;
                    u0 = std::min(u0, uv[0]);
                    u1 = std::max(u1, uv[0]);
                    v0 = std::min(v0, uv[1]);
                    v1 = std::max(v1, uv[1]);
                }
                std::printf("    c%-5d e%-5d %s %s n=%zu uv=[%.6g,%.6g]x[%.6g,%.6g]\n",
                            coedge.id, edge.id,
                            weft::curveTypeName(edge.curve),
                            weft::semanticEdgeTypeName(edge.semantic),
                            coedge.sampleIds.size(), u0, u1, v0, v1);
            }
        }
        return 0;
    }

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

std::set<int> parseIdList(const std::string& value) {
    std::set<int> ids;
    for (size_t pos = 0; pos < value.size();) {
        size_t comma = value.find(',', pos);
        if (comma == std::string::npos) comma = value.size();
        const std::string token = value.substr(pos, comma - pos);
        if (token.empty()) throw std::runtime_error("empty face id");
        ids.insert(std::stoi(token));
        pos = comma + 1;
    }
    return ids;
}

int cmdExtract(const std::vector<std::string>& args) {
    if (args.empty()) { usage(); return 2; }
    const std::string input = args[0];
    std::string output;
    std::set<int> selected;
    int rings = 1;
    for (size_t i = 1; i < args.size(); ++i) {
        if (args[i] == "--faces" && i + 1 < args.size()) {
            selected = parseIdList(args[++i]);
        } else if (args[i] == "--rings" && i + 1 < args.size()) {
            rings = std::stoi(args[++i]);
        } else if ((args[i] == "-o" || args[i] == "--output") &&
                   i + 1 < args.size()) {
            output = args[++i];
        } else {
            throw std::runtime_error("unknown extract option: " + args[i]);
        }
    }
    if (selected.empty()) throw std::runtime_error("extract needs --faces");
    if (output.empty()) throw std::runtime_error("extract needs -o <out.step>");
    if (rings < 0 || rings > 8) {
        throw std::runtime_error("--rings must be between 0 and 8");
    }

    const weft::ImportedModel imported = weft::importStepSecure(input);
    const weft::Model& model = imported.source->snapshot.model;
    weft::Analysis analysis = weft::analyze(model);
    for (int fid : selected) {
        if (fid < 1 || fid > model.faceCount()) {
            throw std::runtime_error("face id out of range: " +
                                     std::to_string(fid));
        }
    }

    std::set<int> included = selected;
    std::set<int> frontier = selected;
    for (int ring = 0; ring < rings; ++ring) {
        std::set<int> next;
        for (int fid : frontier) {
            for (int neighbor : analysis.faces[fid - 1].neighborFaceIds) {
                if (included.insert(neighbor).second) next.insert(neighbor);
            }
        }
        frontier = std::move(next);
        if (frontier.empty()) break;
    }

    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    for (int fid : included) builder.Add(compound, model.faces(fid));
    weft::writeStep(compound, output);
    std::printf("%s -> %s: %zu face(s), source ids:", input.c_str(),
                output.c_str(), included.size());
    for (int fid : included) std::printf(" %d", fid);
    std::printf("\n");
    return 0;
}

// Pure representation conversion. STEP -> mesh delegates to the certified
// command; no B-rep reader may substitute kernel triangle soup.
int cmdMesh(const std::vector<std::string>& args,
            bool validateOnly = false);

int cmdConvert(const std::vector<std::string>& args) {
    if (args.empty()) { usage(); return 2; }
    std::string input = args[0];
    std::string output;
    for (size_t i = 1; i < args.size(); ++i) {
        if ((args[i] == "-o" || args[i] == "--output") && i + 1 < args.size()) {
            output = args[++i];
        } else {
            throw std::runtime_error("unknown option: " + args[i]);
        }
    }
    if (output.empty()) throw std::runtime_error("missing -o <out>");

    weft::io::System sys;
    weft::io::bootstrapIo(sys);
    const weft::io::Format in = sys.probeFormat(input);
    const weft::io::Format out = sys.probeFormatForOutput(output);
    if (weft::io::formatProvidesBRep(in) &&
        weft::io::formatProvidesMesh(out)) {
        if (in != weft::io::Format::Step) {
            throw std::runtime_error(
                "secure B-rep-to-mesh convert currently requires STEP input");
        }
        return cmdMesh({input, "-o", output});
    }
    weft::io::convertFile(sys, input, output);
    std::printf("%s -> %s (%s)\n", input.c_str(), output.c_str(),
                std::string(weft::io::formatIdentifier(out)).c_str());
    return 0;
}

int cmdMesh(const std::vector<std::string>& args, bool validateOnly) {
    if (args.empty()) { usage(); return 2; }
    std::string input = args[0];
    std::string output;
    std::string recipeOut;
    std::string recipeInputPath;
    std::string meshReportPath;
    bool validate = validateOnly;
    bool noNormals = false;
    bool recipeLoaded = false;
    bool ignoredMigrationControl = false;
    bool progress = false;
    bool inventoryRefusals = false;
    weft::RepairProfile repairProfile = weft::RepairProfile::Conservative;
    std::vector<double> lods;
    weft::ObjExportOptions objOpts;
    weft::Recipe recipe;
    weft::GenerationSettings& gs = recipe.settings;
    std::vector<std::string> faceSpecs;
    std::vector<std::function<void(weft::Recipe&)>> postRecipeOverrides;
    std::vector<std::string> postRecipeFaceSpecs;
    auto applyAndRecord = [&](std::function<void(weft::Recipe&)> action) {
        action(recipe);
        if (recipeLoaded) postRecipeOverrides.push_back(std::move(action));
    };

    for (size_t i = 1; i < args.size(); ++i) {
        const std::string& a = args[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= args.size()) throw std::runtime_error(a + " needs a value");
            return args[++i];
        };
        if (a == "-o" || a == "--output") output = next();
        else if (a == "--progress") progress = true;
        else if (a == "--inventory-refusals") inventoryRefusals = true;
        else if (a == "--radial") {
            const int value = std::stoi(next());
            applyAndRecord([value](weft::Recipe& target) {
                target.settings.defaults.radial = value;
            });
        }
        else if (a == "--axial") {
            const int value = std::stoi(next());
            applyAndRecord([value](weft::Recipe& target) {
                target.settings.defaults.axial = value;
            });
        }
        else if (a == "--chord") {
            const double value = std::stod(next());
            applyAndRecord([value](weft::Recipe& target) {
                target.settings.defaults.chordTolerance = value;
            });
        }
        else if (a == "--cap") {
            const std::string value = next();
            applyAndRecord([value](weft::Recipe& target) {
                weft::applySetting(target.settings.defaults, "cap", value);
            });
            ignoredMigrationControl = true;
        }
        else if (a == "--loops") {
            const int value = std::stoi(next());
            applyAndRecord([value](weft::Recipe& target) {
                target.settings.defaults.filletLoops = value;
            });
            ignoredMigrationControl = true;
        }
        else if (a == "--hold") {
            const double value = std::stod(next());
            applyAndRecord([value](weft::Recipe& target) {
                target.settings.defaults.filletHold = value;
            });
            ignoredMigrationControl = true;
        }
        else if (a == "--rings") {
            const int value = std::stoi(next());
            applyAndRecord([value](weft::Recipe& target) {
                target.settings.defaults.junctionRings = value;
            });
            ignoredMigrationControl = true;
        }
        else if (a == "--pure-tris") {
            applyAndRecord([](weft::Recipe& target) {
                target.settings.defaults.quadDominant = false;
                target.settings.defaults.pureTriFloor = true;
            });
            ignoredMigrationControl = true;
        }
        else if (a == "--quads") {
            applyAndRecord([](weft::Recipe& target) {
                target.settings.defaults.quadDominant = true;
            });
            ignoredMigrationControl = true;
        }
        else if (a == "--flat-quads") {
            applyAndRecord([](weft::Recipe& target) {
                target.settings.defaults.minimal = false;
            });
            ignoredMigrationControl = true;
        }
        else if (a == "--adaptive") {
            applyAndRecord([](weft::Recipe& target) {
                target.settings.defaults.adaptive = true;
            });
            ignoredMigrationControl = true;
        }
        else if (a == "--density") {
            // One dial re-budgets the whole model: scales every
            // solved count (adaptive ones too) before the group
            // solve, and fallback tolerances to match.
            const double value = std::stod(next());
            applyAndRecord([value](weft::Recipe& target) {
                target.settings.densityScale = value;
            });
            ignoredMigrationControl = true;
        }
        else if (a == "--weld") {
            // Global weld tolerance (mm): how far apart coincident border
            // verts may sit and still fuse. Loosening it closes seams on
            // sloppy CAD / off-curve fallback borders.
            const double value = std::stod(next());
            applyAndRecord([value](weft::Recipe& target) {
                target.settings.weldTolerance = value;
            });
            ignoredMigrationControl = true;
        }
        else if (a == "--profile") {
            std::string prof = next();
            if (prof == "cad") {
                applyAndRecord([](weft::Recipe& target) {
                    target.settings.defaults.minimal = true;
                    target.settings.defaults.adaptive = true;
                    target.settings.defaults.relativeDeviation = true;
                });
            } else if (prof == "dense") {
                applyAndRecord([](weft::Recipe& target) {
                    target.settings.defaults.minimal = false;
                });
            } else {
                throw std::runtime_error("unknown profile: " + prof);
            }
            ignoredMigrationControl = true;
        }
        else if (a == "--pipeline") {
            const std::string pipeline = next();
            if (pipeline != "compiler" && pipeline != "legacy") {
                throw std::runtime_error("unknown pipeline: " + pipeline);
            }
            ignoredMigrationControl = true;
        }
        else if (a == "--repair") {
            const std::string repair = next();
            if (repair == "conservative") {
                repairProfile = weft::RepairProfile::Conservative;
            } else if (repair == "compatibility") {
                repairProfile = weft::RepairProfile::Compatibility;
            } else {
                throw std::runtime_error("unknown repair profile: " + repair);
            }
        }
        else if (a == "--mesh-report") meshReportPath = next();
        else if (a == "--validate") validate = true;
        else if (a == "--preview") {
            applyAndRecord([](weft::Recipe& target) {
                target.settings.finalizeMesh = false;
            });
            ignoredMigrationControl = true;
        }
        else if (a == "--stitch") {
            applyAndRecord([](weft::Recipe& target) {
                target.settings.decoupleSeams = true;
            });
            ignoredMigrationControl = true;
        }
        else if (a == "--debug") ignoredMigrationControl = true;
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
        else if (a == "--recipe") {
            recipeInputPath = next();
            recipeLoaded = true;
            postRecipeOverrides.clear();
            postRecipeFaceSpecs.clear();
        }
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
            applyAndRecord([op](weft::Recipe& target) {
                target.ops.push_back(op);
            });
        }
        else if (a == "--grid") {
            std::string g = next();
            size_t x = g.find('x');
            const int gridU = std::stoi(g.substr(0, x));
            const int gridV =
                x == std::string::npos ? gridU : std::stoi(g.substr(x + 1));
            applyAndRecord([gridU, gridV](weft::Recipe& target) {
                target.settings.defaults.gridU = gridU;
                target.settings.defaults.gridV = gridV;
            });
            ignoredMigrationControl = true;
        } else if (a == "--face") {
            const std::string spec = next();
            faceSpecs.push_back(spec);  // parsed after defaults are final
            if (recipeLoaded) postRecipeFaceSpecs.push_back(spec);
        } else if (a == "--edge") {
            std::string spec = next();
            size_t colon = spec.find(':');
            if (colon == std::string::npos) {
                throw std::runtime_error("--edge expects ID:N, got " + spec);
            }
            const int edgeId = std::stoi(spec.substr(0, colon));
            const int count = std::stoi(spec.substr(colon + 1));
            applyAndRecord([edgeId, count](weft::Recipe& target) {
                target.settings.perEdge[edgeId] = count;
            });
        } else {
            throw std::runtime_error("unknown option: " + a);
        }
    }
    if (output.empty() && !validateOnly) {
        throw std::runtime_error("missing -o <out.obj>");
    }
    for (const std::string& spec : faceSpecs) parseFaceOverride(gs, spec);
    if (!lods.empty() && !meshReportPath.empty()) {
        throw std::runtime_error(
            "--mesh-report with --lods is unavailable until per-tier secure "
            "reports are defined");
    }
    if (ignoredMigrationControl) {
        std::fprintf(
            stderr,
            "warning: one or more legacy modelling controls were accepted as "
            "migration input but are ignored by the secure pipeline\n");
    }

    const weft::ImportedModel secureImported =
        weft::importStepSecure(input, repairProfile);
    weft::RecipeV2 effectiveRecipe;
    bool hasEffectiveRecipe = false;
    std::vector<weft::RecipeMigrationIssue> migrationIssues;
    if (recipeLoaded) {
        const int version = weft::recipeFileVersion(recipeInputPath);
        if (version == 1) {
            const weft::RecipeV2MigrationResult migrated =
                weft::migrateRecipeV1(
                    secureImported, weft::loadRecipe(recipeInputPath));
            effectiveRecipe = migrated.recipe;
            migrationIssues = migrated.issues;
        } else {
            effectiveRecipe = weft::loadRecipeV2(recipeInputPath);
        }
        hasEffectiveRecipe = true;
    } else if (!recipeOut.empty() || !recipe.ops.empty() ||
               !gs.perFace.empty() || !gs.perEdge.empty()) {
        const weft::RecipeV2MigrationResult migrated =
            weft::captureRecipeV2(secureImported, recipe);
        effectiveRecipe = migrated.recipe;
        migrationIssues = migrated.issues;
        hasEffectiveRecipe = true;
    }

    auto firstConflict = [](const std::vector<weft::RecipeMigrationIssue>& issues)
        -> const weft::RecipeMigrationIssue* {
        const auto found = std::find_if(
            issues.begin(), issues.end(), [](const auto& issue) {
                return issue.severity == weft::RecipeIssueSeverity::Conflict;
            });
        return found == issues.end() ? nullptr : &*found;
    };
    if (const weft::RecipeMigrationIssue* conflict =
            firstConflict(migrationIssues)) {
        throw std::runtime_error(
            "recipe migration conflict [" + conflict->code + "]: " +
            conflict->message);
    }
    for (const weft::RecipeMigrationIssue& issue : migrationIssues) {
        if (issue.severity == weft::RecipeIssueSeverity::Warning) {
            std::fprintf(stderr, "warning: %s: %s\n", issue.code.c_str(),
                         issue.message.c_str());
        }
    }

    if (hasEffectiveRecipe) {
        weft::RecipeV2Resolution resolved =
            weft::resolveRecipeV2(secureImported, effectiveRecipe);
        if (const weft::RecipeMigrationIssue* conflict =
                firstConflict(resolved.issues)) {
            throw std::runtime_error(
                "recipe resolution conflict [" + conflict->code + "]: " +
                conflict->message);
        }
        for (const weft::RecipeMigrationIssue& issue : resolved.issues) {
            if (issue.severity == weft::RecipeIssueSeverity::Warning) {
                std::fprintf(stderr, "warning: %s: %s\n", issue.code.c_str(),
                             issue.message.c_str());
            }
        }
        recipe.settings = resolved.settings;
        recipe.ops = resolved.operations;

        if (!postRecipeOverrides.empty() ||
            !postRecipeFaceSpecs.empty()) {
            for (const auto& apply : postRecipeOverrides) apply(recipe);
            for (const std::string& spec : postRecipeFaceSpecs) {
                parseFaceOverride(recipe.settings, spec);
            }
            const weft::RecipeV2MigrationResult captured =
                weft::captureRecipeV2(secureImported, recipe);
            if (const weft::RecipeMigrationIssue* conflict =
                    firstConflict(captured.issues)) {
                throw std::runtime_error(
                    "recipe override conflict [" + conflict->code + "]: " +
                    conflict->message);
            }
            for (const weft::RecipeMigrationIssue& issue : captured.issues) {
                if (issue.severity == weft::RecipeIssueSeverity::Warning) {
                    std::fprintf(stderr, "warning: %s: %s\n",
                                 issue.code.c_str(), issue.message.c_str());
                }
            }
            effectiveRecipe = captured.recipe;
            resolved =
                weft::resolveRecipeV2(secureImported, effectiveRecipe);
            if (const weft::RecipeMigrationIssue* conflict =
                    firstConflict(resolved.issues)) {
                throw std::runtime_error(
                    "recipe override resolution conflict [" +
                    conflict->code + "]: " + conflict->message);
            }
        }
        if (!recipeOut.empty()) {
            weft::saveRecipeV2(effectiveRecipe, recipeOut);
            std::printf("saved recipe v2 %s\n", recipeOut.c_str());
        }
        const std::vector<weft::RecipeMigrationIssue> applicationIssues =
            weft::validateSecureRecipeApplication(resolved);
        if (const weft::RecipeMigrationIssue* conflict =
                firstConflict(applicationIssues)) {
            throw std::runtime_error(
                "recipe application conflict [" + conflict->code + "]: " +
                conflict->message);
        }
        recipe.settings = resolved.settings;
        recipe.ops = resolved.operations;
    }
    const weft::Model& model = secureImported.workingModel();
    weft::Analysis analysis = weft::analyze(model);
    objOpts.objectNames = &model.solidNames;
    if (!noNormals) objOpts.model = &model;

    // All exports flow through the io registry: probe the OUTPUT extension,
    // build the writer's params from the current CLI flags, then transfer +
    // write. OBJ/glTF/FBX behavior is preserved because the mesh writers are
    // thin adapters over the same hand-rolled exporters.
    weft::io::System sys;
    weft::io::bootstrapIo(sys);
    auto exportMesh = [&](const weft::PolyMesh& m, const std::string& path) {
        weft::io::Format out = sys.probeFormatForOutput(path);
        const weft::io::FactoryWriter* fw = sys.findFactoryWriter(out);
        if (!fw) {  // unknown extension -> preserve the old OBJ fallback
            out = weft::io::Format::Obj;
            fw = sys.findFactoryWriter(out);
        }
        weft::io::ParamGroup pg = fw->createParams(out);
        auto setIf = [&](const char* k, const std::string& v) {
            if (pg.find(k)) pg.set(k, v);
        };
        setIf("triangulate", objOpts.triangulate ? "true" : "false");
        setIf("yUp", objOpts.yUp ? "true" : "false");
        setIf("scale", std::to_string(objOpts.scale));
        setIf("normals", noNormals ? "false" : "true");
        weft::io::WriteInput in{&m, &model, &analysis.solidFaces};
        weft::io::exportFile(sys, out, in, path, &pg);
    };
    weft::MeshingResult latestSecureResult;
    auto generateSelected = [&](const weft::GenerationSettings& selected,
                                weft::GenerationReport* report,
                                weft::CompilerPlan* plan = nullptr) {
        (void)report;
        (void)plan;
        if (selected.defaults.radial < 3) {
            throw std::runtime_error("--radial must be at least 3");
        }
        if (selected.defaults.axial < 1) {
            throw std::runtime_error("--axial must be positive");
        }
        if (!std::isfinite(selected.defaults.chordTolerance) ||
            selected.defaults.chordTolerance <= 0.0) {
            throw std::runtime_error("--chord must be finite and positive");
        }
        if (!std::isfinite(selected.defaults.angleToleranceDeg) ||
            selected.defaults.angleToleranceDeg <= 0.0 ||
            selected.defaults.angleToleranceDeg >= 180.0) {
            throw std::runtime_error(
                "normal angle tolerance must be between 0 and 180 degrees");
        }

        weft::SecureMeshingConfiguration configuration;
        configuration.sampling.chordTolerance =
            selected.defaults.chordTolerance;
        configuration.sampling.normalAngleToleranceRadians =
            selected.defaults.angleToleranceDeg *
            0.01745329251994329576923690768489;
        configuration.sampling.minimumClosedCurveSegments =
            static_cast<std::uint32_t>(selected.defaults.radial);
        configuration.sampling.maximumSegmentCount = std::max<std::uint32_t>(
            4096U, configuration.sampling.minimumClosedCurveSegments);
        configuration.cylinderAxialIntervals =
            static_cast<std::uint32_t>(selected.defaults.axial);
        configuration.progressToStderr = progress;
        configuration.collectAllUnsupported = inventoryRefusals;
        if (selected.perFace.size() == 1) {
            const int faceAxial = selected.perFace.begin()->second.axial;
            if (faceAxial >= 1) {
                configuration.cylinderAxialIntervals =
                    static_cast<std::uint32_t>(faceAxial);
            }
        }
        for (const auto& [edgeId, count] : selected.perEdge) {
            if (edgeId < 1 || count < 1) {
                throw std::runtime_error(
                    "--edge requires positive working edge IDs and counts");
            }
            configuration.exactEdgeIntervalCounts.emplace(
                weft::StableId{weft::StableIdKind::Edge,
                               static_cast<std::uint64_t>(edgeId)},
                static_cast<std::uint32_t>(count));
        }

        weft::SecureMeshingResult generated =
            weft::generateSecureMesh(secureImported, configuration);
        if (!generated) {
            if (inventoryRefusals && !generated.unsupportedRecords.empty()) {
                std::map<std::string, std::size_t> byCode;
                std::map<std::string, std::size_t> byFamily;
                for (const weft::SecureMeshingUnsupportedRecord& record :
                     generated.unsupportedRecords) {
                    ++byCode[record.code + "|" + record.familyCode];
                    ++byFamily[record.familyCode];
                }
                std::printf("WEFT_INVENTORY unsupported_total=%zu\n",
                            generated.unsupportedRecords.size());
                for (const auto& [key, count] : byCode) {
                    std::printf("WEFT_INVENTORY unsupported_bucket %s count=%zu\n",
                                key.c_str(), count);
                }
                for (const auto& [family, count] : byFamily) {
                    std::printf("WEFT_INVENTORY unsupported_family %s count=%zu\n",
                                family.c_str(), count);
                }
            }
            const std::string code = generated.failure
                ? generated.failure->code
                : "secure_pipeline.unknown_refusal";
            const std::string message = generated.failure
                ? generated.failure->message
                : "secure meshing produced neither a result nor a failure";
            throw std::runtime_error(
                "secure meshing refused [" + code + "]: " + message);
        }
        latestSecureResult = std::move(*generated.value);
        const weft::CertifiedAdmissionResult admitted =
            weft::admitCertifiedMeshingResult(latestSecureResult);
        if (!admitted) {
            throw std::runtime_error(
                "secure admission refused [" +
                (admitted.failure ? admitted.failure->code
                                  : std::string("admission.unknown")) +
                "]: " +
                (admitted.failure ? admitted.failure->message
                                  : std::string("uncertified result")));
        }
        return weft::makeCertifiedPolyMeshAdapter(latestSecureResult);
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
            weft::PolyMesh lod = generateSelected(scaled, nullptr);
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

    weft::PolyMesh mesh = generateSelected(gs, nullptr);
    if (!meshReportPath.empty()) {
        std::ofstream reportFile(meshReportPath, std::ios::binary);
        if (!reportFile) {
            throw std::runtime_error(
                "cannot open secure mesh report: " + meshReportPath);
        }
        reportFile << "weft-secure-mesh-report 1\n"
                   << "repair-profile "
                   << weft::repairProfileName(secureImported.repair.profile)
                   << "\n"
                   << "source-sha256 "
                   << secureImported.repair.sourceShapeSha256 << "\n"
                   << "working-sha256 "
                   << secureImported.repair.workingShapeSha256 << "\n"
                   << "repair-identity "
                   << (secureImported.repair.identity ? 1 : 0) << "\n"
                   << "complete "
                   << (latestSecureResult.validation.complete() ? 1 : 0)
                   << "\n";
        for (const weft::ValidationCoverage& coverage :
             latestSecureResult.validation.checks) {
            reportFile << coverage.code << " expected=" << coverage.expected
                       << " checked=" << coverage.checked
                       << " skipped=" << coverage.skipped
                       << " failed=" << coverage.failed << "\n";
        }
        reportFile.close();
        if (!reportFile) {
            throw std::runtime_error(
                "failed to write secure mesh report: " + meshReportPath);
        }
    }
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
    return 0;
}

int cmdCacheCheck(const std::vector<std::string>& args) {
    if (args.empty()) { usage(); return 2; }
    bool hasFaceEdit = false;
    for (std::size_t index = 1; index < args.size(); ++index) {
        if (args[index] == "--face" && index + 1 < args.size()) {
            (void)args[++index];
            hasFaceEdit = true;
        } else if (args[index] != "--preview") {
            throw std::runtime_error(
                "unknown cache-check option: " + args[index]);
        }
    }
    if (!hasFaceEdit) {
        throw std::runtime_error(
            "cache-check needs --face ID:key=value");
    }
    throw std::runtime_error(
        "secure cache-check refused "
        "[secure_cache.incremental_dependency_unimplemented]: per-face "
        "recipe v2 references and certified dependency caching are not yet "
        "implemented");
}

int cmdSweep(const std::vector<std::string>& args) {
    if (args.empty()) { usage(); return 2; }
    const std::string input = args[0];
    std::vector<int> radials = {8, 16, 24, 32, 40, 48};
    bool verbose = false;
    bool ignoredProfile = false;
    for (std::size_t index = 1; index < args.size(); ++index) {
        const std::string& option = args[index];
        if (option == "--profile" && index + 1 < args.size()) {
            const std::string profile = args[++index];
            if (profile != "cad" && profile != "dense") {
                throw std::runtime_error("unknown profile: " + profile);
            }
            ignoredProfile = true;
        } else if (option == "--radials" && index + 1 < args.size()) {
            radials.clear();
            const std::string list = args[++index];
            for (std::size_t position = 0; position < list.size();) {
                std::size_t comma = list.find(',', position);
                if (comma == std::string::npos) comma = list.size();
                radials.push_back(
                    std::stoi(list.substr(position, comma - position)));
                position = comma + 1;
            }
        } else if (option == "--verbose") {
            verbose = true;
        } else {
            throw std::runtime_error("unknown sweep option: " + option);
        }
    }
    if (radials.empty()) {
        throw std::runtime_error("--radials must contain at least one count");
    }
    for (int radial : radials) {
        if (radial < 3) {
            throw std::runtime_error("sweep radial counts must be at least 3");
        }
    }
    if (ignoredProfile) {
        std::fprintf(
            stderr,
            "warning: --profile is accepted as ignored migration input; "
            "the secure sweep varies only the global canonical-boundary "
            "minimum\n");
    }

    const weft::ImportedModel imported = weft::importStepSecure(input);
    const weft::Model& model = imported.workingModel();
    std::size_t completed = 0;
    int failures = 0;
    for (int radial : radials) {
        weft::SecureMeshingConfiguration configuration;
        configuration.sampling.chordTolerance = 0.1;
        configuration.sampling.normalAngleToleranceRadians =
            28.0 * 0.01745329251994329576923690768489;
        configuration.sampling.minimumClosedCurveSegments =
            static_cast<std::uint32_t>(radial);
        configuration.sampling.maximumSegmentCount = std::max<std::uint32_t>(
            4096U, configuration.sampling.minimumClosedCurveSegments);

        const weft::SecureMeshingResult first =
            weft::generateSecureMesh(imported, configuration);
        const weft::SecureMeshingResult repeated =
            weft::generateSecureMesh(imported, configuration);
        if (!first || !repeated) {
            const weft::SecureMeshingResult& refused = !first ? first : repeated;
            const std::string code = refused.failure
                ? refused.failure->code
                : "secure_pipeline.unknown_refusal";
            std::printf("FAIL radial %d: %s\n", radial, code.c_str());
            ++failures;
            continue;
        }

        const weft::MeshingResult& firstValue = *first.value;
        const weft::MeshingResult& repeatedValue = *repeated.value;
        std::string failure;
        if (!firstValue.validation.complete() ||
            !repeatedValue.validation.complete()) {
            failure = "incomplete-certificate";
        } else if (firstValue.certified.topologyFingerprint !=
                   repeatedValue.certified.topologyFingerprint) {
            failure = "nondeterministic-topology-fingerprint";
        }

        const weft::PolyMesh adapter =
            weft::makeCertifiedPolyMeshAdapter(firstValue);
        const weft::ValidationReport workflow =
            weft::validateMesh(adapter, &model);
        if (failure.empty() && !workflow.watertight()) {
            failure = "workflow-watertightness";
        }
        if (!failure.empty()) {
            std::printf("FAIL radial %d: %s\n", radial, failure.c_str());
            ++failures;
        } else {
            ++completed;
            if (verbose) {
                std::printf("ok   radial %d (%zu certified triangles, %s)\n",
                            radial, firstValue.certified.triangles.size(),
                            firstValue.certified.topologyFingerprint.c_str());
            }
        }
    }
    std::printf("secure sweep: %zu completed, %d failure(s)\n",
                completed, failures);
    return failures == 0 ? 0 : 1;
}

int cmdInventory(const std::vector<std::string>& args) {
    if (args.empty()) {
        throw std::runtime_error("inventory needs an input STEP path");
    }
    // Unbuffered progress so PowerShell redirects show stages while running.
    setvbuf(stderr, nullptr, _IONBF, 0);
    setvbuf(stdout, nullptr, _IONBF, 0);

    std::string input = args[0];
    weft::RepairProfile repairProfile = weft::RepairProfile::Conservative;
    std::string gapTsv;
    int probeLimit = 0;
    for (size_t i = 1; i < args.size(); ++i) {
        if (args[i] == "--repair") {
            if (i + 1 >= args.size()) {
                throw std::runtime_error("--repair needs a value");
            }
            const std::string value = args[++i];
            if (value == "conservative") {
                repairProfile = weft::RepairProfile::Conservative;
            } else if (value == "compatibility") {
                repairProfile = weft::RepairProfile::Compatibility;
            } else {
                throw std::runtime_error(
                    "--repair expects conservative|compatibility");
            }
        } else if (args[i] == "--gap-tsv") {
            if (i + 1 >= args.size()) {
                throw std::runtime_error("--gap-tsv needs a path");
            }
            gapTsv = args[++i];
        } else if (args[i] == "--probe-limit") {
            if (i + 1 >= args.size()) {
                throw std::runtime_error("--probe-limit needs an integer");
            }
            probeLimit = std::stoi(args[++i]);
            if (probeLimit < 0) {
                throw std::runtime_error("--probe-limit must be >= 0");
            }
        } else {
            throw std::runtime_error("unknown inventory flag: " + args[i]);
        }
    }

    const auto t0 = std::chrono::steady_clock::now();
    std::fprintf(stderr, "WEFT_PROGRESS inventory.import.begin\n");
#ifdef _WIN32
    _putenv_s("WEFT_IMPORT_PROGRESS", "1");
#else
    setenv("WEFT_IMPORT_PROGRESS", "1", 1);
#endif
    const weft::ImportedModel imported =
        weft::importStepSecure(input, repairProfile);
    const auto t1 = std::chrono::steady_clock::now();
    std::fprintf(stderr, "WEFT_PROGRESS inventory.import.done ms=%lld\n",
                 static_cast<long long>(
                     std::chrono::duration_cast<std::chrono::milliseconds>(t1 -
                                                                           t0)
                         .count()));

    std::fprintf(stderr, "WEFT_PROGRESS inventory.reconnaissance.begin\n");
    const weft::ReconnaissanceReport recon = weft::reconnoitre(imported);
    const auto t2 = std::chrono::steady_clock::now();
    std::fprintf(stderr,
                 "WEFT_PROGRESS inventory.reconnaissance.done ms=%lld\n",
                 static_cast<long long>(
                     std::chrono::duration_cast<std::chrono::milliseconds>(t2 -
                                                                           t0)
                         .count()));

    std::map<std::string, std::size_t> faceFamilies;
    std::map<std::string, std::size_t> curveFamilies;
    std::map<std::string, std::size_t> supportStates;
    std::map<std::string, std::size_t> conditionCodes;
    std::map<std::string, std::size_t> faceBlockers;
    std::map<std::string, std::size_t> curveBlockers;
    std::map<std::string, std::vector<std::uint64_t>> probeIdsByFamily;
    std::size_t unsupportedTotal = 0;
    std::size_t supportedFaces = 0;

    std::ofstream gapStream;
    if (!gapTsv.empty()) {
        gapStream.open(gapTsv, std::ios::binary);
        if (!gapStream) {
            throw std::runtime_error("failed to open --gap-tsv path: " + gapTsv);
        }
        gapStream << "kind\tid\tfamily\tsupport\tstrategy\ttrim\tconditions\n";
    }

    auto joinConditions = [](const std::vector<std::string>& codes) {
        std::string joined;
        for (const std::string& code : codes) {
            if (!joined.empty()) joined.push_back(';');
            joined += code;
        }
        return joined;
    };

    for (const weft::ExactGeometryClassification& record : recon.records) {
        if (record.taxonomy == weft::GeometryTaxonomy::Surface) {
            ++faceFamilies[record.familyCode];
            ++supportStates[weft::geometrySupportStateName(record.support)];
            const std::string trim = record.trimDomain
                ? weft::trimDomainClassName(*record.trimDomain)
                : "-";
            const std::string blocker = record.familyCode + "|" +
                weft::geometrySupportStateName(record.support) + "|" +
                (record.strategyOrReasonCode.empty()
                     ? "-"
                     : record.strategyOrReasonCode) +
                "|" + trim;
            ++faceBlockers[blocker];
            if (record.support ==
                weft::GeometrySupportState::SupportedAnalyticTemplate) {
                ++supportedFaces;
            } else {
                ++unsupportedTotal;
            }
            auto& ids = probeIdsByFamily[record.familyCode];
            if (static_cast<int>(ids.size()) < std::max(probeLimit, 3)) {
                ids.push_back(record.subjectId.ordinal);
            }
            if (gapStream) {
                gapStream << "face\t" << record.subjectId.ordinal << '\t'
                          << record.familyCode << '\t'
                          << weft::geometrySupportStateName(record.support)
                          << '\t' << record.strategyOrReasonCode << '\t' << trim
                          << '\t' << joinConditions(record.conditionCodes)
                          << '\n';
            }
        } else if (record.taxonomy == weft::GeometryTaxonomy::Curve) {
            ++curveFamilies[record.familyCode];
            const bool supportedCurve =
                record.familyCode == "line" ||
                record.familyCode == "circle" ||
                record.familyCode == "bspline" ||
                record.familyCode == "bezier" ||
                std::find(record.conditionCodes.begin(),
                          record.conditionCodes.end(),
                          "degenerate") != record.conditionCodes.end();
            if (!supportedCurve) {
                ++curveBlockers[record.familyCode + "|" +
                                (record.strategyOrReasonCode.empty()
                                     ? "-"
                                     : record.strategyOrReasonCode)];
                ++unsupportedTotal;
                if (gapStream) {
                    gapStream << "curve\t" << record.subjectId.ordinal << '\t'
                              << record.familyCode << '\t'
                              << weft::geometrySupportStateName(record.support)
                              << '\t' << record.strategyOrReasonCode << "\t-\t"
                              << joinConditions(record.conditionCodes) << '\n';
                }
            }
        }
        for (const std::string& code : record.conditionCodes) {
            if (code.rfind("cutout.", 0) == 0 ||
                code.rfind("mapped.", 0) == 0 ||
                code.rfind("freeform.", 0) == 0) {
                ++conditionCodes[code];
            }
        }
    }

    std::printf("WEFT_INVENTORY file=%s\n", input.c_str());
    std::printf("WEFT_INVENTORY import_ms=%lld recon_subjects=%zu\n",
                static_cast<long long>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(t1 -
                                                                           t0)
                        .count()),
                recon.checkedSubjects);
    std::printf(
        "WEFT_INVENTORY meshable=%d recon_complete=%d supported_faces=%zu "
        "unsupported_subjects=%zu\n",
        imported.meshable() ? 1 : 0, recon.complete ? 1 : 0, supportedFaces,
        unsupportedTotal);
    for (const auto& [family, count] : faceFamilies) {
        std::printf("WEFT_INVENTORY face_family %s count=%zu\n", family.c_str(),
                    count);
    }
    for (const auto& [family, count] : curveFamilies) {
        std::printf("WEFT_INVENTORY curve_family %s count=%zu\n", family.c_str(),
                    count);
    }
    for (const auto& [state, count] : supportStates) {
        std::printf("WEFT_INVENTORY support %s count=%zu\n", state.c_str(),
                    count);
    }
    for (const auto& [code, count] : conditionCodes) {
        std::printf("WEFT_INVENTORY condition %s count=%zu\n", code.c_str(),
                    count);
    }
    for (const auto& [blocker, count] : faceBlockers) {
        std::printf("WEFT_INVENTORY face_blocker %s count=%zu\n",
                    blocker.c_str(), count);
    }
    for (const auto& [blocker, count] : curveBlockers) {
        std::printf("WEFT_INVENTORY curve_blocker %s count=%zu\n",
                    blocker.c_str(), count);
    }
    std::printf("WEFT_INVENTORY unsupported_total=%zu\n", unsupportedTotal);
    std::printf(
        "WEFT_INVENTORY inventory_ms=%lld\n",
        static_cast<long long>(
            std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t0)
                .count()));

    // Body-level first refusal (same path the app uses after load).
    {
        std::fprintf(stderr, "WEFT_PROGRESS inventory.body_mesh_probe.begin\n");
        weft::SecureMeshingConfiguration configuration;
        configuration.progressToStderr = true;
        configuration.sampling.chordTolerance = 0.1;
        configuration.sampling.normalAngleToleranceRadians = 20.0 * 3.141592653589793 / 180.0;
        configuration.sampling.minimumClosedCurveSegments = 16;
        const weft::SecureMeshingResult body =
            weft::generateSecureMesh(imported, configuration);
        if (body) {
            std::printf("WEFT_PROBE body result=ok tris=%zu fingerprint=%s\n",
                        body.value->certified.triangles.size(),
                        body.value->certified.topologyFingerprint.c_str());
        } else {
            std::printf(
                "WEFT_PROBE body result=refuse code=%s message=%s\n",
                body.failure ? body.failure->code.c_str() : "unknown",
                body.failure ? body.failure->message.c_str() : "-");
            if (body.failure) {
                for (const weft::StableId& id : body.failure->subjects) {
                    std::printf("WEFT_PROBE body subject kind=%d id=%llu\n",
                                static_cast<int>(id.kind),
                                static_cast<unsigned long long>(id.ordinal));
                }
            }
        }
        std::fprintf(stderr, "WEFT_PROGRESS inventory.body_mesh_probe.done\n");
    }

    if (probeLimit > 0 && imported.working) {
        std::fprintf(stderr,
                     "WEFT_PROGRESS inventory.face_probes.begin limit=%d\n",
                     probeLimit);
        weft::SecureMeshingConfiguration probeConfig;
        probeConfig.sampling.chordTolerance = 0.1;
        probeConfig.sampling.normalAngleToleranceRadians =
            20.0 * 3.141592653589793 / 180.0;
        probeConfig.sampling.minimumClosedCurveSegments = 16;
        for (const auto& [family, ids] : probeIdsByFamily) {
            const int n = std::min(probeLimit, static_cast<int>(ids.size()));
            for (int i = 0; i < n; ++i) {
                const std::uint64_t faceOrdinal = ids[static_cast<std::size_t>(i)];
                if (faceOrdinal < 1 ||
                    faceOrdinal >
                        static_cast<std::uint64_t>(
                            imported.working->snapshot.model.faces.Extent())) {
                    continue;
                }
                const TopoDS_Shape faceShape =
                    imported.working->snapshot.model.faces(
                        static_cast<int>(faceOrdinal));
                TopoDS_Compound compound;
                BRep_Builder builder;
                builder.MakeCompound(compound);
                builder.Add(compound, faceShape);
                const std::filesystem::path probeStep =
                    std::filesystem::temp_directory_path() /
                    ("weft_probe_face_" + std::to_string(faceOrdinal) +
                     ".step");
                try {
                    weft::writeStep(compound, probeStep.string());
                    const weft::ImportedModel probeImported =
                        weft::importStepSecure(probeStep.string(),
                                               repairProfile);
                    const weft::SecureMeshingResult probed =
                        weft::generateSecureMesh(probeImported, probeConfig);
                    if (probed) {
                        std::printf(
                            "WEFT_PROBE face id=%llu family=%s result=ok "
                            "tris=%zu\n",
                            static_cast<unsigned long long>(faceOrdinal),
                            family.c_str(),
                            probed.value->certified.triangles.size());
                    } else {
                        std::printf(
                            "WEFT_PROBE face id=%llu family=%s result=refuse "
                            "code=%s message=%s\n",
                            static_cast<unsigned long long>(faceOrdinal),
                            family.c_str(),
                            probed.failure ? probed.failure->code.c_str()
                                           : "unknown",
                            probed.failure ? probed.failure->message.c_str()
                                           : "-");
                    }
                } catch (const std::exception& error) {
                    std::printf(
                        "WEFT_PROBE face id=%llu family=%s result=exception "
                        "message=%s\n",
                        static_cast<unsigned long long>(faceOrdinal),
                        family.c_str(), error.what());
                }
                std::error_code ignored;
                std::filesystem::remove(probeStep, ignored);
            }
        }
        std::fprintf(stderr, "WEFT_PROGRESS inventory.face_probes.done\n");
    }

    if (!gapTsv.empty()) {
        std::printf("WEFT_INVENTORY gap_tsv=%s\n", gapTsv.c_str());
    }
    std::printf(
        "WEFT_INVENTORY total_ms=%lld\n",
        static_cast<long long>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0)
                .count()));
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
        if (cmd == "inventory") return cmdInventory(args);
        if (cmd == "extract") return cmdExtract(args);
        if (cmd == "convert") return cmdConvert(args);
        if (cmd == "mesh") return cmdMesh(args);
        if (cmd == "cache-check") return cmdCacheCheck(args);
        if (cmd == "validate") return cmdMesh(args, /*validateOnly=*/true);
        if (cmd == "sweep") return cmdSweep(args);
        usage();
        return 2;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
