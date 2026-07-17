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

        weft::SecureMeshingResult generated =
            weft::generateSecureMesh(secureImported, configuration);
        if (!generated) {
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

// Frozen pre-rewrite benchmark body; no command dispatch reaches it.
[[maybe_unused]] int legacyCacheCheck(const std::vector<std::string>& args) {
    if (args.empty()) { usage(); return 2; }
    const std::string input = args[0];
    std::string faceSpec;
    bool preview = false;
    for (size_t i = 1; i < args.size(); ++i) {
        if (args[i] == "--face" && i + 1 < args.size()) {
            faceSpec = args[++i];
        } else if (args[i] == "--preview") {
            preview = true;
        } else {
            throw std::runtime_error("unknown cache-check option: " + args[i]);
        }
    }
    const size_t colon = faceSpec.find(':');
    if (colon == std::string::npos) {
        throw std::runtime_error("cache-check needs --face ID:key=value");
    }
    const int faceId = std::stoi(faceSpec.substr(0, colon));

    weft::Model model = weft::loadStep(input);
    weft::Analysis analysis = weft::analyze(model);
    if (faceId < 1 || faceId > model.faceCount()) {
        throw std::runtime_error("face id out of range");
    }
    weft::GenerationSettings base;
    base.defaults.minimal = true;
    base.defaults.adaptive = true;
    base.defaults.relativeDeviation = true;
    base.finalizeMesh = !preview;
    weft::GenerationCache cache;
    auto run = [&](const char* label, const weft::GenerationSettings& settings) {
        const auto begin = std::chrono::steady_clock::now();
        weft::GenerationReport report;
        weft::PolyMesh mesh =
            weft::generate(model, analysis, settings, &report, &cache);
        const auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - begin)
                .count();
        std::printf("%s: %lld ms, %d remeshed, %d reused", label,
                    static_cast<long long>(elapsed), report.cacheMisses,
                    report.cacheHits);
        if (!report.remeshedFaces.empty()) {
            std::printf("; face ids:");
            for (int fid : report.remeshedFaces) std::printf(" %d", fid);
        }
        std::printf("; %zu verts, %zu polys\n", mesh.vertexCount(),
                    mesh.polygonCount());
        return mesh;
    };

    run("cold", base);
    weft::GenerationSettings edited = base;
    weft::FaceMeshSettings face = base.defaults;
    weft::applySettingsList(face, faceSpec.substr(colon + 1));
    edited.perFace[faceId] = face;
    weft::PolyMesh changed = run("single-face edit", edited);
    weft::PolyMesh identical = run("identical warm run", edited);
    if (changed.vertexCount() != identical.vertexCount() ||
        changed.polygonCount() != identical.polygonCount()) {
        std::printf("ERROR: identical warm run changed mesh counts\n");
        return 1;
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

// Frozen pre-rewrite sweep retained only for baseline archaeology:
// meshes the model at its base settings, then bumps every curved /
// revolution face's radial through a sweep of counts, re-validating each
// time. The per-face override must never open a seam, never demote any
// face to raw triangulation, and never leave a face empty. The
// generation cache keeps each iteration to the faces the edit touches.
[[maybe_unused]] int legacySweep(const std::vector<std::string>& args) {
    if (args.empty()) { usage(); return 2; }
    std::string input = args[0];
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    std::vector<int> radials = {8, 16, 24, 32, 40, 48};
    bool verbose = false;
    for (size_t i = 1; i < args.size(); ++i) {
        const std::string& a = args[i];
        if (a == "--profile" && i + 1 < args.size()) {
            const std::string p = args[++i];
            if (p == "cad") {
                gs.defaults.adaptive = true;
                gs.defaults.relativeDeviation = true;
            } else if (p != "dense") {
                throw std::runtime_error("unknown profile: " + p);
            }
        } else if (a == "--radials" && i + 1 < args.size()) {
            radials.clear();
            std::string list = args[++i];
            for (size_t pos = 0; pos < list.size();) {
                size_t comma = list.find(',', pos);
                if (comma == std::string::npos) comma = list.size();
                radials.push_back(std::stoi(list.substr(pos, comma - pos)));
                pos = comma + 1;
            }
        } else if (a == "--verbose") {
            verbose = true;
        } else {
            throw std::runtime_error("unknown option: " + a);
        }
    }

    weft::Model model = weft::loadStep(input);
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationCache cache;
    weft::GenerationReport baseRep;
    weft::PolyMesh baseMesh =
        weft::generate(model, analysis, gs, &baseRep, &cache);
    {
        weft::ValidationReport vr = weft::validateMesh(baseMesh, &model);
        const size_t expected = vr.openEdgesOnInputBoundary;
        if (vr.openEdges != expected || vr.nonManifoldEdges != 0) {
            std::printf("BASE not watertight: %zu open (%zu input-boundary), "
                        "%zu non-manifold\n",
                        vr.openEdges, expected, vr.nonManifoldEdges);
            return 1;
        }
    }

    // Sweep candidates: every face meshed as a revolution family or a
    // coons/rail strip on a curved surface — the faces radial edits hit.
    std::vector<int> candidates;
    for (const auto& [fid, kind] : baseRep.faceMesher) {
        const bool revFamily = kind == weft::MesherKind::RevolutionGrid ||
                               kind == weft::MesherKind::DomeCap ||
                               kind == weft::MesherKind::AnnulusRing;
        if (revFamily) candidates.push_back(fid);
    }
    std::printf("%s: sweeping %zu face(s) x %zu radial(s)\n", input.c_str(),
                candidates.size(), radials.size());

    int failures = 0;
    size_t runs = 0;
    for (int fid : candidates) {
        for (int r : radials) {
            weft::GenerationSettings s = gs;
            weft::FaceMeshSettings f = gs.defaults;
            f.radial = r;
            s.perFace[fid] = f;
            weft::GenerationReport rep;
            weft::PolyMesh mesh;
            try {
                mesh = weft::generate(model, analysis, s, &rep, &cache);
            } catch (const std::exception& e) {
                std::printf("FAIL face %d radial %d: generate threw (%s)\n",
                            fid, r, e.what());
                ++failures;
                continue;
            }
            ++runs;
            weft::ValidationReport vr = weft::validateMesh(mesh, &model);
            const size_t expected = vr.openEdgesOnInputBoundary;
            std::string bad;
            if (vr.openEdges != expected || vr.nonManifoldEdges != 0) {
                bad += " open=" + std::to_string(vr.openEdges) +
                       "/nm=" + std::to_string(vr.nonManifoldEdges);
            }
            int raw = 0, empty = 0;
            std::vector<int> newRawFaces, newEmptyFaces;
            for (const auto& [f2, how] : rep.faceBuild) {
                if (how == 1) {
                    ++raw;
                    auto it = baseRep.faceBuild.find(f2);
                    if (it == baseRep.faceBuild.end() || it->second != 1) {
                        newRawFaces.push_back(f2);
                    }
                }
                if (how == -1) {
                    ++empty;
                    auto it = baseRep.faceBuild.find(f2);
                    if (it == baseRep.faceBuild.end() || it->second != -1) {
                        newEmptyFaces.push_back(f2);
                    }
                }
            }
            int baseRaw = 0, baseEmpty = 0;
            for (const auto& [f2, how] : baseRep.faceBuild) {
                if (how == 1) ++baseRaw;
                if (how == -1) ++baseEmpty;
            }
            if (raw > baseRaw) {
                bad += " raw-demotions=" + std::to_string(raw) + " (base " +
                       std::to_string(baseRaw) + ")";
                if (!newRawFaces.empty()) {
                    bad += " faces=";
                    for (int f2 : newRawFaces) {
                        bad += "#" + std::to_string(f2) + ",";
                    }
                    bad.pop_back();
                }
            }
            if (empty > baseEmpty) {
                bad += " empty=" + std::to_string(empty) + " (base " +
                       std::to_string(baseEmpty) + ")";
                if (!newEmptyFaces.empty()) {
                    bad += " faces=";
                    for (int f2 : newEmptyFaces) {
                        bad += "#" + std::to_string(f2) + ",";
                    }
                    bad.pop_back();
                }
            }
            if (!bad.empty()) {
                std::printf("FAIL face %d radial %d:%s\n", fid, r,
                            bad.c_str());
                ++failures;
            } else if (verbose) {
                std::printf("ok   face %d radial %d (%zu polys)\n", fid, r,
                            mesh.polygonCount());
            }
        }
    }
    std::printf("sweep: %zu runs, %d failure(s)\n", runs, failures);
    return failures ? 1 : 0;
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

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) { usage(); return 2; }
    std::string cmd = argv[1];
    std::vector<std::string> args(argv + 2, argv + argc);
    try {
        if (cmd == "fixture") return cmdFixture(args);
        if (cmd == "inspect") return cmdInspect(args);
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
