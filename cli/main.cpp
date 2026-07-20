// weft — headless client for B-rep import, analysis, topology generation,
// validation, recipes, conversion, and export.

#include "weft/analysis.hpp"
#include "weft/edit.hpp"
#include "weft/fixture.hpp"
#include "weft/mesh.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/export_fbx.hpp"
#include "weft/export_gltf.hpp"
#include "weft/io/system.hpp"
#include "weft/recipe.hpp"
#include "weft/validate.hpp"

#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace {

void usage() {
    std::printf(
        "weft — B-rep retopology CLI\n"
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
        "      shape (step/iges/brep); B-rep->mesh tessellates (obj/glb/stl/fbx)\n"
        "\n"
        "  weft sweep <in.step> [--profile cad] [--radials 8,16,24,...]\n"
        "      adversarial density harness: bump every revolution-family\n"
        "      face's radial through the list, re-validating each result —\n"
        "      an edit must never open a seam, demote a face to raw\n"
        "      triangulation, or leave one empty; exits 1 on any failure\n"
        "\n"
        "  weft cache-check <in.step> --face ID:key=value[,key=value...] [--preview]\n"
        "      generate once, apply one face edit, and report exact remesh\n"
        "      face ids plus cold, edit and identical-warm timings\n"
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

    weft::Model model = weft::loadStep(input);
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

// weft convert <in> -o <out> — pure import -> export, no retopo. B-rep -> B-rep
// serializes Model::shape; B-rep -> mesh uses a default BRepMesh tessellation.
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
    weft::io::Format out = sys.probeFormatForOutput(output);
    weft::io::convertFile(sys, input, output);
    std::printf("%s -> %s (%s)\n", input.c_str(), output.c_str(),
                std::string(weft::io::formatIdentifier(out)).c_str());
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
        else if (a == "--stitch") gs.decoupleSeams = true;  // experiment
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
    // Print mesher plan + demotions before the watertight exit so failed
    // validates still yield classification evidence (WEFT_FACE_KINDS / counts).
    int rc = 0;
    if (validate) {
        weft::ValidationReport vr = weft::validateMesh(mesh, &model);
        std::printf("%s", weft::formatReport(vr).c_str());
        if (!vr.watertight()) rc = 1;
    }
    if (report.faceMesher.size() <= 48 || std::getenv("WEFT_FACE_KINDS")) {
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
    // Build honesty: the kinds above show the PLAN; say when a face's
    // planned mesher couldn't build (contract floor keeps exact borders,
    // raw triangulation is the tri-soup last resort, empty is a hole),
    // attributed by face id and cause string.
    {
        const std::string demotions = weft::formatBuildDemotions(report);
        if (!demotions.empty()) std::printf("%s", demotions.c_str());
    }
    if (!report.edgeDivisions.empty()) {
        std::printf("  density-matched edges:");
        for (const auto& [eid, div] : report.edgeDivisions) {
            std::printf(" #%d=%d", eid, div);
        }
        std::printf("\n");
    }
    return rc;
}

int cmdCacheCheck(const std::vector<std::string>& args) {
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

// weft sweep — the adversarial density and override harness:
// meshes the model at its base settings, then bumps every curved /
// revolution face's radial through a sweep of counts, re-validating each
// time. The per-face override must never open a seam, never demote any
// face to raw triangulation, and never leave a face empty. The
// generation cache keeps each iteration to the faces the edit touches.
int cmdSweep(const std::vector<std::string>& args) {
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
