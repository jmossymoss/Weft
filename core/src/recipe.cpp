#include "weft/recipe.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace weft {

void applySetting(FaceMeshSettings& s, const std::string& key,
                  const std::string& value) {
    if (key == "radial") s.radial = std::stoi(value);
    else if (key == "axial") s.axial = std::stoi(value);
    else if (key == "gridu") s.gridU = std::stoi(value);
    else if (key == "gridv") s.gridV = std::stoi(value);
    else if (key == "chord") s.chordTolerance = std::stod(value);
    else if (key == "angle") s.angleToleranceDeg = std::stod(value);
    else if (key == "loops") s.filletLoops = std::stoi(value);
    else if (key == "hold") s.filletHold = std::stod(value);
    else if (key == "rings") s.junctionRings = std::stoi(value);
    else if (key == "quads") s.quadDominant = std::stoi(value) != 0;
    else if (key == "minimal") s.minimal = std::stoi(value) != 0;
    else if (key == "skip") s.exclude = std::stoi(value) != 0;
    else if (key == "mesher") s.forceMesher = std::stoi(value);
    else if (key == "linkrims") s.linkRims = std::stoi(value) != 0;
    else if (key == "minsize") s.minSize = std::stod(value);
    else if (key == "reldev") s.relativeDeviation = std::stoi(value) != 0;
    else if (key == "adapt") s.adaptive = std::stoi(value) != 0;
    else if (key == "boundary") s.boundary = std::stoi(value);
    else if (key == "sqcollar") s.squareCollar = std::stoi(value) != 0;
    else if (key == "crot") s.coonsRotate = std::stoi(value);
    else if (key == "cap") {
        if (value == "ngon") s.cap = CapStyle::NGon;
        else if (value == "fan") s.cap = CapStyle::Fan;
        else throw std::runtime_error("unknown cap style: " + value);
    } else {
        throw std::runtime_error("unknown setting: " + key);
    }
}

void applySettingsList(FaceMeshSettings& s, const std::string& list) {
    size_t pos = 0;
    while (pos < list.size()) {
        size_t comma = list.find(',', pos);
        std::string pair = list.substr(pos, comma - pos);
        size_t eq = pair.find('=');
        if (eq == std::string::npos) {
            throw std::runtime_error("expected key=val, got " + pair);
        }
        applySetting(s, pair.substr(0, eq), pair.substr(eq + 1));
        pos = comma == std::string::npos ? list.size() : comma + 1;
    }
}

static std::string settingsToString(const FaceMeshSettings& s) {
    char buf[384];
    std::snprintf(buf, sizeof buf,
                  "radial=%d,axial=%d,gridu=%d,gridv=%d,cap=%s,chord=%g,"
                  "angle=%g,loops=%d,hold=%g,rings=%d,quads=%d,minimal=%d,"
                  "skip=%d,mesher=%d,linkrims=%d,minsize=%g,reldev=%d,"
                  "adapt=%d,boundary=%d,sqcollar=%d,crot=%d",
                  s.radial, s.axial, s.gridU, s.gridV,
                  s.cap == CapStyle::Fan ? "fan" : "ngon", s.chordTolerance,
                  s.angleToleranceDeg, s.filletLoops, s.filletHold,
                  s.junctionRings, s.quadDominant ? 1 : 0, s.minimal ? 1 : 0,
                  s.exclude ? 1 : 0, s.forceMesher, s.linkRims ? 1 : 0,
                  s.minSize, s.relativeDeviation ? 1 : 0,
                  s.adaptive ? 1 : 0, s.boundary, s.squareCollar ? 1 : 0,
                  s.coonsRotate);
    return buf;
}

void saveRecipe(const Recipe& recipe, const std::string& path) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot open for writing: " + path);
    out << "weft-recipe 1\n";
    out << "default " << settingsToString(recipe.settings.defaults) << "\n";
    for (const auto& [fid, s] : recipe.settings.perFace) {
        out << "face " << fid << " " << settingsToString(s) << "\n";
    }
    for (const auto& [eid, count] : recipe.settings.perEdge) {
        out << "edge " << eid << " " << count << "\n";
    }
    out.precision(17);
    for (const ManualOp& op : recipe.ops) {
        if (op.kind == ManualOp::Kind::Bridge) {
            out << "op bridge " << op.edgeA << " " << op.edgeB << " "
                << op.twist << " " << op.spans << " " << op.twistSide
                << "\n";
        } else if (op.kind == ManualOp::Kind::NudgeVertex) {
            out << "op nudge " << op.faceId << " " << op.u << " " << op.v
                << " " << op.u2 << " " << op.v2 << "\n";
        } else if (op.kind == ManualOp::Kind::FillLoop) {
            out << "op fill " << op.edgeA << "\n";
        } else if (op.kind == ManualOp::Kind::DeletePoly) {
            // (u,v,t) hold the world-space centroid of the polygon.
            out << "op delpoly " << op.u << " " << op.v << " " << op.t
                << "\n";
        } else if (op.kind == ManualOp::Kind::DissolveLoop) {
            // (u,v,t) hold the world-space midpoint of the seed edge.
            out << "op dissolve " << op.u << " " << op.v << " " << op.t
                << "\n";
        } else {
            out << "op loop " << op.faceId << " " << op.u << " " << op.v
                << " " << op.t << "\n";
        }
    }
}

Recipe loadRecipe(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open recipe: " + path);

    std::string header, version;
    in >> header >> version;
    if (header != "weft-recipe" || version != "1") {
        throw std::runtime_error("not a weft recipe (v1): " + path);
    }

    Recipe recipe;
    GenerationSettings& gs = recipe.settings;
    std::string line;
    std::getline(in, line);  // finish the header line
    int lineNo = 1;
    while (std::getline(in, line)) {
        ++lineNo;
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string kind;
        ss >> kind;
        try {
            if (kind == "default") {
                std::string list;
                ss >> list;
                applySettingsList(gs.defaults, list);
            } else if (kind == "face") {
                int fid;
                std::string list;
                ss >> fid >> list;
                FaceMeshSettings s = gs.defaults;
                applySettingsList(s, list);
                gs.perFace[fid] = s;
            } else if (kind == "edge") {
                int eid, count;
                ss >> eid >> count;
                gs.perEdge[eid] = count;
            } else if (kind == "op") {
                std::string opKind;
                ss >> opKind;
                ManualOp op;
                if (opKind == "loop") {
                    ss >> op.faceId >> op.u >> op.v >> op.t;
                    if (!ss) throw std::runtime_error("malformed op loop");
                } else if (opKind == "bridge") {
                    op.kind = ManualOp::Kind::Bridge;
                    ss >> op.edgeA >> op.edgeB;
                    if (!ss) throw std::runtime_error("malformed op bridge");
                    if (!(ss >> op.twist)) op.twist = 0;  // older recipes
                    if (!(ss >> op.spans)) op.spans = 1;
                    if (!(ss >> op.twistSide)) op.twistSide = 0;
                } else if (opKind == "nudge") {
                    op.kind = ManualOp::Kind::NudgeVertex;
                    ss >> op.faceId >> op.u >> op.v >> op.u2 >> op.v2;
                    if (!ss) throw std::runtime_error("malformed op nudge");
                } else if (opKind == "fill") {
                    op.kind = ManualOp::Kind::FillLoop;
                    ss >> op.edgeA;
                    if (!ss) throw std::runtime_error("malformed op fill");
                } else if (opKind == "delpoly") {
                    op.kind = ManualOp::Kind::DeletePoly;
                    ss >> op.u >> op.v >> op.t;
                    if (!ss) {
                        throw std::runtime_error("malformed op delpoly");
                    }
                } else if (opKind == "dissolve") {
                    op.kind = ManualOp::Kind::DissolveLoop;
                    ss >> op.u >> op.v >> op.t;
                    if (!ss) {
                        throw std::runtime_error("malformed op dissolve");
                    }
                } else {
                    throw std::runtime_error("unknown op: " + opKind);
                }
                recipe.ops.push_back(op);
            } else {
                throw std::runtime_error("unknown directive: " + kind);
            }
        } catch (const std::exception& e) {
            throw std::runtime_error(path + ":" + std::to_string(lineNo) +
                                     ": " + e.what());
        }
    }
    return recipe;
}

}  // namespace weft
