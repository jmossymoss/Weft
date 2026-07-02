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
    char buf[160];
    std::snprintf(buf, sizeof buf,
                  "radial=%d,axial=%d,gridu=%d,gridv=%d,cap=%s,chord=%g",
                  s.radial, s.axial, s.gridU, s.gridV,
                  s.cap == CapStyle::Fan ? "fan" : "ngon", s.chordTolerance);
    return buf;
}

void saveRecipe(const GenerationSettings& settings, const std::string& path) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot open for writing: " + path);
    out << "weft-recipe 1\n";
    out << "default " << settingsToString(settings.defaults) << "\n";
    for (const auto& [fid, s] : settings.perFace) {
        out << "face " << fid << " " << settingsToString(s) << "\n";
    }
    for (const auto& [eid, count] : settings.perEdge) {
        out << "edge " << eid << " " << count << "\n";
    }
}

GenerationSettings loadRecipe(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open recipe: " + path);

    std::string header, version;
    in >> header >> version;
    if (header != "weft-recipe" || version != "1") {
        throw std::runtime_error("not a weft recipe (v1): " + path);
    }

    GenerationSettings gs;
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
            } else {
                throw std::runtime_error("unknown directive: " + kind);
            }
        } catch (const std::exception& e) {
            throw std::runtime_error(path + ":" + std::to_string(lineNo) +
                                     ": " + e.what());
        }
    }
    return gs;
}

}  // namespace weft
