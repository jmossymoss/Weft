#include "weft/io/params.hpp"

#include <algorithm>
#include <stdexcept>

namespace weft::io {

const ParamSpec* ParamGroup::find(const std::string& k) const {
    for (const ParamSpec& s : specs)
        if (s.key == k) return &s;
    return nullptr;
}

void ParamGroup::resetToDefaults() {
    for (const ParamSpec& s : specs) values[s.key] = s.def;
}

static const std::string& valueOr(const ParamGroup& g, const std::string& k) {
    auto it = g.values.find(k);
    if (it != g.values.end()) return it->second;
    if (const ParamSpec* s = g.find(k)) return s->def;
    throw std::runtime_error("io::ParamGroup: unknown key '" + k + "'");
}

bool ParamGroup::getBool(const std::string& k) const {
    const std::string& v = valueOr(*this, k);
    return v == "1" || v == "true" || v == "yes" || v == "on";
}

int ParamGroup::getInt(const std::string& k) const {
    return std::stoi(valueOr(*this, k));
}

double ParamGroup::getDouble(const std::string& k) const {
    return std::stod(valueOr(*this, k));
}

std::string ParamGroup::getEnum(const std::string& k) const {
    const std::string v = valueOr(*this, k);
    if (const ParamSpec* s = find(k); s && s->type == ParamSpec::Enum) {
        if (std::find(s->enumNames.begin(), s->enumNames.end(), v) == s->enumNames.end())
            throw std::runtime_error("io::ParamGroup: value '" + v + "' not valid for enum '" +
                                     k + "'");
    }
    return v;
}

void ParamGroup::set(const std::string& k, const std::string& v) {
    const ParamSpec* s = find(k);
    if (!s) throw std::runtime_error("io::ParamGroup: unknown key '" + k + "'");
    if (s->type == ParamSpec::Enum &&
        std::find(s->enumNames.begin(), s->enumNames.end(), v) == s->enumNames.end())
        throw std::runtime_error("io::ParamGroup: value '" + v + "' not valid for enum '" + k +
                                 "'");
    values[k] = v;
}

}  // namespace weft::io
