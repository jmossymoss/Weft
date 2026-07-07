#pragma once

#include <map>
#include <string>
#include <vector>

namespace weft::io {

// A single typed parameter. `enumNames` are the stable wire strings for Enum
// params (a stable contract — renaming one breaks saved recipes). One
// descriptor table drives CLI help, recipe (de)serialization and app widgets.
struct ParamSpec {
    std::string key;  // "schema", "unit", "ascii", ...
    enum Type { Bool, Int, Double, Enum, String } type;
    std::vector<std::string> enumNames;  // Enum only; the stable wire strings
    std::string def;                     // default rendered as string
    std::string help;
};

// A POD value map plus its descriptor table (analogue of Mayo's
// PropertyGroup). `values` starts equal to the specs' defaults.
struct ParamGroup {
    std::vector<ParamSpec> specs;
    std::map<std::string, std::string> values;  // key -> current value

    // typed accessors used inside applyParams()
    bool getBool(const std::string& k) const;
    int getInt(const std::string& k) const;
    double getDouble(const std::string& k) const;
    std::string getEnum(const std::string& k) const;  // validated vs enumNames
    // Sets `k` to `v`; throws std::runtime_error on an unknown key or a value
    // that is not one of the declared enumNames (for Enum specs).
    void set(const std::string& k, const std::string& v);

    // Fill `values` from the specs' defaults (idempotent). Called by the
    // factories when they hand out a fresh group.
    void resetToDefaults();
    const ParamSpec* find(const std::string& k) const;
};

}  // namespace weft::io
