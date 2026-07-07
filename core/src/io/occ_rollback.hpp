#pragma once

// Internal to core/src/io. OCCT's readers/writers depend on process-global
// Interface_Static variables. This guard snapshots each key's prior value when
// change() is called and restores them all (LIFO) in the destructor — required
// for correctness across sequential imports and mandatory before any threaded
// import. Ported from Mayo's OccStaticVariablesRollback.

#include <string>
#include <vector>

namespace weft::io {

class OccStaticVariablesRollback {
public:
    OccStaticVariablesRollback() = default;
    OccStaticVariablesRollback(const OccStaticVariablesRollback&) = delete;
    OccStaticVariablesRollback& operator=(const OccStaticVariablesRollback&) = delete;
    ~OccStaticVariablesRollback();

    void change(const char* key, int value);
    void change(const char* key, const char* value);

private:
    struct Snapshot {
        std::string key;
        bool isInt = false;
        int intValue = 0;
        std::string strValue;
    };
    void snapshot(const char* key);
    std::vector<Snapshot> m_snapshots;
};

}  // namespace weft::io
