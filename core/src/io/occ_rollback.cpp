#include "occ_rollback.hpp"

#include <Interface_Static.hxx>

namespace weft::io {

void OccStaticVariablesRollback::change(const char* key, int value) {
    Snapshot s;
    s.key = key;
    s.isInt = true;
    s.intValue = Interface_Static::IVal(key);
    m_snapshots.push_back(std::move(s));
    Interface_Static::SetIVal(key, value);
}

void OccStaticVariablesRollback::change(const char* key, const char* value) {
    Snapshot s;
    s.key = key;
    s.isInt = false;
    s.strValue = Interface_Static::CVal(key);
    m_snapshots.push_back(std::move(s));
    Interface_Static::SetCVal(key, value);
}

OccStaticVariablesRollback::~OccStaticVariablesRollback() {
    // Restore in reverse (LIFO) so overlapping keys resolve to their earliest
    // snapshot.
    for (auto it = m_snapshots.rbegin(); it != m_snapshots.rend(); ++it) {
        if (it->isInt)
            Interface_Static::SetIVal(it->key.c_str(), it->intValue);
        else
            Interface_Static::SetCVal(it->key.c_str(), it->strValue.c_str());
    }
}

}  // namespace weft::io
