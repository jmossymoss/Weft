// probe95: analysis flags for given faces (isFillet etc.)
#include "weft/analysis.hpp"
#include "weft/model.hpp"
#include <cstdio>
#include <cstdlib>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    for (int i = 2; i < argc; ++i) {
        const int fid = std::atoi(argv[i]);
        const weft::FaceInfo& fi = a.faces[fid - 1];
        std::printf("face %d: isFillet=%d\n", fid,
                    fi.isFillet ? 1 : 0);
    }
    return 0;
}
