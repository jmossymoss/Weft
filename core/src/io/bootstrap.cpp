#include "weft/io/system.hpp"

#include <memory>
#include <span>

namespace weft::io {

// Reader makers, defined in the per-format translation units.
std::unique_ptr<Reader> makeStepReader();
std::unique_ptr<Reader> makeIgesReader();
std::unique_ptr<Reader> makeBrepReader();

namespace {

// One factory for all three OCC B-rep readers (mirrors Mayo's OccFactory*).
class OccFactoryReader final : public FactoryReader {
public:
    std::span<const Format> formats() const override {
        static constexpr Format kF[] = {Format::Step, Format::Iges, Format::Brep};
        return kF;
    }
    std::unique_ptr<Reader> create(Format f) const override {
        switch (f) {
            case Format::Step: return makeStepReader();
            case Format::Iges: return makeIgesReader();
            case Format::Brep: return makeBrepReader();
            default: return nullptr;
        }
    }
};

}  // namespace

void bootstrapIo(System& sys) {
    sys.addFactoryReader(std::make_unique<OccFactoryReader>());
    // Writer factories are added in Stage B.
    addPredefinedFormatProbes(sys);
}

}  // namespace weft::io
