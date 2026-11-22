#pragma once

#include "bspdecompiler/bsp.hpp"

namespace bspdecompiler {

class Decompiler {
public:
    static void run(const std::filesystem::path& inputPath,
                    const std::filesystem::path& outputPath);
};

} // namespace bspdecompiler
