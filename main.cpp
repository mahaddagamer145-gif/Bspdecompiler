#include "bspdecompiler/decompiler.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Drag one or more .bsp files onto this program, or run:\n"
                  << "  bspdecompiler <map.bsp> [map2.bsp ...]\n";
        return 2;
    }

    int failures = 0;
    for (int i = 1; i < argc; ++i) {
        const std::filesystem::path inputPath = argv[i];
        try {
            auto extension = inputPath.extension().string();
            std::transform(extension.begin(), extension.end(), extension.begin(),
                           [](unsigned char ch) {
                               return static_cast<char>(std::tolower(ch));
                           });
            if (extension != ".bsp") {
                throw std::runtime_error("input file must have a .bsp extension");
            }

            auto outputPath = inputPath;
            outputPath.replace_filename(inputPath.stem().string() +
                                        "_decompiled.vmf");
            bspdecompiler::Decompiler::run(inputPath, outputPath);
            std::cout << "Wrote " << outputPath.string() << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "Failed to decompile " << inputPath.string()
                      << ": " << error.what() << '\n';
        }
    }
    return failures == 0 ? 0 : 1;
}
