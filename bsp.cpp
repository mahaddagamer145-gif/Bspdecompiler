#include "bspdecompiler/bsp.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <set>
#include <stdexcept>
#include <utility>

namespace bspdecompiler {
namespace {

constexpr std::size_t LumpCount = 64;
constexpr std::uint32_t BspIdent = 0x50534256;
constexpr std::size_t EntitiesLump = 0;
constexpr std::size_t PlanesLump = 1;
constexpr std::size_t TexdataLump = 2;
constexpr std::size_t VerticesLump = 3;
constexpr std::size_t NodesLump = 5;
constexpr std::size_t TexinfoLump = 6;
constexpr std::size_t FacesLump = 7;
constexpr std::size_t LeafLump = 10;
constexpr std::size_t EdgesLump = 12;
constexpr std::size_t SurfaceEdgesLump = 13;
constexpr std::size_t OriginalFacesLump = 27;
constexpr std::size_t LeafBrushesLump = 17;
constexpr std::size_t BrushesLump = 18;
constexpr std::size_t BrushSidesLump = 19;
constexpr std::size_t ModelsLump = 14;
constexpr std::size_t DisplacementsLump = 26;
constexpr std::size_t GameLump = 35;
constexpr std::size_t CubemapsLump = 42;
constexpr std::size_t TexdataStringDataLump = 43;
constexpr std::size_t TexdataStringTableLump = 44;
constexpr std::size_t OverlaysLump = 45;
constexpr std::size_t DisplacementTrianglesLump = 48;
constexpr std::size_t DisplacementVerticesLump = 33;
constexpr std::uint32_t StaticPropGameLumpId = 0x73707270;

std::string formatNumber(double value) {
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << std::fixed << std::setprecision(8) << value;
    auto result = stream.str();
    while (!result.empty() && result.back() == '0') {
        result.pop_back();
    }
    if (!result.empty() && result.back() == '.') {
        result.pop_back();
    }
    return result.empty() || result == "-0" ? "0" : result;
}

struct Lump {
    std::uint32_t offset = 0;
    std::uint32_t length = 0;
    std::int32_t version = 0;
    std::int32_t fourCC = 0;
};

class Reader {
public:
    explicit Reader(const std::vector<std::uint8_t>& data) : data_(data) {}

    std::uint16_t u16(std::size_t offset) const {
        require(offset, 2);
        return static_cast<std::uint16_t>(data_[offset]) |
               static_cast<std::uint16_t>(data_[offset + 1] << 8);
    }

    std::uint8_t u8(std::size_t offset) const {
        require(offset, 1);
        return data_[offset];
    }

    std::int16_t i16(std::size_t offset) const {
        return static_cast<std::int16_t>(u16(offset));
    }

    std::uint32_t u32(std::size_t offset) const {
        require(offset, 4);
        return static_cast<std::uint32_t>(data_[offset]) |
               (static_cast<std::uint32_t>(data_[offset + 1]) << 8) |
               (static_cast<std::uint32_t>(data_[offset + 2]) << 16) |
               (static_cast<std::uint32_t>(data_[offset + 3]) << 24);
    }

    std::int32_t i32(std::size_t offset) const {
        return static_cast<std::int32_t>(u32(offset));
    }

    float f32(std::size_t offset) const {
        const std::uint32_t bits = u32(offset);
        float value = 0.0F;
        static_assert(sizeof(value) == sizeof(bits), "Unexpected float size");
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    std::string bytes(std::size_t offset, std::size_t length) const {
        require(offset, length);
        return std::string(reinterpret_cast<const char*>(data_.data() + offset),
                           length);
    }

    std::size_t size() const { return data_.size(); }

private:
    void require(std::size_t offset, std::size_t length) const {
        if (offset > data_.size() || length > data_.size() - offset) {
            throw std::runtime_error("BSP lump points outside the file");
        }
    }

    const std::vector<std::uint8_t>& data_;
};

std::vector<std::uint8_t> readFile(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("could not open input file");
    }
    input.seekg(0, std::ios::end);
    const auto length = input.tellg();
    if (length < 0) {
        throw std::runtime_error("could not determine input file size");
    }
    std::vector<std::uint8_t> data(static_cast<std::size_t>(length));
    input.seekg(0, std::ios::beg);
    if (!data.empty() &&
        !input.read(reinterpret_cast<char*>(data.data()),
                    static_cast<std::streamsize>(data.size()))) {
        throw std::runtime_error("could not read complete input file");
    }
    return data;
}

std::string parseQuotedString(const std::string& text, std::size_t& position) {
    ++position;
    std::string value;
    while (position < text.size()) {
        const char ch = text[position++];
        if (ch == '"') {
            return value;
        }
        if (ch == '\\' && position < text.size()) {
            const char escaped = text[position++];
            if (escaped == '"' || escaped == '\\') {
                value.push_back(escaped);
            } else {
                value.push_back(ch);
                value.push_back(escaped);
            }
        } else {
            value.push_back(ch);
        }
    }
    throw std::runtime_error("unterminated quoted string in entity lump");
}

std::vector<Entity> parseEntities(const Reader& reader, const Lump& lump) {
    std::string text = reader.bytes(lump.offset, lump.length);
    const auto nul = text.find('\0');
    if (nul != std::string::npos) {
        text.resize(nul);
    }
    std::vector<Entity> entities;
    std::size_t position = 0;
    while ((position = text.find('{', position)) != std::string::npos) {
        ++position;
        Entity entity;
        bool closed = false;
        while (position < text.size()) {
            while (position < text.size() &&
                   std::isspace(static_cast<unsigned char>(text[position]))) {
                ++position;
            }
            if (position == text.size()) {
                break;
            }
            if (text[position] == '}') {
                ++position;
                closed = true;
                break;
            }
            if (text[position] != '"') {
                ++position;
                continue;
            }

            const auto key = parseQuotedString(text, position);
            while (position < text.size() &&
                   std::isspace(static_cast<unsigned char>(text[position]))) {
                ++position;
            }
            if (position >= text.size() || text[position] != '"') {
                throw std::runtime_error(
                    "entity lump contains a key without a quoted value");
            }
            const auto value = parseQuotedString(text, position);
            entity.keyValues.push_back({key, value});
            if (key == "model" && value.size() > 1 && value[0] == '*') {
                try {
                    entity.model = std::stoi(value.substr(1));
                } catch (const std::exception&) {
                    throw std::runtime_error("invalid brush model index in entity");
                }
            } else if (key == "origin") {
                std::istringstream origin(value);
                if (!(origin >> entity.origin.x >> entity.origin.y >>
                      entity.origin.z)) {
                    entity.origin = {};
                }
            }
        }
        if (!closed) {
            throw std::runtime_error("unterminated entity in entity lump");
        }
        entities.push_back(std::move(entity));
    }
    return entities;
}

bool parseInteger(const std::string& value, int& result) {
    try {
        std::size_t consumed = 0;
        result = std::stoi(value, &consumed);
        return consumed == value.size();
    } catch (const std::exception&) {
        return false;
    }
}

std::vector<std::string> split(const std::string& value, char delimiter) {
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (true) {
        const auto end = value.find(delimiter, start);
        parts.push_back(value.substr(start, end == std::string::npos
                                               ? std::string::npos
                                               : end - start));
        if (end == std::string::npos) {
            return parts;
        }
        start = end + 1;
    }
}

std::string lowercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char ch) {
                       return static_cast<char>(std::tolower(ch));
                   });
    return value;
}

std::string fixTextureName(std::string name,
                           const std::vector<Cubemap>& cubemaps,
                           std::int32_t& envmap) {
    envmap = -1;
    auto path = split(name, '/');
    if (path.size() < 3 || lowercase(path.front()) != "maps") {
        return name;
    }

    auto segments = split(path.back(), '_');
    std::size_t suffixLength = 0;
    int cubeX = 0;
    int cubeY = 0;
    int cubeZ = 0;
    const auto count = segments.size();
    int a = 0;
    int b = 0;
    int c = 0;
    if (count >= 6 && lowercase(segments[count - 2]) == "depth" &&
        parseInteger(segments[count - 1], a) &&
        parseInteger(segments[count - 5], cubeX) &&
        parseInteger(segments[count - 4], cubeY) &&
        parseInteger(segments[count - 3], cubeZ)) {
        suffixLength = 5;
    } else if (count >= 3 && lowercase(segments[count - 2]) == "depth" &&
               parseInteger(segments[count - 1], a)) {
        suffixLength = 2;
    } else if (count >= 4 &&
               parseInteger(segments[count - 3], cubeX) &&
               parseInteger(segments[count - 2], cubeY) &&
               parseInteger(segments[count - 1], cubeZ)) {
        suffixLength = 3;
    } else if (count >= 3 &&
               lowercase(segments[count - 2]) == "wvt" &&
               lowercase(segments[count - 1]) == "patch") {
        suffixLength = 2;
    }

    if (suffixLength == 0) {
        return name.substr(name.find_last_of('/') + 1);
    }
    if (suffixLength != 2 || lowercase(segments[count - 2]) != "depth") {
        for (std::size_t i = 0; i < cubemaps.size(); ++i) {
            const auto& cube = cubemaps[i];
            if (cube.origin.x == cubeX && cube.origin.y == cubeY &&
                cube.origin.z == cubeZ) {
                envmap = static_cast<std::int32_t>(i);
                break;
            }
        }
    }

    std::string material = segments.front();
    for (std::size_t i = 1; i < count - suffixLength; ++i) {
        material += '_' + segments[i];
    }
    std::string result;
    const std::size_t firstPathPart =
        path.size() > 2 && lowercase(path[2]) == "maps" ? 4 : 2;
    for (std::size_t i = firstPathPart; i + 1 < path.size(); ++i) {
        result += path[i] + '/';
    }
    return result + material;
}

BspFace readFace(const Reader& reader, std::size_t offset, bool legacy) {
    const std::size_t start = offset + (legacy ? 32 : 0);
    BspFace face;
    face.plane = reader.u16(start);
    face.firstEdge = reader.i32(start + 4);
    face.edgeCount = reader.i16(start + 8);
    face.texinfo = reader.i16(start + 10);
    face.displacement = reader.i16(start + 12);
    const std::size_t extension = legacy ? 20 : 0;
    const std::size_t areaOffset = start + 24 + extension;
    face.originalFace = reader.i32(areaOffset + 20);
    face.smoothingGroups = reader.u32(areaOffset + (legacy ? 24 : 28));
    return face;
}

} // namespace

Vec3 Vec3::operator+(const Vec3& other) const {
    return {x + other.x, y + other.y, z + other.z};
}

Vec3 Vec3::operator-(const Vec3& other) const {
    return {x - other.x, y - other.y, z - other.z};
}

Vec3 Vec3::operator*(double scale) const {
    return {x * scale, y * scale, z * scale};
}

double Vec3::dot(const Vec3& other) const {
    return x * other.x + y * other.y + z * other.z;
}

Vec3 Vec3::cross(const Vec3& other) const {
    return {y * other.z - z * other.y, z * other.x - x * other.z,
            x * other.y - y * other.x};
}

double Vec3::length() const {
    return std::sqrt(dot(*this));
}

BspFile BspFile::load(const std::filesystem::path& path) {
    const auto data = readFile(path);
    Reader reader(data);
    if (reader.size() < 8 + LumpCount * 16 + 4) {
        throw std::runtime_error("file is too small to contain a BSP header");
    }
    if (reader.u32(0) != BspIdent) {
        throw std::runtime_error("file does not have a VBSP signature");
    }

    BspFile bsp;
    bsp.version = reader.i32(4);
    if (bsp.version != 17 && bsp.version != 19 && bsp.version != 20) {
        throw std::runtime_error("unsupported BSP version " +
                                 std::to_string(bsp.version));
    }

    std::array<Lump, LumpCount> lumps{};
    for (std::size_t i = 0; i < LumpCount; ++i) {
        const std::size_t header = 8 + i * 16;
        lumps[i] = {reader.u32(header), reader.u32(header + 4),
                    reader.i32(header + 8), reader.i32(header + 12)};
        if (lumps[i].offset > reader.size() ||
            lumps[i].length > reader.size() - lumps[i].offset) {
            throw std::runtime_error("BSP lump " + std::to_string(i) +
                                     " is outside the input file");
        }
    }

    const auto requireUncompressed = [&lumps](std::size_t index) {
        if (lumps[index].fourCC != 0) {
            throw std::runtime_error("compressed BSP lump " +
                                     std::to_string(index) +
                                     " is not supported");
        }
    };
    const auto requireRecords = [&lumps, &requireUncompressed](
                                    std::size_t index, std::size_t recordSize) {
        requireUncompressed(index);
        if (recordSize == 0 || lumps[index].length % recordSize != 0) {
            throw std::runtime_error("BSP lump " + std::to_string(index) +
                                     " has an invalid record length");
        }
    };

    requireUncompressed(EntitiesLump);
    bsp.entities = parseEntities(reader, lumps[EntitiesLump]);

    requireRecords(VerticesLump, 12);
    for (std::size_t offset = lumps[VerticesLump].offset;
         offset < lumps[VerticesLump].offset + lumps[VerticesLump].length;
         offset += 12) {
        bsp.vertices.push_back(
            {reader.f32(offset), reader.f32(offset + 4), reader.f32(offset + 8)});
    }

    requireRecords(EdgesLump, 4);
    for (std::size_t offset = lumps[EdgesLump].offset;
         offset < lumps[EdgesLump].offset + lumps[EdgesLump].length;
         offset += 4) {
        bsp.edges.push_back({reader.u16(offset), reader.u16(offset + 2)});
    }

    requireRecords(SurfaceEdgesLump, 4);
    for (std::size_t offset = lumps[SurfaceEdgesLump].offset;
         offset < lumps[SurfaceEdgesLump].offset +
                      lumps[SurfaceEdgesLump].length;
         offset += 4) {
        bsp.surfaceEdges.push_back(reader.i32(offset));
    }

    const auto loadFaces = [&reader, &lumps, &requireRecords](
                               std::size_t lumpIndex, bool legacy,
                               std::vector<BspFace>& destination) {
        const std::size_t recordSize = legacy ? 104 : 56;
        requireRecords(lumpIndex, recordSize);
        for (std::size_t offset = lumps[lumpIndex].offset;
             offset < lumps[lumpIndex].offset + lumps[lumpIndex].length;
             offset += recordSize) {
            destination.push_back(readFace(reader, offset, legacy));
        }
    };
    loadFaces(FacesLump, lumps[FacesLump].version == 0, bsp.faces);
    loadFaces(OriginalFacesLump, bsp.version == 17, bsp.originalFaces);

    requireRecords(PlanesLump, 20);
    for (std::size_t offset = lumps[PlanesLump].offset;
         offset < lumps[PlanesLump].offset + lumps[PlanesLump].length;
         offset += 20) {
        bsp.planes.push_back(
            {{reader.f32(offset), reader.f32(offset + 4), reader.f32(offset + 8)},
             reader.f32(offset + 12)});
    }

    requireRecords(BrushesLump, 12);
    for (std::size_t offset = lumps[BrushesLump].offset;
         offset < lumps[BrushesLump].offset + lumps[BrushesLump].length;
         offset += 12) {
        bsp.brushes.push_back(
            {reader.i32(offset), reader.i32(offset + 4), reader.i32(offset + 8)});
    }

    requireRecords(BrushSidesLump, 8);
    for (std::size_t offset = lumps[BrushSidesLump].offset;
         offset < lumps[BrushSidesLump].offset + lumps[BrushSidesLump].length;
         offset += 8) {
        bsp.brushSides.push_back({reader.u16(offset), reader.i16(offset + 2),
                                  reader.i16(offset + 4), reader.i16(offset + 6)});
    }

    requireRecords(TexinfoLump, 72);
    for (std::size_t offset = lumps[TexinfoLump].offset;
         offset < lumps[TexinfoLump].offset + lumps[TexinfoLump].length;
         offset += 72) {
        TexInfo info;
        for (int axis = 0; axis < 2; ++axis) {
            for (int component = 0; component < 4; ++component) {
                info.textureVectors[axis][component] =
                    reader.f32(offset + (axis * 4 + component) * 4);
                info.lightmapVectors[axis][component] =
                    reader.f32(offset + 32 + (axis * 4 + component) * 4);
            }
        }
        info.texdata = reader.i32(offset + 68);
        const Vec3 lightU{info.lightmapVectors[0][0],
                          info.lightmapVectors[0][1],
                          info.lightmapVectors[0][2]};
        const Vec3 lightV{info.lightmapVectors[1][0],
                          info.lightmapVectors[1][1],
                          info.lightmapVectors[1][2]};
        const double scale = (lightU.length() + lightV.length()) / 2.0;
        if (scale > 0.001) {
            info.lightmapScale =
                std::max(1, static_cast<int>(std::lround(1.0 / scale)));
        }
        bsp.texinfos.push_back(info);
    }

    requireRecords(TexdataLump, 32);
    std::vector<std::int32_t> textureNameIds;
    for (std::size_t offset = lumps[TexdataLump].offset;
         offset < lumps[TexdataLump].offset + lumps[TexdataLump].length;
         offset += 32) {
        textureNameIds.push_back(reader.i32(offset + 12));
    }
    requireRecords(TexdataStringTableLump, 4);
    std::vector<std::int32_t> stringOffsets;
    for (std::size_t offset = lumps[TexdataStringTableLump].offset;
         offset < lumps[TexdataStringTableLump].offset +
                      lumps[TexdataStringTableLump].length;
         offset += 4) {
        stringOffsets.push_back(reader.i32(offset));
    }
    requireRecords(CubemapsLump, 16);
    for (std::size_t offset = lumps[CubemapsLump].offset;
         offset < lumps[CubemapsLump].offset + lumps[CubemapsLump].length;
         offset += 16) {
        bsp.cubemaps.push_back(
            {{static_cast<double>(reader.i32(offset)),
              static_cast<double>(reader.i32(offset + 4)),
              static_cast<double>(reader.i32(offset + 8))},
             reader.i32(offset + 12), {}});
    }

    const auto& stringData = lumps[TexdataStringDataLump];
    requireUncompressed(TexdataStringDataLump);
    const auto readTextureString = [&](std::int32_t nameId) {
        if (nameId < 0 ||
            static_cast<std::size_t>(nameId) >= stringOffsets.size()) {
            return std::string();
        }
        const auto relative = stringOffsets[static_cast<std::size_t>(nameId)];
        if (relative < 0 ||
            static_cast<std::size_t>(relative) >= stringData.length) {
            throw std::runtime_error("texture string offset is outside its lump");
        }
        const auto start = static_cast<std::size_t>(stringData.offset) +
                           static_cast<std::size_t>(relative);
        std::size_t end = start;
        while (end < static_cast<std::size_t>(stringData.offset) +
                         stringData.length &&
               data[end] != 0) {
            ++end;
        }
        return reader.bytes(start, end - start);
    };
    for (const auto nameId : textureNameIds) {
        std::int32_t envmap = -1;
        bsp.textureNames.push_back(
            fixTextureName(readTextureString(nameId), bsp.cubemaps, envmap));
        bsp.textureEnvIndices.push_back(envmap);
    }

    requireRecords(DisplacementsLump, 176);
    for (std::size_t offset = lumps[DisplacementsLump].offset;
         offset < lumps[DisplacementsLump].offset +
                      lumps[DisplacementsLump].length;
         offset += 176) {
        DispInfo info;
        info.startPosition = {reader.f32(offset), reader.f32(offset + 4),
                              reader.f32(offset + 8)};
        info.firstVertex = reader.i32(offset + 12);
        info.firstTriangle = reader.i32(offset + 16);
        info.power = reader.i32(offset + 20);
        info.allowedVertices = {};
        for (std::size_t i = 0; i < info.allowedVertices.size(); ++i) {
            info.allowedVertices[i] = reader.i32(offset + 136 + i * 4);
        }
        if (info.power < 1 || info.power > 4) {
            throw std::runtime_error("displacement has an invalid power value");
        }
        bsp.displacements.push_back(info);
    }

    requireRecords(DisplacementVerticesLump, 20);
    for (std::size_t offset = lumps[DisplacementVerticesLump].offset;
         offset < lumps[DisplacementVerticesLump].offset +
                      lumps[DisplacementVerticesLump].length;
         offset += 20) {
        bsp.displacementVertices.push_back(
            {{reader.f32(offset), reader.f32(offset + 4), reader.f32(offset + 8)},
             reader.f32(offset + 12), reader.f32(offset + 16)});
    }

    requireRecords(DisplacementTrianglesLump, 2);
    for (std::size_t offset = lumps[DisplacementTrianglesLump].offset;
         offset < lumps[DisplacementTrianglesLump].offset +
                      lumps[DisplacementTrianglesLump].length;
         offset += 2) {
        bsp.displacementTriangles.push_back(reader.u16(offset));
    }

    requireRecords(OverlaysLump, 352);
    for (std::size_t offset = lumps[OverlaysLump].offset;
         offset < lumps[OverlaysLump].offset + lumps[OverlaysLump].length;
         offset += 352) {
        Overlay overlay;
        overlay.id = reader.u32(offset);
        overlay.texinfo = reader.i16(offset + 4);
        overlay.faceCount = reader.u16(offset + 6);
        for (std::size_t i = 0; i < overlay.faces.size(); ++i) {
            overlay.faces[i] = reader.i32(offset + 8 + i * 4);
        }
        overlay.startU = reader.f32(offset + 264);
        overlay.endU = reader.f32(offset + 268);
        overlay.startV = reader.f32(offset + 272);
        overlay.endV = reader.f32(offset + 276);
        for (std::size_t i = 0; i < overlay.uv.size(); ++i) {
            const auto vectorOffset = offset + 280 + i * 12;
            overlay.uv[i] = {reader.f32(vectorOffset), reader.f32(vectorOffset + 4),
                             reader.f32(vectorOffset + 8)};
        }
        overlay.origin = {reader.f32(offset + 328), reader.f32(offset + 332),
                          reader.f32(offset + 336)};
        overlay.normal = {reader.f32(offset + 340), reader.f32(offset + 344),
                          reader.f32(offset + 348)};
        overlay.basisU = {overlay.uv[0].z, overlay.uv[1].z, overlay.uv[2].z};
        const bool flipV = overlay.uv[3].z == 1.0;
        for (auto& uv : overlay.uv) {
            uv.z = 0.0;
        }
        overlay.basisV = overlay.normal.cross(overlay.basisU);
        const auto basisLength = overlay.basisV.length();
        if (basisLength > 1e-12) {
            overlay.basisV = overlay.basisV * (1.0 / basisLength);
            if (flipV) {
                overlay.basisV = overlay.basisV * -1.0;
            }
        }
        bsp.overlays.push_back(overlay);
    }

    requireUncompressed(GameLump);
    if (lumps[GameLump].length >= 4) {
        const auto gameStart = static_cast<std::size_t>(lumps[GameLump].offset);
        const auto gameLumpCount = reader.i32(gameStart);
        if (gameLumpCount < 0 ||
            static_cast<std::size_t>(gameLumpCount) >
                (lumps[GameLump].length - 4) / 16) {
            throw std::runtime_error("invalid game-lump directory");
        }
        for (std::int32_t i = 0; i < gameLumpCount; ++i) {
            const auto directoryEntry = gameStart + 4 +
                                        static_cast<std::size_t>(i) * 16;
            if (reader.u32(directoryEntry) != StaticPropGameLumpId) {
                continue;
            }
            const auto propVersion = reader.u16(directoryEntry + 6);
            const auto signedOffset = reader.i32(directoryEntry + 8);
            const auto signedLength = reader.i32(directoryEntry + 12);
            if (signedOffset < 0 || signedLength < 0) {
                throw std::runtime_error("static-prop game lump has invalid bounds");
            }
            const auto propStart = static_cast<std::size_t>(signedOffset);
            const auto propLength = static_cast<std::size_t>(signedLength);
            if (propStart > reader.size() || propLength > reader.size() - propStart) {
                throw std::runtime_error("static-prop game lump is outside the BSP");
            }
            const auto propEnd = propStart + propLength;
            const auto requirePropBytes = [&propStart, &propEnd](
                                              std::size_t offset,
                                              std::size_t length) {
                if (offset < propStart || offset > propEnd ||
                    length > propEnd - offset) {
                    throw std::runtime_error("truncated static-prop game lump");
                }
            };
            std::size_t cursor = propStart;
            requirePropBytes(cursor, 4);
            const auto modelCount = reader.i32(cursor);
            cursor += 4;
            if (modelCount < 0 ||
                static_cast<std::size_t>(modelCount) > (propEnd - cursor) / 128) {
                throw std::runtime_error("invalid static-prop model table");
            }
            for (std::int32_t model = 0; model < modelCount; ++model) {
                auto name = reader.bytes(cursor, 128);
                const auto terminator = name.find('\0');
                if (terminator != std::string::npos) {
                    name.resize(terminator);
                }
                bsp.staticPropModels.push_back(std::move(name));
                cursor += 128;
            }

            requirePropBytes(cursor, 4);
            const auto leafCount = reader.i32(cursor);
            cursor += 4;
            if (leafCount < 0 ||
                static_cast<std::size_t>(leafCount) > (propEnd - cursor) / 2) {
                throw std::runtime_error("invalid static-prop leaf table");
            }
            cursor += static_cast<std::size_t>(leafCount) * 2;
            requirePropBytes(cursor, 4);
            const auto propCount = reader.i32(cursor);
            cursor += 4;
            const std::size_t propSize =
                56 + (propVersion >= 5 ? 4 : 0) +
                (propVersion >= 6 ? 4 : 0) +
                (propVersion >= 7 ? 4 : 0) +
                (propVersion >= 8 ? 4 : 0);
            if (propCount < 0 ||
                static_cast<std::size_t>(propCount) > (propEnd - cursor) / propSize) {
                throw std::runtime_error("invalid static-prop table");
            }
            for (std::int32_t prop = 0; prop < propCount; ++prop) {
                requirePropBytes(cursor, propSize);
                StaticProp staticProp;
                staticProp.origin = {reader.f32(cursor), reader.f32(cursor + 4),
                                     reader.f32(cursor + 8)};
                staticProp.angles = {reader.f32(cursor + 12),
                                     reader.f32(cursor + 16),
                                     reader.f32(cursor + 20)};
                staticProp.model = reader.u16(cursor + 24);
                staticProp.solid = reader.u8(cursor + 30);
                staticProp.skin = reader.i32(cursor + 32);
                staticProp.fadeMin = reader.f32(cursor + 36);
                staticProp.fadeMax = reader.f32(cursor + 40);
                bsp.staticProps.push_back(staticProp);
                cursor += propSize;
            }
            if (cursor > propEnd) {
                throw std::runtime_error("static-prop data exceeds its game lump");
            }
            break;
        }
    }

    requireRecords(ModelsLump, 48);
    for (std::size_t offset = lumps[ModelsLump].offset;
         offset < lumps[ModelsLump].offset + lumps[ModelsLump].length;
         offset += 48) {
        bsp.models.push_back({reader.i32(offset + 36)});
    }

    requireRecords(NodesLump, 32);
    for (std::size_t offset = lumps[NodesLump].offset;
         offset < lumps[NodesLump].offset + lumps[NodesLump].length;
         offset += 32) {
        bsp.nodes.push_back({{reader.i32(offset + 4), reader.i32(offset + 8)}});
    }

    const std::size_t leafRecordSize = bsp.version == 19 ? 56 : 32;
    requireRecords(LeafLump, leafRecordSize);
    for (std::size_t offset = lumps[LeafLump].offset;
         offset < lumps[LeafLump].offset + lumps[LeafLump].length;
         offset += leafRecordSize) {
        bsp.leaves.push_back({reader.u16(offset + 24), reader.u16(offset + 26)});
    }

    requireRecords(LeafBrushesLump, 2);
    for (std::size_t offset = lumps[LeafBrushesLump].offset;
         offset < lumps[LeafBrushesLump].offset +
                      lumps[LeafBrushesLump].length;
         offset += 2) {
        bsp.leafBrushes.push_back(reader.u16(offset));
    }

    for (const auto& brush : bsp.brushes) {
        if (brush.firstSide < 0 || brush.sideCount < 0 ||
            static_cast<std::size_t>(brush.firstSide) > bsp.brushSides.size() ||
            static_cast<std::size_t>(brush.sideCount) >
                bsp.brushSides.size() -
                    static_cast<std::size_t>(brush.firstSide)) {
            throw std::runtime_error("brush references invalid brush sides");
        }
    }
    return bsp;
}

std::vector<std::size_t> BspFile::collectBrushes(std::int32_t headNode) const {
    std::set<std::size_t> found;
    std::set<std::int32_t> visitedNodes;
    std::set<std::int32_t> visitedLeaves;
    std::vector<std::int32_t> pending{headNode};
    while (!pending.empty()) {
        const auto nodeIndex = pending.back();
        pending.pop_back();
        if (nodeIndex < 0) {
            const auto leafIndex = -1 - nodeIndex;
            if (leafIndex < 0 ||
                static_cast<std::size_t>(leafIndex) >= leaves.size() ||
                !visitedLeaves.insert(leafIndex).second) {
                continue;
            }
            const auto& leaf = leaves[static_cast<std::size_t>(leafIndex)];
            const std::size_t first = leaf.firstBrush;
            const std::size_t count = leaf.brushCount;
            if (first > leafBrushes.size() ||
                count > leafBrushes.size() - first) {
                throw std::runtime_error("leaf references invalid leaf brushes");
            }
            for (std::size_t i = first; i < first + count; ++i) {
                if (leafBrushes[i] < brushes.size()) {
                    found.insert(leafBrushes[i]);
                }
            }
            continue;
        }
        if (static_cast<std::size_t>(nodeIndex) >= nodes.size() ||
            !visitedNodes.insert(nodeIndex).second) {
            continue;
        }
        const auto& node = nodes[static_cast<std::size_t>(nodeIndex)];
        pending.push_back(node.children[0]);
        pending.push_back(node.children[1]);
    }
    return {found.begin(), found.end()};
}

std::vector<std::size_t> BspFile::brushesForModel(
    std::int32_t modelIndex) const {
    if (modelIndex < 0 || static_cast<std::size_t>(modelIndex) >= models.size()) {
        return {};
    }
    auto result = collectBrushes(models[static_cast<std::size_t>(modelIndex)].headNode);
    if (modelIndex == 0 && result.empty()) {
        result.resize(brushes.size());
        for (std::size_t i = 0; i < result.size(); ++i) {
            result[i] = i;
        }
    }
    return result;
}

std::string BspFile::textureName(std::int32_t texinfoIndex) const {
    if (texinfoIndex < 0 ||
        static_cast<std::size_t>(texinfoIndex) >= texinfos.size()) {
        return "TOOLS/TOOLSSKIP";
    }
    const auto texdata = texinfos[static_cast<std::size_t>(texinfoIndex)].texdata;
    if (texdata < 0 || static_cast<std::size_t>(texdata) >= textureNames.size() ||
        textureNames[static_cast<std::size_t>(texdata)].empty()) {
        return "TOOLS/TOOLSSKIP";
    }
    auto name = textureNames[static_cast<std::size_t>(texdata)];
    if (name.size() >= 5 &&
        std::equal(name.begin(), name.begin() + 5, "maps/",
                   [](char a, char b) {
                       return std::tolower(static_cast<unsigned char>(a)) ==
                              std::tolower(static_cast<unsigned char>(b));
                   })) {
        const auto slash = name.find_last_of('/');
        if (slash != std::string::npos) {
            name = name.substr(slash + 1);
        }
    }
    return name;
}

std::string BspFile::textureAxis(std::int32_t texinfoIndex, int axis,
                                 const Vec3& origin) const {
    if (texinfoIndex < 0 ||
        static_cast<std::size_t>(texinfoIndex) >= texinfos.size()) {
        return axis == 0 ? "[1 0 0 0] 0.25" : "[0 -1 0 0] 0.25";
    }
    const auto& info = texinfos[static_cast<std::size_t>(texinfoIndex)];
    const auto& vector = info.textureVectors[static_cast<std::size_t>(axis)];
    const Vec3 direction{vector[0], vector[1], vector[2]};
    const double length = direction.length();
    if (length <= std::numeric_limits<double>::epsilon()) {
        return axis == 0 ? "[1 0 0 0] 0.25" : "[0 -1 0 0] 0.25";
    }
    const double shift = vector[3] - origin.dot(direction);
    std::ostringstream result;
    result.imbue(std::locale::classic());
    result << '[' << formatNumber(direction.x / length) << ' '
           << formatNumber(direction.y / length) << ' '
           << formatNumber(direction.z / length) << ' '
           << formatNumber(shift) << "] " << formatNumber(1.0 / length);
    return result.str();
}

} // namespace bspdecompiler
