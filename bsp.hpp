#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace bspdecompiler {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3 operator+(const Vec3& other) const;
    Vec3 operator-(const Vec3& other) const;
    Vec3 operator*(double scale) const;
    double dot(const Vec3& other) const;
    Vec3 cross(const Vec3& other) const;
    double length() const;
};

struct Plane {
    Vec3 normal;
    double distance = 0.0;
};

struct BrushSide {
    std::uint16_t plane = 0;
    std::int16_t texinfo = -1;
    std::int16_t displacement = -1;
    std::int16_t bevel = 0;
};

struct Brush {
    std::int32_t firstSide = 0;
    std::int32_t sideCount = 0;
    std::int32_t contents = 0;
};

struct TexInfo {
    std::array<std::array<float, 4>, 2> textureVectors{};
    std::array<std::array<float, 4>, 2> lightmapVectors{};
    std::int32_t texdata = -1;
    int lightmapScale = 16;
};

struct BspFace {
    std::uint16_t plane = 0;
    std::int32_t firstEdge = 0;
    std::int16_t edgeCount = 0;
    std::int16_t texinfo = -1;
    std::int16_t displacement = -1;
    std::int32_t originalFace = -1;
    std::uint32_t smoothingGroups = 0;
};

struct DispInfo {
    Vec3 startPosition;
    std::int32_t firstVertex = 0;
    std::int32_t firstTriangle = 0;
    std::int32_t power = 0;
    std::array<std::int32_t, 10> allowedVertices{};
    std::int32_t sideId = -1;
};

struct DispVertex {
    Vec3 normal;
    double distance = 0.0;
    double alpha = 0.0;
};

struct Overlay {
    std::uint32_t id = 0;
    std::int16_t texinfo = -1;
    std::uint16_t faceCount = 0;
    std::array<std::int32_t, 64> faces{};
    double startU = 0.0;
    double endU = 0.0;
    double startV = 0.0;
    double endV = 0.0;
    std::array<Vec3, 4> uv{};
    Vec3 origin;
    Vec3 normal;
    Vec3 basisU;
    Vec3 basisV;
};

struct Cubemap {
    Vec3 origin;
    std::int32_t size = 0;
    std::vector<std::uint32_t> sides;
};

struct StaticProp {
    Vec3 origin;
    Vec3 angles;
    std::uint16_t model = 0;
    std::int32_t skin = 0;
    std::uint8_t solid = 0;
    double fadeMin = 0.0;
    double fadeMax = 0.0;
};

struct EntityKeyValue {
    std::string key;
    std::string value;
};

struct Entity {
    std::vector<EntityKeyValue> keyValues;
    std::int32_t model = -1;
    Vec3 origin;
};

struct BspModel {
    std::int32_t headNode = -1;
};

struct BspNode {
    std::array<std::int32_t, 2> children{};
};

struct BspLeaf {
    std::uint16_t firstBrush = 0;
    std::uint16_t brushCount = 0;
};

class BspFile {
public:
    static BspFile load(const std::filesystem::path& path);

    std::vector<std::size_t> brushesForModel(std::int32_t modelIndex) const;
    std::string textureName(std::int32_t texinfoIndex) const;
    std::string textureAxis(std::int32_t texinfoIndex, int axis, const Vec3& origin) const;

    int version = 0;
    std::vector<Plane> planes;
    std::vector<BrushSide> brushSides;
    std::vector<Brush> brushes;
    std::vector<Vec3> vertices;
    std::vector<std::array<std::uint16_t, 2>> edges;
    std::vector<std::int32_t> surfaceEdges;
    std::vector<BspFace> faces;
    std::vector<BspFace> originalFaces;
    std::vector<TexInfo> texinfos;
    std::vector<std::string> textureNames;
    std::vector<std::int32_t> textureEnvIndices;
    std::vector<Entity> entities;
    std::vector<BspModel> models;
    std::vector<BspNode> nodes;
    std::vector<BspLeaf> leaves;
    std::vector<std::uint16_t> leafBrushes;
    std::vector<DispInfo> displacements;
    std::vector<DispVertex> displacementVertices;
    std::vector<std::uint16_t> displacementTriangles;
    std::vector<Overlay> overlays;
    std::vector<Cubemap> cubemaps;
    std::vector<StaticProp> staticProps;
    std::vector<std::string> staticPropModels;

private:
    std::vector<std::size_t> collectBrushes(std::int32_t headNode) const;
};

} // namespace bspdecompiler
