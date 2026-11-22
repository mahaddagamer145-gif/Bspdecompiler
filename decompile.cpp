#include "bspdecompiler/decompiler.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace bspdecompiler {
namespace {

constexpr double ClipEpsilon = 0.01;
constexpr double WindExtent = 56756.0;
constexpr std::int32_t DetailContents = 1 << 27;
constexpr std::int32_t ClipContents = (1 << 16) | (1 << 17);
constexpr std::int32_t PlayerClipContents = 1 << 16;
constexpr std::int32_t NpcClipContents = 1 << 17;

using Polygon = std::vector<Vec3>;

std::string formatNumber(double value) {
    if (std::abs(value) < 0.000000005) {
        value = 0.0;
    }
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

std::string lowercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char ch) {
                       return static_cast<char>(std::tolower(ch));
                   });
    return value;
}

Vec3 normalized(const Vec3& value) {
    const auto length = value.length();
    return length > 1e-12 ? value * (1.0 / length) : Vec3{};
}

std::string quoted(const std::string& value) {
    std::string result;
    result.reserve(value.size() + 2);
    for (const char ch : value) {
        if (ch == '"' || ch == '\\') {
            result.push_back('\\');
        }
        result.push_back(ch);
    }
    return '"' + result + '"';
}

std::string vectorString(const Vec3& value) {
    return formatNumber(value.x) + " " + formatNumber(value.y) + " " +
           formatNumber(value.z);
}

std::string planePoint(const Vec3& value) {
    return '(' + vectorString(value) + ')';
}

void keyValue(std::ostream& output, int depth, const std::string& key,
              const std::string& value) {
    output << std::string(static_cast<std::size_t>(depth), '\t')
           << quoted(key) << ' ' << quoted(value) << '\n';
}

Polygon sidePolygon(const BspFile& bsp, const Brush& brush, int sideIndex,
                    bool crunch = true) {
    const auto& baseSide =
        bsp.brushSides[static_cast<std::size_t>(brush.firstSide + sideIndex)];
    if (baseSide.plane >= bsp.planes.size()) {
        return {};
    }
    const auto& plane = bsp.planes[baseSide.plane];
    const std::array<double, 3> normalComponents{
        plane.normal.x, plane.normal.y, plane.normal.z};
    int dominantAxis = 0;
    for (int axis = 1; axis < 3; ++axis) {
        if (std::abs(normalComponents[static_cast<std::size_t>(axis)]) >
            std::abs(normalComponents[static_cast<std::size_t>(dominantAxis)])) {
            dominantAxis = axis;
        }
    }
    if (plane.normal.length() < 1e-12) {
        return {};
    }

    const Vec3 origin = plane.normal * plane.distance;
    Vec3 up = dominantAxis == 2 ? Vec3{1.0, 0.0, 0.0}
                                : Vec3{0.0, 0.0, 1.0};
    up = up - plane.normal * up.dot(plane.normal);
    const double upLength = up.length();
    if (upLength < 1e-12) {
        return {};
    }
    up = up * (WindExtent / upLength);
    Vec3 right = up.cross(plane.normal);
    right = right * WindExtent;
    Polygon polygon{origin - right + up, origin + right + up,
                    origin + right - up, origin - right - up};

    for (int i = 0; i < brush.sideCount && polygon.size() >= 3; ++i) {
        if (i == sideIndex) {
            continue;
        }
        const auto& otherSide =
            bsp.brushSides[static_cast<std::size_t>(brush.firstSide + i)];
        if (otherSide.bevel == 1 || otherSide.plane >= bsp.planes.size()) {
            continue;
        }
        const auto& clippingPlane = bsp.planes[otherSide.plane];
        Polygon clipped;
        clipped.reserve(polygon.size() + 1);
        const auto classify = [](double distance) {
            if (distance > ClipEpsilon) {
                return 1;
            }
            if (distance < -ClipEpsilon) {
                return 0;
            }
            return 2;
        };
        for (std::size_t vertex = 0; vertex < polygon.size(); ++vertex) {
            const Vec3& current = polygon[vertex];
            const Vec3& next = polygon[(vertex + 1) % polygon.size()];
            const double currentDistance =
                clippingPlane.normal.dot(current) - clippingPlane.distance;
            const double nextDistance =
                clippingPlane.normal.dot(next) - clippingPlane.distance;
            const int currentSide = classify(currentDistance);
            const int nextSide = classify(nextDistance);

            if (currentSide == 0 || currentSide == 2) {
                clipped.push_back(current);
            }
            if (currentSide != 2 && nextSide != 2 &&
                currentSide != nextSide) {
                const double denominator = currentDistance - nextDistance;
                if (std::abs(denominator) > 1e-12) {
                    const double fraction = currentDistance / denominator;
                    clipped.push_back(current + (next - current) * fraction);
                }
            }
        }
        polygon = std::move(clipped);
    }

    if (!crunch) {
        return polygon;
    }

    Polygon clean;
    for (const auto& point : polygon) {
        if (clean.empty() || (point - clean.back()).length() > 0.1) {
            clean.push_back(point);
        }
    }
    if (clean.size() > 1 && (clean.front() - clean.back()).length() <= 0.1) {
        clean.pop_back();
    }
    return clean;
}

Polygon facePolygon(const BspFile& bsp, const BspFace& face) {
    if (face.firstEdge < 0 || face.edgeCount < 0 ||
        static_cast<std::size_t>(face.firstEdge) > bsp.surfaceEdges.size() ||
        static_cast<std::size_t>(face.edgeCount) >
            bsp.surfaceEdges.size() - static_cast<std::size_t>(face.firstEdge)) {
        throw std::runtime_error("face references invalid surface edges");
    }
    Polygon polygon;
    polygon.reserve(static_cast<std::size_t>(face.edgeCount));
    for (int i = 0; i < face.edgeCount; ++i) {
        const auto surfaceEdge =
            bsp.surfaceEdges[static_cast<std::size_t>(face.firstEdge + i)];
        const auto edgeIndex = surfaceEdge < 0
                                   ? -static_cast<std::int64_t>(surfaceEdge)
                                   : static_cast<std::int64_t>(surfaceEdge);
        if (edgeIndex < 0 ||
            static_cast<std::size_t>(edgeIndex) >= bsp.edges.size()) {
            throw std::runtime_error("surface edge references an invalid edge");
        }
        const auto& edge = bsp.edges[static_cast<std::size_t>(edgeIndex)];
        const auto vertexIndex = edge[surfaceEdge < 0 ? 1 : 0];
        if (vertexIndex >= bsp.vertices.size()) {
            throw std::runtime_error("edge references an invalid vertex");
        }
        polygon.push_back(bsp.vertices[vertexIndex]);
    }
    return polygon;
}

bool windingsMatch(const Polygon& first, const Polygon& second) {
    if (first.size() != second.size() || first.empty()) {
        return false;
    }
    double bestDistance = std::numeric_limits<double>::max();
    for (std::size_t offset = 0; offset < first.size(); ++offset) {
        double distance = 0.0;
        for (std::size_t i = 0; i < first.size(); ++i) {
            distance +=
                (first[i] - second[(i + offset) % second.size()]).length();
        }
        bestDistance = std::min(bestDistance, distance);
    }
    return bestDistance < 0.5;
}

bool polygonContains(const Polygon& outer, const Polygon& inner,
                     const Plane& plane) {
    if (outer.size() < 3 || inner.empty()) {
        return false;
    }

    Vec3 center;
    for (const auto& point : outer) {
        center = center + point;
    }
    center = center * (1.0 / static_cast<double>(outer.size()));
    const double normalLength = plane.normal.length();
    if (normalLength < 1e-12) {
        return false;
    }
    const Vec3 normal = plane.normal * (1.0 / normalLength);
    constexpr double tolerance = 0.5;

    for (const auto& point : inner) {
        if (std::abs(normal.dot(point) - plane.distance / normalLength) >
            tolerance) {
            return false;
        }
        for (std::size_t edge = 0; edge < outer.size(); ++edge) {
            const auto& start = outer[edge];
            const auto edgeVector = outer[(edge + 1) % outer.size()] - start;
            const double edgeLength = edgeVector.length();
            if (edgeLength < 1e-8) {
                continue;
            }
            const double centerSide = edgeVector.cross(center - start).dot(normal);
            const double pointSide = edgeVector.cross(point - start).dot(normal);
            if ((centerSide >= 0.0 && pointSide < -tolerance * edgeLength) ||
                (centerSide < 0.0 && pointSide > tolerance * edgeLength)) {
                return false;
            }
        }
    }
    return true;
}

bool choosePlanePoints(const Polygon& polygon, const Vec3& origin,
                       std::array<Vec3, 3>& result) {
    if (polygon.size() < 3) {
        return false;
    }
    double largestArea = 0.0;
    std::size_t second = polygon.size();
    std::size_t third = polygon.size();
    for (std::size_t i = 1; i < polygon.size(); ++i) {
        for (std::size_t j = i + 1; j < polygon.size(); ++j) {
            const double area =
                (polygon[i] - polygon[0]).cross(polygon[j] - polygon[0]).length();
            if (area > largestArea) {
                largestArea = area;
                second = i;
                third = j;
            }
        }
    }
    if (second == polygon.size() || third == polygon.size() ||
        largestArea < 1e-6) {
        return false;
    }
    result = {polygon[0] + origin, polygon[second] + origin,
              polygon[third] + origin};
    return true;
}

class VmfWriter {
public:
    VmfWriter(BspFile& bsp, std::ostream& output,
              const std::filesystem::path& inputPath)
        : bsp_(bsp), output_(output), inputPath_(inputPath),
          brushSideIds_(bsp.brushSides.size(), 0) {}

    void write() {
        writeHeader();
        writeWorld();
        writeWorldBrushes();
        writeDisplacements();
        output_ << "}\n";
        writeEntities();
        writeDetails();
        writeOverlays();
        writeStaticProps();
        writeCubemaps();
        writeTrailer();
    }

private:
    void writeHeader() {
        output_ << "versioninfo\n{\n";
        keyValue(output_, 1, "editorversion", "400");
        keyValue(output_, 1, "editorbuild", "2978");
        keyValue(output_, 1, "mapversion", "2");
        keyValue(output_, 1, "formatversion", "100");
        keyValue(output_, 1, "prefab", "0");
        output_ << "}\nvisgroups\n{\n\tvisgroup\n\t{\n";
        keyValue(output_, 2, "name", "Auto");
        keyValue(output_, 2, "visgroupid", "1");
        keyValue(output_, 2, "color", "152 185 142");
        output_ << "\t\tvisgroup\n\t\t{\n";
        keyValue(output_, 3, "name", "Entities");
        keyValue(output_, 3, "visgroupid", "3");
        keyValue(output_, 3, "color", "104 137 238");
        output_ << "\t\t}\n\t\tvisgroup\n\t\t{\n";
        keyValue(output_, 3, "name", "Detail Brushes");
        keyValue(output_, 3, "visgroupid", "2");
        keyValue(output_, 3, "color", "240 25 150");
        output_ << "\t\t}\n\t\tvisgroup\n\t\t{\n";
        keyValue(output_, 3, "name", "Brush Entities");
        keyValue(output_, 3, "visgroupid", "4");
        keyValue(output_, 3, "color", "231 148 165");
        output_ << "\t\t}\n\t\tvisgroup\n\t\t{\n";
        keyValue(output_, 3, "name", "Clip Brushes");
        keyValue(output_, 3, "visgroupid", "6");
        keyValue(output_, 3, "color", "255 128 128");
        output_ << "\t\t}\n\t\tvisgroup\n\t\t{\n";
        keyValue(output_, 3, "name", "Hint Brushes");
        keyValue(output_, 3, "visgroupid", "7");
        keyValue(output_, 3, "color", "100 120 190");
        output_ << "\t\t}\n\t}\n}\nviewsettings\n{\n";
        keyValue(output_, 1, "bSnapToGrid", "0");
        keyValue(output_, 1, "bShowGrid", "1");
        keyValue(output_, 1, "nGridSpacing", "128");
        keyValue(output_, 1, "bShow3DGrid", "0");
        output_ << "}\n";
    }

    void writeWorld() {
        output_ << "world\n{\n";
        keyValue(output_, 1, "id", "1");
        keyValue(output_, 1, "mapversion", "2");
        if (!bsp_.entities.empty()) {
            for (const auto& entry : bsp_.entities.front().keyValues) {
                if (entry.key != "world_mins" && entry.key != "world_maxs") {
                    keyValue(output_, 1, entry.key, entry.value);
                }
            }
        }
        keyValue(output_, 1, "comment",
                 "Decompiled by BSPDecompiler from " + inputPath_.filename().string());
    }

    void writeWorldBrushes() {
        for (const auto brushIndex : bsp_.brushesForModel(0)) {
            const auto& brush = bsp_.brushes[brushIndex];
            if (brush.contents != (1 | DetailContents)) {
                writeBrush(1, brush, {}, static_cast<int>(brushIndex));
            }
        }
    }

    void writeEntities() {
        for (std::size_t i = 1; i < bsp_.entities.size(); ++i) {
            const auto& entity = bsp_.entities[i];
            output_ << "entity\n{\n";
            keyValue(output_, 1, "id", std::to_string(nextObjectId_++));

            std::vector<const EntityKeyValue*> connections;
            for (const auto& entry : entity.keyValues) {
                if (entry.key == "model" && entity.model >= 0) {
                    continue;
                }
                if (isConnection(entry.value)) {
                    connections.push_back(&entry);
                } else {
                    keyValue(output_, 1, entry.key, entry.value);
                }
            }
            if (!connections.empty()) {
                output_ << "\tconnections\n\t{\n";
                for (const auto* connection : connections) {
                    keyValue(output_, 2, connection->key, connection->value);
                }
                output_ << "\t}\n";
            }

            if (entity.model > 0) {
                for (const auto brushIndex : bsp_.brushesForModel(entity.model)) {
                    writeBrush(1, bsp_.brushes[brushIndex], entity.origin,
                               static_cast<int>(brushIndex));
                }
            }
            output_ << "}\n";
        }
    }

    bool isConnection(const std::string& value) const {
        if (std::count(value.begin(), value.end(), ',') != 4) {
            return false;
        }
        const auto lastComma = value.find_last_of(',');
        const auto finalField = value.substr(lastComma + 1);
        return finalField == "1" || finalField == "-1";
    }

    std::string material(const BrushSide& side, std::int32_t contents) const {
        if ((contents & ClipContents) == PlayerClipContents) {
            return "TOOLS/TOOLSPLAYERCLIP";
        }
        if ((contents & ClipContents) == NpcClipContents) {
            return "TOOLS/TOOLSNPCRCLIP";
        }
        if ((contents & ClipContents) == ClipContents) {
            return "TOOLS/TOOLSCLIP";
        }
        return bsp_.textureName(side.texinfo);
    }

    std::string axisForPlane(std::int32_t texinfoIndex, int axis,
                             const Vec3& normal, const Vec3& origin) const {
        bool generate = texinfoIndex < 0 ||
                        static_cast<std::size_t>(texinfoIndex) >=
                            bsp_.texinfos.size();
        if (!generate) {
            const auto& info = bsp_.texinfos[static_cast<std::size_t>(texinfoIndex)];
            const Vec3 firstTextureVector{info.textureVectors[0][0],
                                          info.textureVectors[0][1],
                                          info.textureVectors[0][2]};
            const Vec3 secondTextureVector{info.textureVectors[1][0],
                                           info.textureVectors[1][1],
                                           info.textureVectors[1][2]};
            const Vec3 textureNormal =
                firstTextureVector.cross(secondTextureVector);
            const auto length = textureNormal.length();
            generate = length <= 1e-12 ||
                       std::abs(normal.dot(textureNormal * (1.0 / length))) < 0.02;
        }
        if (!generate) {
            return bsp_.textureAxis(texinfoIndex, axis, origin);
        }

        Vec3 up{0.0, 0.0, 1.0};
        if (std::abs(up.dot(normal)) > 0.707) {
            up = {1.0, 0.0, 0.0};
        }
        Vec3 direction = up.cross(normal);
        const auto directionLength = direction.length();
        if (directionLength > 1e-12) {
            direction = direction * (1.0 / directionLength);
        }
        if (axis == 1) {
            direction = direction.cross(normal);
            const auto length = direction.length();
            if (length > 1e-12) {
                direction = direction * (1.0 / length);
            }
        }
        return '[' + vectorString(direction) + " 0] 0.25";
    }

    int surfaceGroup(const BrushSide& side, const std::string& texture,
                     std::int32_t contents) const {
        if (side.texinfo < 0) {
            return 1;
        }
        if (lowercase(texture) == "tools/toolshint") {
            return 2;
        }
        if ((contents & ClipContents) != 0 ||
            lowercase(texture) == "tools/toolsplayerclip" ||
            lowercase(texture) == "tools/toolsnpcclip" ||
            lowercase(texture) == "tools/toolsclip") {
            return 1;
        }
        return 0;
    }

    void addCubemapSide(std::int32_t texinfoIndex, std::uint32_t sideId) {
        if (texinfoIndex < 0 ||
            static_cast<std::size_t>(texinfoIndex) >= bsp_.texinfos.size()) {
            return;
        }
        const auto texdata =
            bsp_.texinfos[static_cast<std::size_t>(texinfoIndex)].texdata;
        if (texdata < 0 ||
            static_cast<std::size_t>(texdata) >= bsp_.textureEnvIndices.size()) {
            return;
        }
        const auto cubeIndex =
            bsp_.textureEnvIndices[static_cast<std::size_t>(texdata)];
        if (cubeIndex < 0 ||
            static_cast<std::size_t>(cubeIndex) >= bsp_.cubemaps.size()) {
            return;
        }
        auto& sides = bsp_.cubemaps[static_cast<std::size_t>(cubeIndex)].sides;
        if (std::find(sides.begin(), sides.end(), sideId) == sides.end()) {
            sides.push_back(sideId);
        }
    }

    void writeBrush(int depth, const Brush& brush, const Vec3& origin,
                    int brushIndex) {
        output_ << std::string(static_cast<std::size_t>(depth), '\t')
                << "solid\n"
                << std::string(static_cast<std::size_t>(depth), '\t') << "{\n";
        keyValue(output_, depth + 1, "id", std::to_string(nextObjectId_++));
        int writtenSides = 0;
        int visibilityGroup = 0;
        for (int sideIndex = 0; sideIndex < brush.sideCount; ++sideIndex) {
            const auto sideArrayIndex =
                static_cast<std::size_t>(brush.firstSide + sideIndex);
            const auto& side = bsp_.brushSides[sideArrayIndex];
            if (side.bevel == 1) {
                continue;
            }
            auto polygon = sidePolygon(bsp_, brush, sideIndex);
            std::array<Vec3, 3> points{};
            if (!choosePlanePoints(polygon, origin, points)) {
                continue;
            }
            const auto sideId = nextSideId_++;
            brushSideIds_[sideArrayIndex] = sideId;
            const auto texture = material(side, brush.contents);
            const auto& plane = bsp_.planes[side.plane];
            const int group = surfaceGroup(side, texture, brush.contents);
            if (group > 0) {
                visibilityGroup = group;
            }
            const auto sideDepth = depth + 1;
            output_ << std::string(static_cast<std::size_t>(sideDepth), '\t')
                    << "side\n"
                    << std::string(static_cast<std::size_t>(sideDepth), '\t')
                    << "{\n";
            keyValue(output_, sideDepth + 1, "id", std::to_string(sideId));
            keyValue(output_, sideDepth + 1, "plane",
                     planePoint(points[0]) + " " + planePoint(points[1]) + " " +
                         planePoint(points[2]));
            keyValue(output_, sideDepth + 1, "material", texture);
            keyValue(output_, sideDepth + 1, "uaxis",
                     axisForPlane(side.texinfo, 0, plane.normal, origin));
            keyValue(output_, sideDepth + 1, "vaxis",
                     axisForPlane(side.texinfo, 1, plane.normal, origin));
            keyValue(output_, sideDepth + 1, "rotation", "0");
            int lightmapScale = 16;
            if (side.texinfo >= 0 &&
                static_cast<std::size_t>(side.texinfo) < bsp_.texinfos.size()) {
                lightmapScale =
                    bsp_.texinfos[static_cast<std::size_t>(side.texinfo)]
                        .lightmapScale;
            }
            keyValue(output_, sideDepth + 1, "lightmapscale",
                     std::to_string(lightmapScale));
            keyValue(output_, sideDepth + 1, "smoothing_groups", "0");
            output_ << std::string(static_cast<std::size_t>(sideDepth), '\t')
                    << "}\n";
            addCubemapSide(side.texinfo, sideId);
            ++writtenSides;
        }
        if (writtenSides < 4) {
            std::cerr << "Warning: brush " << brushIndex
                      << " produced fewer than four valid sides\n";
        }
        if (visibilityGroup > 0) {
            output_ << "\t\teditor\n\t\t{\n";
            keyValue(output_, 3, "color", "200 100 100");
            keyValue(output_, 3, "visgroupid",
                     std::to_string(5 + visibilityGroup));
            keyValue(output_, 3, "visgroupshown", "1");
            output_ << "\t\t}\n";
        }
        output_ << std::string(static_cast<std::size_t>(depth), '\t') << "}\n";
    }

    void writeBackface(const Vec3& first, const Vec3& second,
                       const Vec3& third, const std::string& texture,
                       const std::string& uaxis, const std::string& vaxis) {
        output_ << "\t\tside\n\t\t{\n";
        keyValue(output_, 3, "id", std::to_string(nextSideId_++));
        keyValue(output_, 3, "plane",
                 planePoint(first) + " " + planePoint(third) + " " +
                     planePoint(second));
        keyValue(output_, 3, "material", texture);
        keyValue(output_, 3, "uaxis", uaxis);
        keyValue(output_, 3, "vaxis", vaxis);
        keyValue(output_, 3, "rotation", "0");
        keyValue(output_, 3, "lightmapscale", "16");
        keyValue(output_, 3, "smoothing_groups", "0");
        output_ << "\t\t}\n";
    }

    void writeDisplacementBack(const Polygon& polygon, const Vec3& normal,
                               const std::string& texture) {
        const Vec3 extrusion = normal * 4.0;
        const auto extrusionDirection = normalized(extrusion);
        for (std::size_t i = 0; i < polygon.size(); ++i) {
            const auto& first = polygon[i];
            const auto& second = polygon[(i + 1) % polygon.size()];
            const auto edgeDirection = normalized(second - first);
            writeBackface(first, second, first + extrusion, texture,
                          '[' + vectorString(edgeDirection) + " 0] 0.25",
                          '[' + vectorString(extrusionDirection) + " 0] 0.25");
        }

        std::array<Vec3, 3> backPlane{};
        if (choosePlanePoints(polygon, extrusion, backPlane)) {
            Vec3 up{0.0, 0.0, 1.0};
            if (std::abs(up.dot(normal)) > 0.707) {
                up = {1.0, 0.0, 0.0};
            }
            const auto uaxis = normalized(up.cross(normal));
            const auto vaxis = normalized(uaxis.cross(normal));
            writeBackface(backPlane[0], backPlane[1], backPlane[2], texture,
                          '[' + vectorString(uaxis) + " 0] 0.25",
                          '[' + vectorString(vaxis) + " 0] 0.25");
        }
    }

    void writeDispInfo(const DispInfo& info) {
        const auto power = static_cast<unsigned>(info.power);
        const std::size_t size = std::size_t{1} << power;
        const std::size_t vertexCount = (size + 1) * (size + 1);
        const std::size_t triangleCount = 2 * size * size;
        const auto firstVertex = static_cast<std::size_t>(info.firstVertex);
        const auto firstTriangle = static_cast<std::size_t>(info.firstTriangle);
        if (info.firstVertex < 0 || info.firstTriangle < 0 ||
            firstVertex > bsp_.displacementVertices.size() ||
            vertexCount > bsp_.displacementVertices.size() - firstVertex ||
            firstTriangle > bsp_.displacementTriangles.size() ||
            triangleCount >
                bsp_.displacementTriangles.size() - firstTriangle) {
            throw std::runtime_error("displacement references invalid vertex or triangle data");
        }

        output_ << "\t\t\tdispinfo\n\t\t\t{\n";
        keyValue(output_, 4, "power", std::to_string(info.power));
        keyValue(output_, 4, "startposition",
                 '[' + vectorString(info.startPosition) + ']');
        keyValue(output_, 4, "elevation", "0");
        keyValue(output_, 4, "subdiv", "0");
        const auto writeRows = [&](const char* section, std::size_t columns,
                                   const auto& valueAt) {
            output_ << "\t\t\t\t" << section << "\n\t\t\t\t{\n";
            std::string row;
            std::size_t rowIndex = 0;
            for (std::size_t i = 0; i < vertexCount; ++i) {
                row += valueAt(i);
                row += ' ';
                if (i % columns == columns - 1) {
                    keyValue(output_, 5, "row" + std::to_string(rowIndex++), row);
                    row.clear();
                }
            }
            output_ << "\t\t\t\t}\n";
        };
        writeRows("normals", size + 1, [&](std::size_t index) {
            const auto& vertex = bsp_.displacementVertices[firstVertex + index];
            return vectorString(vertex.normal);
        });
        writeRows("distances", size + 1, [&](std::size_t index) {
            return formatNumber(
                bsp_.displacementVertices[firstVertex + index].distance);
        });
        writeRows("offset_normals", size + 1,
                  [](std::size_t) { return std::string("0 0 0"); });
        writeRows("offsets", size + 1,
                  [](std::size_t) { return std::string("0 0 0"); });
        writeRows("alphas", size + 1, [&](std::size_t index) {
            return formatNumber(
                bsp_.displacementVertices[firstVertex + index].alpha);
        });

        static constexpr std::array<const char*, 7> triangleTags{
            "0", "0", "1", "0", "0", "0", "9"};
        output_ << "\t\t\t\ttriangle_tags\n\t\t\t\t{\n";
        for (std::size_t row = 0; row < size; ++row) {
            std::string values;
            for (std::size_t column = 0; column < 2 * size; ++column) {
                auto tag =
                    bsp_.displacementTriangles[firstTriangle + row * 2 * size +
                                               column];
                if (tag >= triangleTags.size()) {
                    tag = 0;
                }
                values += triangleTags[tag];
                values += ' ';
            }
            keyValue(output_, 5, "row" + std::to_string(row), values);
        }
        output_ << "\t\t\t\t}\n\t\t\t\tallowed_verts\n\t\t\t\t{\n";
        std::string allowed;
        for (const auto vertex : info.allowedVertices) {
            allowed += std::to_string(vertex) + ' ';
        }
        keyValue(output_, 5, "10", allowed);
        output_ << "\t\t\t\t}\n\t\t\t}\n";
    }

    void writeDisplacements() {
        for (const auto& face : bsp_.faces) {
            if (face.displacement < 0) {
                continue;
            }
            if (static_cast<std::size_t>(face.displacement) >=
                bsp_.displacements.size()) {
                throw std::runtime_error("face references an invalid displacement");
            }
            const auto polygon = facePolygon(bsp_, face);
            std::array<Vec3, 3> planePoints{};
            if (!choosePlanePoints(polygon, {}, planePoints)) {
                std::cerr << "Warning: skipping a displacement with a degenerate face\n";
                continue;
            }
            const auto normal =
                normalized((planePoints[1] - planePoints[0])
                              .cross(planePoints[2] - planePoints[0]));
            if (normal.length() < 0.99) {
                std::cerr << "Warning: skipping a displacement with an invalid normal\n";
                continue;
            }
            auto& info =
                bsp_.displacements[static_cast<std::size_t>(face.displacement)];
            const auto texture = bsp_.textureName(face.texinfo);
            output_ << "\tsolid\n\t{\n";
            keyValue(output_, 2, "id", std::to_string(nextObjectId_++));
            writeDisplacementBack(polygon, normal, texture);

            info.sideId = static_cast<std::int32_t>(nextSideId_++);
            output_ << "\t\tside\n\t\t{\n";
            keyValue(output_, 3, "id", std::to_string(info.sideId));
            keyValue(output_, 3, "plane",
                     planePoint(planePoints[0]) + " " +
                         planePoint(planePoints[1]) + " " +
                         planePoint(planePoints[2]));
            keyValue(output_, 3, "material", texture);
            keyValue(output_, 3, "uaxis",
                     bsp_.textureAxis(face.texinfo, 0, {}));
            keyValue(output_, 3, "vaxis",
                     bsp_.textureAxis(face.texinfo, 1, {}));
            keyValue(output_, 3, "rotation", "0");
            int lightmapScale = 16;
            if (face.texinfo >= 0 &&
                static_cast<std::size_t>(face.texinfo) < bsp_.texinfos.size()) {
                lightmapScale =
                    bsp_.texinfos[static_cast<std::size_t>(face.texinfo)]
                        .lightmapScale;
            }
            keyValue(output_, 3, "lightmapscale",
                     std::to_string(lightmapScale));
            keyValue(output_, 3, "smoothing_groups",
                     std::to_string(face.smoothingGroups));
            writeDispInfo(info);
            output_ << "\t\t}\n\t}\n";
        }
    }

    void writeDetails() {
        for (const auto brushIndex : bsp_.brushesForModel(0)) {
            const auto& brush = bsp_.brushes[brushIndex];
            if (brush.contents != (1 | DetailContents)) {
                continue;
            }
            output_ << "entity\n{\n";
            keyValue(output_, 1, "id", std::to_string(nextObjectId_++));
            keyValue(output_, 1, "classname", "func_detail");
            writeBrush(1, brush, {}, static_cast<int>(brushIndex));
            output_ << "}\n";
        }
    }

    int overlaySide(std::int32_t faceIndex) const {
        if (faceIndex < 0 ||
            static_cast<std::size_t>(faceIndex) >= bsp_.faces.size()) {
            return -1;
        }
        const auto& face = bsp_.faces[static_cast<std::size_t>(faceIndex)];
        if (face.displacement >= 0) {
            if (static_cast<std::size_t>(face.displacement) >=
                bsp_.displacements.size()) {
                return -1;
            }
            return bsp_.displacements[static_cast<std::size_t>(face.displacement)]
                .sideId;
        }
        std::vector<Polygon> targetPolygons;
        if (face.originalFace >= 0 &&
            static_cast<std::size_t>(face.originalFace) <
                bsp_.originalFaces.size()) {
            auto original = facePolygon(
                bsp_, bsp_.originalFaces[static_cast<std::size_t>(
                          face.originalFace)]);
            if (original.size() >= 3) {
                targetPolygons.push_back(std::move(original));
            }
        }
        auto current = facePolygon(bsp_, face);
        if (current.size() >= 3) {
            targetPolygons.push_back(std::move(current));
        }
        if (targetPolygons.empty()) {
            return -1;
        }

        for (std::size_t brushIndex = 0; brushIndex < bsp_.brushes.size();
             ++brushIndex) {
            const auto& brush = bsp_.brushes[brushIndex];
            for (int sideIndex = 0; sideIndex < brush.sideCount; ++sideIndex) {
                const auto sideIndexGlobal =
                    static_cast<std::size_t>(brush.firstSide + sideIndex);
                const auto& side = bsp_.brushSides[sideIndexGlobal];
                if (side.plane != face.plane || side.texinfo != face.texinfo ||
                    brushSideIds_[sideIndexGlobal] == 0) {
                    continue;
                }
                const auto polygon =
                    sidePolygon(bsp_, brush, sideIndex, false);
                for (const auto& target : targetPolygons) {
                    if (windingsMatch(polygon, target)) {
                        return static_cast<int>(
                            brushSideIds_[sideIndexGlobal]);
                    }
                }
            }
        }

        if (face.plane >= bsp_.planes.size()) {
            return -1;
        }
        const auto& plane = bsp_.planes[face.plane];
        for (std::size_t brushIndex = 0; brushIndex < bsp_.brushes.size();
             ++brushIndex) {
            const auto& brush = bsp_.brushes[brushIndex];
            for (int sideIndex = 0; sideIndex < brush.sideCount; ++sideIndex) {
                const auto sideIndexGlobal =
                    static_cast<std::size_t>(brush.firstSide + sideIndex);
                const auto& side = bsp_.brushSides[sideIndexGlobal];
                if (side.plane != face.plane || side.texinfo != face.texinfo ||
                    brushSideIds_[sideIndexGlobal] == 0) {
                    continue;
                }
                const auto polygon =
                    sidePolygon(bsp_, brush, sideIndex, false);
                for (const auto& target : targetPolygons) {
                    if (polygonContains(polygon, target, plane)) {
                        return static_cast<int>(
                            brushSideIds_[sideIndexGlobal]);
                    }
                }
            }
        }
        return -1;
    }

    void writeOverlays() {
        for (std::size_t i = 0; i < bsp_.overlays.size(); ++i) {
            const auto& overlay = bsp_.overlays[i];
            output_ << "entity\n{\n";
            keyValue(output_, 1, "id", std::to_string(nextObjectId_++));
            keyValue(output_, 1, "classname", "info_overlay");
            keyValue(output_, 1, "material", bsp_.textureName(overlay.texinfo));
            keyValue(output_, 1, "StartU", formatNumber(overlay.startU));
            keyValue(output_, 1, "EndU", formatNumber(overlay.endU));
            keyValue(output_, 1, "StartV", formatNumber(overlay.startV));
            keyValue(output_, 1, "EndV", formatNumber(overlay.endV));
            keyValue(output_, 1, "BasisOrigin", vectorString(overlay.origin));
            keyValue(output_, 1, "BasisU", vectorString(overlay.basisU));
            keyValue(output_, 1, "BasisV", vectorString(overlay.basisV));
            keyValue(output_, 1, "BasisNormal", vectorString(overlay.normal));
            for (std::size_t uv = 0; uv < overlay.uv.size(); ++uv) {
                keyValue(output_, 1, "uv" + std::to_string(uv),
                         vectorString(overlay.uv[uv]));
            }
            keyValue(output_, 1, "RenderOrder",
                     std::to_string((overlay.faceCount & 0xc000) >> 14));
            const std::size_t faceCount = overlay.faceCount & 0x3fff;
            std::set<int> sides;
            for (std::size_t face = 0;
                 face < std::min(faceCount, overlay.faces.size()); ++face) {
                const auto side = overlaySide(overlay.faces[face]);
                if (side >= 0) {
                    sides.insert(side);
                }
            }
            std::string sideList;
            for (const auto side : sides) {
                sideList += std::to_string(side) + ' ';
            }
            keyValue(output_, 1, "sides", sideList);
            keyValue(output_, 1, "origin", vectorString(overlay.origin));
            output_ << "}\n";
        }
    }

    void writeStaticProps() {
        for (const auto& prop : bsp_.staticProps) {
            if (prop.model >= bsp_.staticPropModels.size()) {
                throw std::runtime_error("static prop references an invalid model");
            }
            output_ << "entity\n{\n";
            keyValue(output_, 1, "id", std::to_string(nextObjectId_++));
            keyValue(output_, 1, "classname", "prop_static");
            keyValue(output_, 1, "origin", vectorString(prop.origin));
            keyValue(output_, 1, "angles", vectorString(prop.angles));
            keyValue(output_, 1, "skin", std::to_string(prop.skin));
            keyValue(output_, 1, "fademindist", formatNumber(prop.fadeMin));
            keyValue(output_, 1, "fademaxdist", formatNumber(prop.fadeMax));
            keyValue(output_, 1, "solid", std::to_string(prop.solid));
            keyValue(output_, 1, "model", bsp_.staticPropModels[prop.model]);
            output_ << "}\n";
        }
    }

    void writeCubemaps() {
        for (const auto& cubemap : bsp_.cubemaps) {
            output_ << "entity\n{\n";
            keyValue(output_, 1, "id", std::to_string(nextObjectId_++));
            keyValue(output_, 1, "classname", "env_cubemap");
            keyValue(output_, 1, "origin", vectorString(cubemap.origin));
            keyValue(output_, 1, "cubemapsize", std::to_string(cubemap.size));
            std::string sideList;
            if (!cubemap.sides.empty() && cubemap.sides.size() <= 8) {
                for (const auto side : cubemap.sides) {
                    if (!sideList.empty()) {
                        sideList += ' ';
                    }
                    sideList += std::to_string(side);
                }
            }
            keyValue(output_, 1, "sides", sideList);
            output_ << "}\n";
        }
    }

    void writeTrailer() {
        output_ << "cameras\n{\n";
        keyValue(output_, 1, "activecamera", "-1");
        output_ << "}\ncordon\n{\n";
        keyValue(output_, 1, "mins", "(99999 99999 99999)");
        keyValue(output_, 1, "maxs", "(-99999 -99999 -99999)");
        keyValue(output_, 1, "active", "0");
        output_ << "}\n";
    }

    BspFile& bsp_;
    std::ostream& output_;
    const std::filesystem::path& inputPath_;
    std::vector<std::uint32_t> brushSideIds_;
    std::uint32_t nextObjectId_ = 2;
    std::uint32_t nextSideId_ = 1;
};

} // namespace

void Decompiler::run(const std::filesystem::path& inputPath,
                     const std::filesystem::path& outputPath) {
    auto bsp = BspFile::load(inputPath);
    const auto temporaryPath = outputPath.string() + ".tmp";
    {
        std::ofstream output(temporaryPath, std::ios::binary | std::ios::trunc);
        if (!output) {
            throw std::runtime_error("could not create output file");
        }
        output.imbue(std::locale::classic());
        VmfWriter(bsp, output, inputPath).write();
        output.flush();
        if (!output) {
            throw std::runtime_error("failed while writing VMF output");
        }
    }
    std::error_code error;
    std::filesystem::remove(outputPath, error);
    if (error) {
        std::filesystem::remove(temporaryPath);
        throw std::runtime_error("could not replace existing output file: " +
                                 error.message());
    }
    std::filesystem::rename(temporaryPath, outputPath, error);
    if (error) {
        std::filesystem::remove(temporaryPath);
        throw std::runtime_error("could not finalize output file: " +
                                 error.message());
    }
}

} // namespace bspdecompiler
