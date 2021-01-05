#ifndef SMPL_MARKER_UTILS_H
#define SMPL_MARKER_UTILS_H

#include <smpl/debug/marker.h>
#include <smpl/spatial.h>

namespace smpl {
namespace visual {

auto MakeEmptyMarker() -> Marker;

auto MakeSphereMarker(
    double x, double y, double z,
    double radius,
    int hue,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0) -> Marker;

auto MakeCubeMarker(
    double x, double y, double z,
    double length, double width, double height,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0) -> Marker;

auto MakeLineMarker(
    const Vector3& a,
    const Vector3& b,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0)
    -> Marker;

auto MakeFrameMarkers(
    const Affine3& pose,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0)
    -> std::vector<Marker>;

auto MakePoseMarkers(
    const Affine3& pose,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0,
    bool text = false)
    -> std::vector<Marker>;

auto MakeCubesMarker(
    const std::vector<Vector3>& centers,
    double size,
    const Color& color,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0) -> Marker;

auto MakeCubesMarker(
    const std::vector<Vector3>& centers,
    double size,
    const std::vector<Color>& colors,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0) -> Marker;

auto MakeCubesMarker(
    std::vector<Vector3>&& centers,
    double size,
    const Color& color,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0) -> Marker;

auto MakeCubesMarker(
    std::vector<Vector3>&& centers,
    double size,
    std::vector<Color>&& colors,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0) -> Marker;

} // namespace visual
} // namespace smpl

#endif
