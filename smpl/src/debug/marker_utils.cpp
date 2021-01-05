#include <smpl/debug/marker_utils.h>

#include <smpl/debug/colors.h>

namespace smpl {
namespace visual {

auto MakeEmptyMarker() -> Marker {
    visual::Marker m;
    m.shape = Empty();
    return m;
}

auto MakeSphereMarker(
    double x, double y, double z,
    double radius,
    int hue,
    const std::string& frame_id,
    const std::string& ns,
    int id) -> Marker
{
    visual::Marker m;
    m.pose = Affine3(Translation3(x, y, z));
    m.shape = Sphere{ radius };
    m.color = MakeColorHSV(hue);
    m.frame_id = frame_id;
    m.ns = ns;
    m.id = id;

    return m;
}

auto MakeCubeMarker(
    double x, double y, double z,
    double length, double width, double height,
    const std::string& frame_id,
    const std::string& ns,
    int id) -> Marker
{
    visual::Marker m;
    m.pose = Affine3(Translation3(x, y, z));
    m.shape = Cube{length, width, height};
    m.color = Color{200.0/255, 184.0/255, 124.0/255, 1.0f};
    m.frame_id = frame_id;
    m.ns = ns;
    m.id = id;

    return m;
}

auto MakeLineMarker(
    const Vector3& a,
    const Vector3& b,
    const std::string& frame_id,
    const std::string& ns,
    int id)
    -> Marker
{
    Marker m;
    m.pose = Affine3::Identity();
    m.shape = LineList{ { a, b } };
    m.color = Color{ 1.0f, 1.0f, 1.0f, 1.0f };
    m.frame_id = frame_id;
    m.ns = ns;
    m.id = id;
    m.lifetime = 0.0;
    return std::move(m);
}

auto MakeFrameMarkers(
    const Affine3& pose,
    const std::string& frame_id,
    const std::string& ns,
    int id)
    -> std::vector<Marker>
{
    std::vector<Marker> markers(3);

    markers[0].pose = pose;
    markers[0].shape = Arrow{ 0.1, 0.01 };
    markers[0].color = Color{ 1.0f, 0.0f, 0.0f, 1.0f };
    markers[0].frame_id = frame_id;
    markers[0].ns = ns;
    markers[0].id = id;
    markers[0].lifetime = 0.0;

    markers[1].pose = pose * AngleAxis(0.5 * M_PI, Vector3::UnitZ());
    markers[1].shape = Arrow{ 0.1, 0.01 };
    markers[1].color = Color{ 0.0f, 1.0f, 0.0f, 1.0f };
    markers[1].frame_id = frame_id;
    markers[1].ns = ns;
    markers[1].id = id + 1;
    markers[1].lifetime = 0.0;

    markers[2].pose = pose * AngleAxis(-0.5 * M_PI, Vector3::UnitY());
    markers[2].shape = Arrow{ 0.1, 0.01 };
    markers[2].color = Color{ 0.0f, 0.0f, 1.0f, 1.0f };
    markers[2].frame_id = frame_id;
    markers[2].ns = ns;
    markers[2].id = id + 2;
    markers[2].lifetime = 0.0;

    return std::move(markers);
}

auto MakePoseMarkers(
    const Affine3& pose,
    const std::string& frame_id,
    const std::string& ns,
    int id,
    bool text)
    -> std::vector<Marker>
{
    std::vector<Marker> markers(2);

    markers[0].pose = pose;
    markers[0].shape = Arrow{ 0.1, 0.015 };
    markers[0].color = Color{ 0.0f, 0.7f, 0.6f, 0.7f };
    markers[0].frame_id = frame_id;
    markers[0].ns = ns;
    markers[0].id = id;

    markers[1].pose = pose;
    markers[1].shape = Ellipse{ 0.07, 0.07, 0.1 };
    markers[1].color = Color{ };
    markers[1].frame_id = frame_id;
    markers[1].ns = ns;
    markers[1].id = id + 1;

    return markers;
}

auto MakeCubesMarker(
    const std::vector<Vector3>& centers,
    double size,
    const Color& color,
    const std::string& frame_id,
    const std::string& ns,
    int id) -> Marker
{
    visual::Marker marker;
    marker.pose = Affine3::Identity();
    marker.shape = CubeList{ centers, size };
    marker.color = color;
    marker.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    return marker;
}

auto MakeCubesMarker(
    const std::vector<Vector3>& centers,
    double size,
    const std::vector<Color>& colors,
    const std::string& frame_id,
    const std::string& ns,
    int id) -> Marker
{
    visual::Marker marker;
    marker.pose = Affine3::Identity();
    marker.shape = CubeList{ centers, size };
    marker.color = Colors{ colors };
    marker.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    return marker;
}

auto MakeCubesMarker(
    std::vector<Vector3>&& centers,
    double size,
    const Color& color,
    const std::string& frame_id,
    const std::string& ns,
    int id) -> Marker
{
    visual::Marker marker;
    marker.pose = Affine3::Identity();
    marker.shape = CubeList{ std::move(centers), size };
    marker.color = color;
    marker.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    return marker;
}

auto MakeCubesMarker(
    std::vector<Vector3>&& centers,
    double size,
    std::vector<Color>&& colors,
    const std::string& frame_id,
    const std::string& ns,
    int id) -> Marker
{
    visual::Marker marker;
    marker.pose = Affine3::Identity();
    marker.shape = CubeList{ std::move(centers), size };
    marker.color = Colors{ std::move(colors) };
    marker.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    return marker;
}

} // namespace visual
} // namespace smpl
