// The box room of test_depth with a window instead of the lamp: a bright
// patch on a wall that the depth sensor sees through (its glass), framed by
// walls it does see. The window's light is the head's view of it at every
// point (no voxels cover the glass), so it falls off from where the head
// was: a point beside the window gets more of it than one across the room
// (its range from the point cancelled the falloff, 2026-10-23).
#include "roomtone/roomtone.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace roomtone;

namespace {

int failures = 0;
void check(bool ok, const char* what) {
  std::printf("%s: %s\n", ok ? "ok" : "FAIL", what);
  failures += !ok;
}

Vec3 add(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 sub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 mulv(Vec3 a, float k) { return {a.x * k, a.y * k, a.z * k}; }
float dotv(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 norm(Vec3 v) { return mulv(v, 1.f / std::sqrt(dotv(v, v))); }
Vec3 cr(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
Vec3 rot(const Quat& q, Vec3 v) {
  const Vec3 u{q.x, q.y, q.z};
  const Vec3 t = mulv(cr(u, v), 2.f);
  return add(add(v, mulv(t, q.w)), cr(u, t));
}
Quat yaw_pitch(float yaw, float pitch) {
  const Quat a{0.f, std::sin(yaw / 2), 0.f, std::cos(yaw / 2)};
  const Quat b{std::sin(pitch / 2), 0.f, 0.f, std::cos(pitch / 2)};
  return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

// The room: x and z in -2..2, y in 0..2.6; the window 1.2 m square on the
// x = 2 wall, its glass showing a bright outside 20 m away to depth.
float trace(Vec3 o, Vec3 d, float& radiance, bool* glass = nullptr) {
  float t = 1e9f;
  const auto plane = [&](float oc, float dc, float at) {
    if (std::abs(dc) > 1e-6f)
      if (const float s = (at - oc) / dc; s > 1e-4f && s < t)
        t = s;
  };
  plane(o.x, d.x, -2.f), plane(o.x, d.x, 2.f), plane(o.z, d.z, -2.f), plane(o.z, d.z, 2.f);
  plane(o.y, d.y, 0.f), plane(o.y, d.y, 2.6f);
  const Vec3 p = add(o, mulv(d, t));
  if (glass)
    *glass = false;
  if (p.x > 1.99f && std::abs(p.z) < 0.6f && p.y > 0.9f && p.y < 2.1f) {
    radiance = 1.5f;
    if (glass)
      *glass = true;
    return 20.f;
  }
  if (p.y < 0.01f)
    radiance = 0.04f;
  else
    radiance = 0.09f;
  return t;
}

} // namespace

int main() {
  Room room;
  Camera cam;
  cam.fx = cam.fy = 1170.f;
  cam.cx = cam.cy = 864.f;
  cam.sensorWidth = cam.sensorHeight = 1728;
  cam.width = cam.height = 1280;
  cam.lensRotation = {1.f, 0.f, 0.f, 0.f}; // OpenCV's axes into the head's
  room.setCamera(cam);
  const Vec3 headAt{0.f, 1.5f, 0.5f};
  constexpr int G = 32, D = 160;
  const float tanD = std::tan(0.8f);
  std::vector<float> cells(G * G * 4);
  std::vector<uint16_t> depth(D * D);
  int64_t t = 0;
  for (int k = 0; k < 400; ++k) {
    const float yaw = -2.6f + 5.2f * (k % 40) / 39.f, pitch = -0.6f + 1.6f * (k / 40) / 9.f;
    const Quat head = yaw_pitch(yaw, pitch);
    // The depth image, from the head (rows from the bottom).
    for (int y = 0; y < D; ++y)
      for (int x = 0; x < D; ++x) {
        const float u = -tanD + 2.f * tanD * (x + 0.5f) / D, v = -tanD + 2.f * tanD * (y + 0.5f) / D;
        const Vec3 dv = norm({u, v, -1.f});
        float r;
        const float dist = trace(headAt, rot(head, dv), r);
        const float z = dist * -dv.z; // along the view's axis
        depth[y * D + x] = static_cast<uint16_t>(std::lround((1.f - 0.1f / z) * 65535.f));
      }
    DepthImage di;
    di.depth = depth.data();
    di.width = di.height = di.stride = D;
    di.nearZ = 0.1f;
    di.farZ = INFINITY;
    di.tanLeft = di.tanDown = -tanD;
    di.tanRight = di.tanUp = tanD;
    di.view = {head, headAt};
    di.timeNs = t;
    room.addDepth(di);
    for (int gy = 0; gy < G; ++gy)
      for (int gx = 0; gx < G; ++gx) {
        const float px = (gx + 0.5f) * 1728.f / G, py = (gy + 0.5f) * 1728.f / G;
        const Vec3 c{(px - cam.cx) / cam.fx, (py - cam.cy) / cam.fy, 1.f};
        const Vec3 inHead = norm(rot({-1.f, 0.f, 0.f, 0.f}, c));
        float r;
        trace(headAt, rot(head, inHead), r);
        float* p = &cells[(gy * G + gx) * 4];
        p[0] = p[1] = p[2] = std::min(r, 1.f);
        p[3] = r >= 1.f ? 0.5f : 0.f;
      }
    Frame f{cells.data(), G, G, 10.f, 100.f, {head, headAt}, t};
    t += 50'000'000;
    room.addFrame(f);
  }
  const Vec3 nearWindow{1.3f, 0.8f, 0.f}, across{-1.5f, 0.8f, 0.f};
  Lighting a, b;
  room.lightAt(nearWindow, a);
  room.make();
  check(room.lightAt(nearWindow, a), "made");
  std::printf("%zu voxels\n%s\n", room.voxels(), room.summary().c_str());
  room.lightAt(across, b);
  room.make();
  check(room.lightAt(across, b), "made across the room");
  std::printf("%s\n", room.summary().c_str());
  // What a surface facing the window gets (+x), and one facing up: the SH
  // there and both lights, as a shader lights them (a window near enough
  // fills much of the sphere, so its light is in the SH as much as in a key).
  const auto facing = [](const Lighting& l, Vec3 n) {
    const float poly[9] = {1.f, n.y, n.z, n.x, n.x * n.y, n.y * n.z, 3.f * n.z * n.z - 1.f, n.x * n.z,
                           n.x * n.x - n.y * n.y};
    float e = 0.f;
    for (int k = 0; k < 9; ++k)
      e += l.sh[k][1] * poly[k];
    for (const auto& li : l.lights)
      e += std::max(dotv(li.dir, n), 0.f) * li.color[1];
    return e;
  };
  const Vec3 toWindow{1.f, 0.f, 0.f}, up{0.f, 1.f, 0.f};
  std::printf("facing the window: %.3f beside it, %.3f across the room; facing up: %.3f, %.3f (head 1)\n",
              facing(a, toWindow), facing(b, toWindow), facing(a, up), facing(b, up));
  check(facing(a, toWindow) > 2.f * facing(b, toWindow), "the window lights a surface facing it less across the room");
  check(facing(a, up) > facing(b, up), "and from above");
  return failures ? 1 : 0;
}
