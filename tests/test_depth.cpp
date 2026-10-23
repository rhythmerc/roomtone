// A synthetic box room (4 x 2.6 x 4 m, grey walls, a darker floor) with a
// ceiling lamp off to one side, seen by a camera and a depth sensor from a
// head that looks around. With depth, the key light gets the lamp's place,
// and the light at a point on a table comes from the lamp as seen from
// there, not from the head.
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

// The room: x and z in -2..2, y in 0..2.6; the lamp a 0.2 m disc on the ceiling.
const Vec3 kLamp{1.2f, 2.6f, -1.2f};
float trace(Vec3 o, Vec3 d, float& radiance) {
  float t = 1e9f;
  const auto plane = [&](float oc, float dc, float at) {
    if (std::abs(dc) > 1e-6f)
      if (const float s = (at - oc) / dc; s > 1e-4f && s < t)
        t = s;
  };
  plane(o.x, d.x, -2.f), plane(o.x, d.x, 2.f), plane(o.z, d.z, -2.f), plane(o.z, d.z, 2.f);
  plane(o.y, d.y, 0.f), plane(o.y, d.y, 2.6f);
  const Vec3 p = add(o, mulv(d, t));
  if (p.y > 2.59f && std::hypot(p.x - kLamp.x, p.z - kLamp.z) < 0.2f)
    radiance = 1.5f;
  else if (p.y < 0.01f)
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
  // A point on a table ahead of the head, and one right under the lamp.
  const Vec3 table{0.f, 0.8f, -0.6f}, under{1.2f, 0.8f, -1.2f};
  Lighting a, b;
  room.lightAt(table, a);
  room.make();
  check(room.lightAt(table, a), "made");
  std::printf("%zu voxels\n%s\n", room.voxels(), room.summary().c_str());
  const Light& ka = a.lights[0];
  std::printf("at the table: key %.3f from %.2f,%.2f,%.2f%s at %.2f,%.2f,%.2f\n", ka.color[0], ka.dir.x, ka.dir.y,
              ka.dir.z, ka.placed ? " placed" : "", ka.pos.x, ka.pos.y, ka.pos.z);
  check(ka.placed, "the key light has a place");
  check(std::sqrt(dotv(sub(ka.pos, kLamp), sub(ka.pos, kLamp))) < 0.25f, "at the lamp");
  const Vec3 want = norm(sub(kLamp, table));
  check(dotv(ka.dir, want) > 0.99f, "lighting the table from the lamp's direction there");
  room.lightAt(under, b);
  check(room.stale(), "stale for a moved point");
  room.make();
  check(room.lightAt(under, b), "made under the lamp");
  const Light& kb = b.lights[0];
  std::printf("under the lamp: key %.3f from %.2f,%.2f,%.2f\n", kb.color[0], kb.dir.x, kb.dir.y, kb.dir.z);
  check(kb.dir.y > 0.98f, "from straight above, under it");
  // Brighter nearer the lamp, in the reference's (the head's) units: the
  // light on an up-facing surface, the mean light and the key light itself
  // (normalized at the point, the falloff cancelled: farther from a light
  // the rest brightened, 2026-10-22).
  const auto up = [](const Lighting& l) {
    float e = l.sh[0][1] + l.sh[1][1] - l.sh[6][1] - l.sh[8][1];
    for (const auto& li : l.lights)
      e += std::max(li.dir.y, 0.f) * li.color[1];
    return e;
  };
  Lighting c;
  const Vec3 far{-1.6f, 0.8f, 1.6f};
  room.lightAt(far, c);
  room.make();
  room.lightAt(far, c);
  std::printf("light from above: %.2f under the lamp, %.2f at the table, %.2f far (head 1); key %.3f, %.3f, %.3f\n",
              up(b), up(a), up(c), kb.color[1], ka.color[1], c.lights[0].color[1]);
  check(up(b) > up(a) && up(a) > up(c), "brighter nearer the lamp");
  check(kb.color[1] > ka.color[1] && ka.color[1] > c.lights[0].color[1], "the key light falls off with distance");
  check(a.mean == b.mean && b.mean == c.mean, "the fighters' reference (mean) stays the head's");
  // What a make costs from a moved point (the XR thread pays it as the stage moves).
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < 20; ++i) {
    room.lightAt({0.1f * (i % 2), 0.8f, -0.6f}, a);
    room.make();
  }
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / 20;
  std::printf("a make from a moved point: %.2f ms (%zu voxels)\n", ms, room.voxels());
  return failures ? 1 : 0;
}
