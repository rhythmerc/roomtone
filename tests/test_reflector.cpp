// What lies below the content is a lit surface, not a light. A synthetic box
// room (4 x 2.6 x 4 m) with a ceiling lamp, seen with depth by a seated head
// looking around and down: a sunlit patch of carpet beside the content (a
// white sheet, a lap in the light) was taken for a lamp and placed on the
// floor, and as a point light there lit what stood near it 10-100 times over
// (2026-11-17, a hotspot on a pool's floor). It stays in the soft light, as
// light from below; a screen on the wall level with the content, or a little
// below it, is still a light, placed.
#include "roomtone/roomtone.hpp"

#include <algorithm>
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

const Vec3 kLamp{-1.2f, 2.6f, 1.2f};
const Vec3 kPatch{0.7f, 0.f, -0.6f}; // on the floor, beside the content
const Vec3 kContent{0.f, 0.3f, -0.6f};

struct Scene {
  bool patch = false;  // the sunlit carpet
  float screenY = -1.f; // a screen's middle on the x = 2 wall (0.5 x 0.3 m), none if < 0
  Vec3 at = kContent;   // where the light is made
};

// Distance along d from o, and the radiance there.
float trace(const Scene& s, Vec3 o, Vec3 d, float& radiance) {
  float t = 1e9f;
  const auto plane = [&](float oc, float dc, float at) {
    if (std::abs(dc) > 1e-6f)
      if (const float k = (at - oc) / dc; k > 1e-4f && k < t)
        t = k;
  };
  plane(o.x, d.x, -2.f), plane(o.x, d.x, 2.f), plane(o.z, d.z, -2.f), plane(o.z, d.z, 2.f);
  plane(o.y, d.y, 0.f), plane(o.y, d.y, 2.6f);
  const Vec3 p = add(o, mulv(d, t));
  if (p.y > 2.59f && std::hypot(p.x - kLamp.x, p.z - kLamp.z) < 0.2f)
    radiance = 1.5f;
  else if (s.patch && p.y < 0.01f && std::hypot(p.x - kPatch.x, p.z - kPatch.z) < 0.35f)
    radiance = 0.8f;
  else if (s.screenY > 0.f && p.x > 1.99f && std::abs(p.z - kContent.z) < 0.25f && std::abs(p.y - s.screenY) < 0.15f)
    radiance = 1.2f;
  else if (p.y < 0.01f)
    radiance = 0.04f;
  else
    radiance = 0.09f;
  return t;
}

// The light at the content, the head seated at the room's middle looking
// around and down at it.
Lighting light(const Scene& s, std::string* summary = nullptr) {
  Room room;
  Camera cam;
  cam.fx = cam.fy = 1170.f;
  cam.cx = cam.cy = 864.f;
  cam.sensorWidth = cam.sensorHeight = 1728;
  cam.width = cam.height = 1280;
  cam.lensRotation = {1.f, 0.f, 0.f, 0.f};
  room.setCamera(cam);
  room.setPoints(&s.at, 1);
  const Vec3 headAt{0.f, 1.1f, 0.6f};
  constexpr int G = 32, D = 160;
  const float tanD = std::tan(0.8f);
  std::vector<float> cells(G * G * 4);
  std::vector<uint16_t> depth(D * D);
  int64_t t = 0;
  for (int k = 0; k < 400; ++k) {
    const float yaw = -2.6f + 5.2f * (k % 40) / 39.f, pitch = -1.1f + 2.0f * (k / 40) / 9.f;
    const Quat head = yaw_pitch(yaw, pitch);
    for (int y = 0; y < D; ++y)
      for (int x = 0; x < D; ++x) {
        const float u = -tanD + 2.f * tanD * (x + 0.5f) / D, v = -tanD + 2.f * tanD * (y + 0.5f) / D;
        const Vec3 dv = norm({u, v, -1.f});
        float r;
        const float z = trace(s, headAt, rot(head, dv), r) * -dv.z;
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
        float r;
        trace(s, headAt, rot(head, norm(rot({-1.f, 0.f, 0.f, 0.f}, c))), r);
        float* p = &cells[(gy * G + gx) * 4];
        p[0] = p[1] = p[2] = std::min(r, 1.f);
        p[3] = r >= 1.f ? 0.5f : 0.f;
      }
    Frame f{cells.data(), G, G, 10.f, 100.f, {head, headAt}, t};
    t += 50'000'000;
    room.addFrame(f);
  }
  room.make();
  Lighting l;
  room.lightAt(size_t{0}, l);
  if (summary)
    *summary = room.summary();
  return l;
}

float shAt(const Lighting& l, Vec3 n) {
  const float x = n.x, y = n.y, z = n.z;
  const float poly[9] = {1.f, y, z, x, x * y, y * z, 3.f * z * z - 1.f, x * z, x * x - y * y};
  float e = 0.f;
  for (int k = 0; k < 9; ++k)
    e += l.sh[k][1] * poly[k];
  return e;
}

void print(const char* what, const Lighting& l, const std::string& summary) {
  std::printf("%s: soft light from below %.3f, from above %.3f\n", what, shAt(l, {0.f, -1.f, 0.f}),
              shAt(l, {0.f, 1.f, 0.f}));
  for (int i = 0; i < 2; ++i) {
    const Light& k = l.lights[i];
    std::printf("  light %d: %.3f from %.2f,%.2f,%.2f%s at %.2f,%.2f,%.2f\n", i, k.color[1], k.dir.x, k.dir.y, k.dir.z,
                k.placed ? " placed" : "", k.pos.x, k.pos.y, k.pos.z);
  }
  std::printf("  %s\n", summary.substr(0, 400).c_str());
}

// The light below the content's horizon (a light, not the soft light).
bool lightBelow(const Lighting& l) {
  for (const Light& k : l.lights)
    if (k.color[1] > 0.f && k.dir.y < -0.1f)
      return true;
  return false;
}

} // namespace

int main() {
  std::string sum;
  const Lighting plain = light({}, &sum);
  print("the room", plain, sum);
  const Lighting patch = light({true}, &sum);
  print("a sunlit patch of carpet beside the content", patch, sum);
  check(!lightBelow(patch), "the patch is no light");
  bool lamp = false;
  for (const Light& k : patch.lights)
    lamp |= k.color[1] > 0.05f && k.placed && k.pos.y > 2.4f;
  check(lamp, "the lamp is still a light, on the ceiling");
  check(shAt(patch, {0.f, -1.f, 0.f}) > 1.3f * shAt(plain, {0.f, -1.f, 0.f}),
        "its light is in the soft light, from below");
  // A screen on the wall 2 m away: level with the content, 5 degrees below
  // it, and 40 degrees below a point high in the room (it stands on a wall:
  // a light, though below).
  const struct {
    const char* what;
    float y;
    Vec3 at;
  } screens[] = {{"a screen level with the content", 0.3f, kContent},
                 {"a screen a little below it", 0.12f, kContent},
                 {"a screen well below a point high in the room", 0.5f, {0.8f, 1.5f, -0.6f}}};
  for (const auto& sc : screens) {
    const Lighting screen = light({false, sc.y, sc.at}, &sum);
    print(sc.what, screen, sum);
    bool found = false;
    for (const Light& k : screen.lights)
      found |= k.color[1] > 0.f && k.placed && k.pos.x > 1.8f && std::abs(k.pos.y - sc.y) < 0.2f;
    check(found, "the screen is a light, placed on the wall");
  }
  return failures ? 1 : 0;
}
