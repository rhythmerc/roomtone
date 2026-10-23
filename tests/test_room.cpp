// A synthetic room: grey walls and a small bright lamp up and ahead. Frames
// sweep the head around it; the key light should come from the lamp.
#include "roomtone/roomtone.hpp"

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

Vec3 norm(Vec3 v) {
  const float n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  return {v.x / n, v.y / n, v.z / n};
}
float dotv(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 rot(const Quat& q, Vec3 v) {
  const Vec3 u{q.x, q.y, q.z};
  const auto cr = [](Vec3 a, Vec3 b) { return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; };
  const Vec3 t = cr(u, v);
  const Vec3 t2{t.x * 2, t.y * 2, t.z * 2};
  const Vec3 c = cr(u, t2);
  return {v.x + t2.x * q.w + c.x, v.y + t2.y * q.w + c.y, v.z + t2.z * q.w + c.z};
}
Quat yaw_pitch(float yaw, float pitch) {
  // yaw about +Y, then pitch about the turned +X
  const Quat qy{0.f, std::sin(yaw / 2), 0.f, std::cos(yaw / 2)};
  const Quat qp{std::sin(pitch / 2), 0.f, 0.f, std::cos(pitch / 2)};
  return {qy.w * qp.x + qy.x * qp.w + qy.y * qp.z - qy.z * qp.y, qy.w * qp.y - qy.x * qp.z + qy.y * qp.w + qy.z * qp.x,
          qy.w * qp.z + qy.x * qp.y - qy.y * qp.x + qy.z * qp.w, qy.w * qp.w - qy.x * qp.x - qy.y * qp.y - qy.z * qp.z};
}

const Vec3 kLamp = norm({0.3f, 0.8f, -0.5f});

// Radiance (camera 0-1 at exposure 10 ms, ISO 100) along a direction.
float radiance(Vec3 d) { return dotv(d, kLamp) > std::cos(0.12f) ? 1.0f : 0.08f + 0.04f * d.y; }

} // namespace

int main() {
  Room room;
  // A Quest 3-like camera: 1280 of 1728, f 1170, looking ahead.
  Camera cam;
  cam.fx = cam.fy = 1170.f;
  cam.cx = cam.cy = 864.f;
  cam.sensorWidth = cam.sensorHeight = 1728;
  cam.width = cam.height = 1280;
  // As on the Quest, the lens pose turns OpenCV's axes (y down, z ahead)
  // into the head's (y up, -z ahead): half a turn about X.
  cam.lensRotation = {1.f, 0.f, 0.f, 0.f};
  room.setCamera(cam);
  constexpr int G = 32;
  std::vector<float> cells(G * G * 4);
  int64_t t = 0;
  for (int k = 0; k < 300; ++k) {
    const float yaw = -1.2f + 2.4f * (k % 30) / 29.f, pitch = -0.4f + 1.4f * (k / 30) / 9.f;
    const Quat head = yaw_pitch(yaw, pitch);
    for (int gy = 0; gy < G; ++gy)
      for (int gx = 0; gx < G; ++gx) {
        const float px = (gx + 0.5f) * 1728.f / G, py = (gy + 0.5f) * 1728.f / G;
        const Vec3 c{(px - cam.cx) / cam.fx, (py - cam.cy) / cam.fy, 1.f};
        const Vec3 inHead = norm(rot({-1.f, 0.f, 0.f, 0.f}, c)); // by the lens rotation's inverse
        const float r = radiance(norm(rot(head, inHead)));
        float* p = &cells[(gy * G + gx) * 4];
        p[0] = p[1] = p[2] = std::min(r, 1.f);
        p[3] = r >= 1.f ? 0.5f : 0.f;
      }
    Frame f{cells.data(), G, G, 10.f, 100.f, {head, {}}, t};
    t += 33'000'000;
    room.addFrame(f);
  }
  Lighting l;
  check(room.lightAt({0.f, -0.3f, -0.7f}, l), "made");
  const float c = dotv(norm(l.lights[0].dir), kLamp);
  std::printf("key %.2f,%.2f,%.2f (lamp %.2f,%.2f,%.2f), cos %.3f, level %.2f\n%s\n", l.lights[0].dir.x,
              l.lights[0].dir.y, l.lights[0].dir.z, kLamp.x, kLamp.y, kLamp.z, c, l.level, room.summary().c_str());
  check(c > 0.95f, "the key light comes from the lamp");
  check(l.lights[0].color[0] > 0.f, "the key light is lit");
  return failures ? 1 : 0;
}
