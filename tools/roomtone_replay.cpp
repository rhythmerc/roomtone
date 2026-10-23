// roomtone_replay RECORDING [--seconds S] [--probe x,y,z dx,dy,dz count step]:
// plays a recorded session into a Room and prints what it made at the
// recorded queries (S apart, default 2 s), at the end, and at points along a
// line from there.
#include "roomtone/record.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s RECORDING [--seconds S]\n", argv[0]);
    return 2;
  }
  double seconds = 2.0;
  for (int i = 2; i + 1 < argc; ++i)
    if (!std::strcmp(argv[i], "--seconds"))
      seconds = std::atof(argv[i + 1]);
  roomtone::Room room;
  uint32_t frames = 0, restarts = 0, hits = 0, depths = 0;
  int64_t lastPrint = 0;
  roomtone::Vec3 lastPoint;
  bool havePoint = false, multi = false; // the last ask was setPoints (several)
  const auto print = [&](int64_t t, roomtone::Vec3 p) {
    roomtone::Lighting l;
    if (!(multi ? room.lightAt(size_t{0}, l) : room.lightAt(p, l)))
      return;
    std::printf("t %.1f s, point %.2f,%.2f,%.2f, %u frames (%u restarts), %u hits, %u depth images: level %.2f, "
                "mean %.2f, key %.2f,%.2f,%.2f from %.2f,%.2f,%.2f%s, fill %.2f,%.2f,%.2f from %.2f,%.2f,%.2f%s\n  %s\n",
                t / 1e9, p.x, p.y, p.z, frames, restarts, hits, depths, l.level, l.mean, l.lights[0].color[0],
                l.lights[0].color[1], l.lights[0].color[2], l.lights[0].dir.x, l.lights[0].dir.y, l.lights[0].dir.z,
                l.lights[0].placed ? " (placed)" : "", l.lights[1].color[0], l.lights[1].color[1],
                l.lights[1].color[2], l.lights[1].dir.x, l.lights[1].dir.y, l.lights[1].dir.z,
                l.lights[1].placed ? " (placed)" : "", room.summary().c_str());
  };
  int64_t lastTime = 0;
  const bool ok = roomtone::replay(argv[1], room, [&](const roomtone::ReplayEvent& e) {
    switch (e.kind) {
    case roomtone::ReplayEvent::Frame:
      ++frames;
      restarts += e.frame->restarted;
      lastTime = e.timeNs;
      break;
    case roomtone::ReplayEvent::Hit:
      ++hits;
      break;
    case roomtone::ReplayEvent::Depth:
      ++depths;
      break;
    case roomtone::ReplayEvent::Points:
    case roomtone::ReplayEvent::Query:
      multi = e.kind == roomtone::ReplayEvent::Points;
      lastPoint = e.point;
      havePoint = true;
      lastTime = e.timeNs;
      if (e.timeNs - lastPrint >= static_cast<int64_t>(seconds * 1e9)) {
        lastPrint = e.timeNs;
        print(e.timeNs, e.point);
      }
      break;
    default:
      break;
    }
  });
  if (!ok) {
    std::fprintf(stderr, "can't read %s\n", argv[1]);
    return 1;
  }
  room.make();
  std::printf("end:\n");
  print(lastTime, havePoint ? lastPoint : roomtone::Vec3{});
  // --probe x,y,z dx,dy,dz count step: the light at points along a line
  // (tracking space at the recording's end), each made alone: the light
  // from above and toward the key light, as a shader would light them.
  for (int i = 2; i + 4 < argc; ++i) {
    if (std::strcmp(argv[i], "--probe") != 0)
      continue;
    roomtone::Vec3 o, d;
    std::sscanf(argv[i + 1], "%f,%f,%f", &o.x, &o.y, &o.z);
    std::sscanf(argv[i + 2], "%f,%f,%f", &d.x, &d.y, &d.z);
    const int n = std::atoi(argv[i + 3]);
    const float step = static_cast<float>(std::atof(argv[i + 4]));
    const auto eval = [](const roomtone::Lighting& l, roomtone::Vec3 nv, int ch) {
      const float x = nv.x, y = nv.y, z = nv.z;
      const float poly[9] = {1.f, y, z, x, x * y, y * z, 3.f * z * z - 1.f, x * z, x * x - y * y};
      float e = 0.f;
      for (int k = 0; k < 9; ++k)
        e += l.sh[k][ch] * poly[k];
      return e;
    };
    for (int k = 0; k <= n; ++k) {
      const roomtone::Vec3 p{o.x + d.x * step * k, o.y + d.y * step * k, o.z + d.z * step * k};
      roomtone::Lighting l;
      room.setPoints(&p, 1);
      room.make();
      if (!room.lightAt(size_t{0}, l))
        continue;
      const auto& key = l.lights[0];
      const float shUp = eval(l, {0.f, 1.f, 0.f}, 1), shKey = eval(l, key.dir, 1);
      float lit = shUp;
      for (const auto& li : l.lights)
        lit += std::max(li.dir.y, 0.f) * li.color[1];
      std::printf("probe %.2f,%.2f,%.2f: up %.3f (SH %.3f), facing the key SH %.3f + key %.3f%s at %.2f m, "
                  "mean %.2f, level %.2f\n  %s\n",
                  p.x, p.y, p.z, lit, shUp, shKey, key.color[1], key.placed ? " (placed)" : "", key.range, l.mean,
                  l.level, room.summary().substr(0, 260).c_str());
    }
  }
  return 0;
}
