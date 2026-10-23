// roomtone_replay RECORDING [--every N]: plays a recorded session into a
// Room and prints what it made at each recorded query (every Nth, default
// every query 2 s apart), and at the end.
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
  return 0;
}
