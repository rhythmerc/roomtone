/* The C interface, from C: test_room's synthetic room (grey walls, a lamp up
 * and ahead) through roomtone.h, recorded and replayed into a second room,
 * which should make the same light. */
#include "roomtone/roomtone.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;
static void check(int ok, const char* what) {
  printf("%s: %s\n", ok ? "ok" : "FAIL", what);
  failures += !ok;
}

static roomtone_vec3 norm(roomtone_vec3 v) {
  const float n = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
  const roomtone_vec3 out = {v.x / n, v.y / n, v.z / n};
  return out;
}
static float dotv(roomtone_vec3 a, roomtone_vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static roomtone_vec3 cross(roomtone_vec3 a, roomtone_vec3 b) {
  const roomtone_vec3 out = {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
  return out;
}
static roomtone_vec3 rot(roomtone_quat q, roomtone_vec3 v) {
  const roomtone_vec3 u = {q.x, q.y, q.z};
  roomtone_vec3 t = cross(u, v);
  t.x *= 2, t.y *= 2, t.z *= 2;
  const roomtone_vec3 c = cross(u, t);
  const roomtone_vec3 out = {v.x + t.x * q.w + c.x, v.y + t.y * q.w + c.y, v.z + t.z * q.w + c.z};
  return out;
}
static roomtone_quat yaw_pitch(float yaw, float pitch) {
  const roomtone_quat a = {0.f, sinf(yaw / 2), 0.f, cosf(yaw / 2)};
  const roomtone_quat b = {sinf(pitch / 2), 0.f, 0.f, cosf(pitch / 2)};
  const roomtone_quat out = {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                             a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
  return out;
}

#define G 32

int main(int argc, char** argv) {
  const char* path = argc > 1 ? argv[1] : "test_c_api.roomtone";
  const roomtone_vec3 lamp = norm((roomtone_vec3){0.3f, 0.8f, -0.5f});
  const roomtone_vec3 content = {0.f, -0.3f, -0.7f};
  static float cells[G * G * 4];

  roomtone_room* room = roomtone_room_new();
  roomtone_recorder* rec = roomtone_recorder_open(path);
  check(rec != NULL, "the recording opens");
  if (!rec)
    return 1;

  roomtone_camera cam;
  memset(&cam, 0, sizeof cam);
  cam.fx = cam.fy = 1170.f;
  cam.cx = cam.cy = 864.f;
  cam.sensor_width = cam.sensor_height = 1728;
  cam.width = cam.height = 1280;
  cam.lens_rotation = (roomtone_quat){1.f, 0.f, 0.f, 0.f};
  check(!roomtone_room_has_camera(room), "no camera yet");
  roomtone_room_set_camera(room, &cam);
  roomtone_recorder_camera(rec, &cam);
  check(roomtone_room_has_camera(room), "a camera");

  roomtone_room_set_points(room, &content, 1);
  roomtone_recorder_points(rec, &content, 1, 0);

  int64_t t = 0;
  for (int k = 0; k < 300; ++k) {
    const float yaw = -1.2f + 2.4f * (k % 30) / 29.f, pitch = -0.4f + 1.4f * (k / 30) / 9.f;
    roomtone_frame f;
    memset(&f, 0, sizeof f);
    f.head.orientation = yaw_pitch(yaw, pitch);
    for (int gy = 0; gy < G; ++gy)
      for (int gx = 0; gx < G; ++gx) {
        const float px = (gx + 0.5f) * 1728.f / G, py = (gy + 0.5f) * 1728.f / G;
        const roomtone_vec3 c = {(px - cam.cx) / cam.fx, (py - cam.cy) / cam.fy, 1.f};
        const roomtone_vec3 in_head = norm(rot((roomtone_quat){-1.f, 0.f, 0.f, 0.f}, c));
        const roomtone_vec3 d = norm(rot(f.head.orientation, in_head));
        const float r = dotv(d, lamp) > cosf(0.12f) ? 1.0f : 0.08f + 0.04f * d.y;
        float* p = &cells[(gy * G + gx) * 4];
        p[0] = p[1] = p[2] = r < 1.f ? r : 1.f;
        p[3] = r >= 1.f ? 0.5f : 0.f;
      }
    f.cells = cells;
    f.cols = f.rows = G;
    f.exposure_ms = 10.f;
    f.iso = 100.f;
    f.time_ns = t;
    t += 33000000;
    const roomtone_frame_result result = roomtone_room_add_frame(room, &f);
    roomtone_recorder_frame(rec, &f);
    if (k == 0)
      check(result.used, "a frame is used");
  }
  roomtone_recorder_close(rec);
  check(roomtone_room_frames(room) == 300, "300 frames");

  roomtone_room_make(room);
  roomtone_lighting l;
  check(roomtone_room_light_at_index(room, 0, &l), "made");
  check(dotv(norm(l.lights[0].dir), lamp) > 0.95f, "the key light comes from the lamp");
  check(l.lights[0].color[0] > 0.f, "the key light is lit");
  check(l.level > 0.f && l.mean > 0.f, "a level and a mean");

  roomtone_spot spots[4];
  const size_t count = roomtone_room_spots(room, spots, 4);
  check(count == 1 && dotv(spots[0].dir, lamp) > 0.95f, "one bright spot, the lamp");

  char summary[512], small[8];
  const size_t length = roomtone_room_summary(room, summary, sizeof summary);
  check(length > 0 && strlen(summary) == (length < sizeof summary ? length : sizeof summary - 1), "a summary");
  roomtone_room_summary(room, small, sizeof small);
  check(strlen(small) == sizeof small - 1 && !strncmp(small, summary, sizeof small - 1), "a summary, cut to fit");
  printf("%s\n", summary);

  /* The recording makes the same light in another room. */
  roomtone_room* again = roomtone_room_new();
  check(roomtone_replay_file(path, again), "the recording replays");
  check(!roomtone_replay_file("no-such-file.roomtone", again), "a missing recording doesn't");
  roomtone_room_make(again);
  roomtone_lighting m;
  check(roomtone_room_light_at_index(again, 0, &m), "the replay made a light");
  float worst = 0.f;
  for (int i = 0; i < 9; ++i)
    for (int c = 0; c < 3; ++c)
      worst = fmaxf(worst, fabsf(l.sh[i][c] - m.sh[i][c]));
  for (int c = 0; c < 3; ++c)
    worst = fmaxf(worst, fabsf(l.lights[0].color[c] - m.lights[0].color[c]));
  worst = fmaxf(worst, fabsf(l.level - m.level));
  printf("replay differs by %g at most\n", worst);
  check(worst < 1e-5f, "the replay's light is the room's");

  roomtone_room_free(again);
  roomtone_room_free(room);
  roomtone_room_free(NULL);
  roomtone_recorder_close(NULL);
  remove(path);
  return failures ? 1 : 0;
}
