# Roomtone

Room light estimation for mixed reality: light virtual content with the user's real room.

ARKit and ARCore estimate the room's light, and so does Android XR (`XR_ANDROID_light_estimation`). Meta Quest offers no such API and no ambient light sensor to apps. Roomtone makes the estimate from what Quest does give: the Passthrough Camera API's frames with their exposure, the headset's pose, and depth (raycasts, the Depth API).

It produces what a renderer wants:

- **Spherical harmonics (L2)** for the soft light from every direction, scaled so an up-facing surface gets 1 (Unity's ambient probe takes this directly).
- **Up to two key lights**, the brightest parts of the room at least 45 degrees apart, with a place in the room when depth saw them, aimed (and falling off) from the point you ask about.
- **A level** that matches the brightness passthrough shows the room at, so virtual objects come out as bright as real ones beside them. The camera shares passthrough's auto exposure; the level uses the exposure of the frames that looked at the point asked about, and counts a frame a bright light dominated (a monitor, a window) as a little brighter.

It also copes with rooms seen only in part (unseen directions take the seen surfaces, lights left out), with lights switched on and off (a room that reads 3x brighter or darker for a second starts over), and with recentering.

The library is plain C++17 with no dependencies and no GPU work: a frame reduced to a small grid (32x32 cells of mean linear colour) costs well under a millisecond, and it runs only while you feed it frames.

## Use

```cpp
#include <roomtone/roomtone.hpp>

roomtone::Room room;
room.setCamera(camera);          // Camera2's lens pose and intrinsics
room.addFrame(frame);            // each reduced camera frame, its exposure and the head's pose
room.addHit(origin, point);      // depth hits along rays from the head (optional)

roomtone::Lighting light;
if (room.lightAt(contentPosition, light)) {
  // light.sh, light.lights[0..1], light.mean, light.level
}
```

Coordinates are tracking space in metres, +Y up (OpenXR's). A `Room` isn't thread-safe; one mutex around it is enough.

## Record and replay

`roomtone::Recorder` writes a session's inputs to a file; `roomtone_replay FILE` plays it back on a desktop and prints what the room made, so the estimate can be tuned and tested without a headset.

## Status

Extracted from [Xelee](https://github.com/rhythmerc/melee-pc) (a Melee port's Quest 3 mixed reality mode), where the stage sits on a table and is lit by the room. The constants were tuned on a Quest 3 in one user's rooms. Depth images place the room's surfaces as 10 cm voxels, and the light is made from the points you ask about (several across the content, to blend between): near walls, the table under it and a lamp beside it light it from where they are. Placed lights carry a range, so a shader can light them as point lights.

## License

MIT.
