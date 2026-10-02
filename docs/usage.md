# Control and usage reference

See the [README](../README.md) for installation and quick-start commands.
Examples here use macOS `mjpython`; on Linux use `python` with the rendering
backend configured as described in the README.

## Action coordinate frame

Astra returns **absolute XYZ in the episode's robot frame**, in meters: +X forward,
+Y left, +Z up. The origin is the Panda arm's mounting body (`robot0_link0`), sampled
after initial settling. The frame stays fixed even if the mobile base drifts.
The controlled point is between the fingers. Astra also outputs `yaw`, `pitch`,
and `roll` in radians as absolute offsets from the initial gripper orientation.
All zeros restore the starting orientation; copy observation `tip_yaw_pitch_roll`
to hold the current orientation. The rotation is `Rz(yaw) Ry(pitch) Rx(roll)` applied
to the initial orientation in robot coordinates: roll about robot X, then pitch
about robot Y, then yaw about robot Z, all following the right-hand rule. Positive
yaw turns counterclockwise viewed from above. Angles are not incremental commands.
XYZ stays the target grasp point while the palm/fingers rotate around it.

`gripper` is a continuous absolute opening target: **0 = fully closed, 1 = fully
open**, and 0.5 = half open (approximately 4 cm out of 8 cm full finger travel).
The local Panda adapter holds intermediate openings using the existing position
actuators. This is aperture control, not force control; objects can block closure.
Observations include both the commanded and measured normalized opening.

For a small rotation test with Astra:

```sh
mjpython robot.py --vision --max-calls 1 \
  --instruction "Keep the current XYZ and gripper open. Set yaw to 0.15 radians, pitch to 0, and roll to 0."
```

Custom instructions still use the existing reaching/task success check. This test
can exhaust its action budget even when the rotation tracks successfully.

Gripper, object, and fixture positions and orientations in model observations use
this frame, including free-joint poses. Articulated joint coordinates are unchanged;
object sizes remain in object-local axes. Targets are converted back to world coordinates before the existing IK.
Logs record the frame transform and both robot/world targets and measured positions.

Every Astra request includes `recent_actions`: the five most recent completed or
rejected actions, oldest first (empty on the first request). Each compact entry has
the target and gripper command, requested and measured XYZ displacement, tracking
error, target/measured yaw-pitch-roll, orientation error, requested-direction progress
fraction, and status. An action is reached only if orientation error is below
0.05 rad (about 2.9 degrees), endpoint error is below 1 cm, and it makes at least 80% progress in the requested
direction. Commands shorter than 1 mm waive the progress requirement; position and orientation tolerances still apply. Only the latest two
entries retain Astra's note. All coordinates
use the episode robot frame. The detailed start/end positions and outcome prose are
still saved in local traces, but are not resent to Astra. `last_action_result` is
also omitted because it duplicated the newest history entry. This history uses robot
state, not simulator contact labels or object positions, and is included in both
vision and numerical mode. Earlier camera images are not resent.

All three camera images include projected robot-axis rulers: red +X, green +Y,
blue +Z. The mount ruler is 15 cm; a parallel copy at the tip is 10 cm. The latter
does not redefine the origin. These virtual overlays are drawn over objects and
can fall outside a camera view. They appear in vision input, recordings, and the
browser display, not the native MuJoCo viewer.

Verify the coordinate conversion and controller without any API calls:

```sh
mjpython robot.py --headless --test-axes
```

This commands +/-3 cm on each robot axis, with 8 simulated seconds per motion,
returning to the initial tip position between tests. It records expected/measured
world displacement and fails if tracking error exceeds 5 mm. A failure can also
indicate contact or an unreachable pose in the chosen scene.

Then test a visual approach with Astra:

```sh
mjpython robot.py --vision --env CheesyBread --seed 0 --max-calls 12 \
  --instruction "Move toward the microwave handle using the robot-axis overlays. Keep the gripper open."
```

Explicit XYZ instructions now refer to the robot frame, not world coordinates.
The frame and overlays clarify control directions; visual localization can still
be inaccurate. The custom instruction does not replace the existing success check.

## Camera input for Astra

Use `--instruction "Your goal here"` to override the goal sent to Astra. For example:

```sh
mjpython robot.py --vision --max-calls 1 --instruction "Move the gripper a small distance to the left in the left external camera image, keeping approximately the same height."
```

This overrides the prompt only; the existing reaching/pickup/task success checks
still apply. A custom movement may therefore finish with action budget exhaustion.
For simple movement experiments, omit `--task` to avoid also sending an official
RoboCasa task instruction. On Linux, use `python` with `--headless`.

```sh
mjpython robot.py --task PickPlaceCounterToCabinet --vision
mjpython robot.py --pickup --object obj --vision
```

Every model request includes three fresh, separately labeled RGB JPEG images: left,
right, and wrist. Images are captured together on the simulation thread before the
background API request. They use the same visual-only geometry as the browser display.

Vision mode sends the task instruction plus robot proprioception: gripper XYZ and
orientation, arm joint positions, finger positions, and commanded/measured gripper opening.
Action feedback includes robot tracking error and rejected commands. It does **not**
send object/fixture poses or sizes, fixture roles, object-contact labels, object lift,
or simulator task-success status. Pickup names the target by its language description.
The simulator still evaluates task/pickup success locally, without giving those
private measurements to Astra. Output is absolute robot-frame XYZ + yaw/pitch/roll + gripper opening.

This is RGB plus proprioception, not image-only control. There is no depth sensor or
camera-calibration matrix input; the axis overlays supply local visual rulers, but estimating metric XYZ can still be difficult;
visual task success is not guaranteed. Images add API token usage and may add latency.

`--no-camera-views` hides the browser display but still sends images with `--vision`.
`--headless` hides both windows but still renders the model's camera frames. Inspect
the allowed text fields and image dimensions without calling the API:

```sh
mjpython robot.py --task PickPlaceCounterToCabinet --vision --inspect --headless
```

`--vision --mock` captures the images but holds the robot still for a wiring check;
it does not use privileged coordinates to simulate a visual policy. A one-call task
smoke test is expected to report budget exhaustion with success=False:

```sh
mjpython robot.py --task PickPlaceCounterToCabinet --vision --mock --headless --max-calls 1
```

Camera capture and a mocked multimodal API request/response have been verified.
Live Astra vision task completion has not been tested.

## Three live camera views

Every graphical run now also opens a local browser page showing the left external,
right external, and wrist cameras side by side, with labels. The page address is
printed in the terminal if the browser does not open automatically.

```sh
mjpython robot.py --pickup --object obj
```

The cameras render at 320×240 each using MuJoCo's native renderer. The browser
updates at approximately 10 fps; recording captures every 20 Hz action step.
The existing interactive viewer remains available.
Without `--vision`, Astra receives numerical state only. The browser retains the last image when
the run ends. Use `--no-camera-views` for only the original viewer, or `--headless`
to disable both displays. Recording remains enabled in either case.

## Saved runs

Every simulation run automatically creates `outputs/<task>_<timestamp>/` containing:

- `transcript.html`: open in a browser for the final outcome, system prompt,
  video, and one box per model step with its input and full returned action.
  Click a video timestamp to seek.
- `video.mp4`: labeled left/right/wrist views, 960×240 at 20 fps. It records the initial
  scene and every action step; frozen API waits are omitted. One action is 8 simulated seconds.
- `traces.jsonl`: chronological structured events, including system prompts, full
  numerical observations or vision inputs, response metadata/usage, and parsed actions.
- `run.json`: run settings, start time, completion status, and frame count.
- JPEG snapshots: recorded views and the exact image bytes sent with vision requests.

The HTML uses local files and requires no server. Full API requests are represented
as SDK arguments, with the structured-output schema and local JPEG paths replacing
base64 image URLs. Only model output returned by the API is recorded; no hidden
reasoning is available. Mock calls are labeled separately. Traces contain no API key.
Video and traces are finalized on success, errors, or Ctrl-C; force-killing the process
may leave an incomplete MP4. A failed load can leave traces without video.
After updating the viewer code, rebuild older transcripts from their saved traces with
`python run_log.py --rebuild` inside the project environment.

Recording uses `imageio` and `imageio-ffmpeg`, already installed in `astra_robotics`.
On macOS, use `mjpython` even with `--headless`, since recordings still render cameras.
`--list-tasks` only lists tasks and does not create a recording.

## Run a RoboCasa-defined task

```sh
mjpython robot.py --task PickPlaceCounterToCabinet
```

This uses the selected environment's `get_ep_meta()["lang"]` as Astra's goal and
`env._check_success()` as the sole success condition, checked each controller tick.
Reaching a waypoint or lifting an object does not end the task. For counter-to-cabinet,
RoboCasa checks that the object is inside the cabinet and the gripper has moved away.
In numerical mode, each API request includes the official instruction, success status, task fixture-role
names (e.g. `cab` and `counter`), grasped-object names, and the numerical scene state.
The cameras remain display-only unless `--vision` is enabled; see its observation rules above.

Select another installed task by name, or list the registry:

```sh
python robot.py --list-tasks
mjpython robot.py --task PickPlaceCounterToSink --max-calls 40
mjpython robot.py --task PickPlaceCounterToCabinet --inspect --headless
```

Task mode defaults to **30 paid API requests maximum**. Exhausting the budget reports
`success=False` and exits with an error. `--task` selects both the environment and
objective; do not combine it with custom `--pickup` or `--object` objectives.
`--env` alone still only selects a scene for the custom demo modes.

To check task creation and evaluator wiring without API calls:

```sh
mjpython robot.py --task PickPlaceCounterToCabinet --mock --headless --max-calls 1
```

This mock holds the robot still and normally exits with **task failure**. It is not
a scripted task-solving policy. Task instruction/state export and this failure path
have been verified; autonomous Astra task completion has not been tested.

This remains a modified-controller experiment (with privileged observations in numerical mode),
not a standard RoboCasa benchmark score. The stationary base and lack of collision planning
limit which tasks it can solve; some tasks/layouts need additional assets or motion
capabilities. The official evaluator is used unchanged, without claiming universal
task support.

## Pickup

```sh
mjpython robot.py --pickup --object obj
```

`--pickup` defaults to object `obj`, so `mjpython robot.py --pickup` also works.
Astra chooses every XYZ waypoint, wrist orientation, and explicit gripper action:

```json
{"x": 0.35, "y": -0.40, "z": 0.94, "yaw": 0.15, "pitch": 0.0, "roll": 0.0, "gripper": 0.0}
```

The gripper command takes effect at the start of the movement and persists while
waiting for the next API response. Approach and descend with `gripper: 1.0`, close in a
separate action at the grasp position, then retain that opening target while lifting. Each accepted action
runs for 160 controller updates (8 simulated seconds) before checking completion
and requesting the next action. Physics pauses during API requests, while rendering
continues. Failed IK leaves the gripper unchanged and does not advance physics.

In numerical mode, pickup observations include finger joint positions, the last gripper command,
two-finger grasp-contact status, initial object position, and actual object lift.
Success requires both finger pads to contact the selected object while its root
origin is at least 8 cm above its initial height for ten consecutive updates.
Simply moving the gripper upward does not count. The window closes on success.

Test without API calls:

```sh
mjpython robot.py --pickup --mock
mjpython robot.py --pickup --mock --headless
```

The mock sequence is tuned to the default pear/scene, not a general grasp planner.
Astra uses the observations and feedback to choose its own actions; live pickup
performance has not been tested here. IK does not check collisions or plan collision-free paths.
Change the action budget with `--max-calls 16` if you want more attempts/API calls.

The pickup goal lives in `run()` under `if args.pickup`, the model action schema is
`Position`. `Arm.step()` maps opening [0, 1] to [-1, 1] for our absolute-opening
Panda adapter, which generates opposing finger actuator position targets.
The pickup success condition is the `lift_stable` check in `run()`.

## Custom reaching

```sh
mjpython robot.py --object obj
```

Your edited reaching branch currently asks Astra to "Move the tip to the cabinet"
and checks a target 70 cm above object `obj`. Those two definitions may disagree;
keep `instruction` and `target` aligned when editing your reaching goals.
The pickup option has its own goal and leaves that custom reaching branch unchanged.
The model can choose intermediate XYZ waypoints, receiving measured feedback after
movement. Object root origins are not guaranteed to be geometric centers or surfaces.
An offset from the origin is a demo objective, not a general grasp affordance.

Inspect the exact numerical scene observation, without an API call:

```sh
mjpython robot.py --inspect --headless
```

The observation contains every episode object and fixture: name/type, robot-frame XYZ,
quaternion (wxyz), articulated joint positions, available sizes, and object language
labels, plus robot arm joint positions and gripper XYZ. Pickup requests additionally
include gripper and lift feedback as described above. This is privileged simulator
state, including occluded items. It is not a full mesh or collision map.

Use `--seed`, `--layout`, `--style`, or `--env` to change the scene. Defaults are
`PickPlaceCounterToCabinet`, layout 1, style 1, seed 0. Other tasks may require extra
object assets. The installed subset uses Objaverse objects, ordinary textures, and
Lightwheel fixtures and accessory objects; AI-generated textures and objects are omitted.

To avoid accidentally using Conda base's `mjpython`, invoke this project's launcher
explicitly when comparing seeds:

```sh
./astra_robotics/bin/mjpython robot.py --task PickPlaceSinkToCounter --seed 6 --inspect
./astra_robotics/bin/mjpython robot.py --task PickPlaceSinkToCounter --seed 6 --vision --inspect
```

`--inspect` prints diagnostics and exits; it does not update an existing browser viewer.

The local RoboCasa checkout includes a reproducibility fix in
`robocasa/models/fixtures/counter.py`: counter surface deduplication uses
`dict.fromkeys` instead of `set` to preserve sampling order. XML elements hash by
identity, so the old ordering varied across processes, triggering different placement
retries and object resampling despite the same seed. Preserve this patch if replacing
or reinstalling the RoboCasa checkout.

## Scope

This supports reaching and pickup inside a real RoboCasa kitchen, with gravity
and contacts enabled. Astra controls XYZ, wrist yaw/pitch/roll, and the gripper;
the base/torso receive stationary commands. Pickup does not complete the
benchmark's counter-to-cabinet task: placement/navigation are separate objectives.

IK runs on scratch simulator data and passes joint targets to the real controller;
it does not teleport the robot. Targets must be finite and IK-reachable within joint limits. Joint setpoints ramp at 0.5 rad/s.
There is no collision-aware path planner: contacts or unreachable targets can cause
failure, which is reported to Astra. Reaching success requires measured target error
below 1 cm for ten controller updates; pickup uses the grasp-and-lift check above.
These checks are for simulation, not real hardware.
