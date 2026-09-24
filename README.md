# Astra + RoboCasa365

Project files: `robot.py`, `run_log.py`, `requirements.txt`, `setup.md`, and this README. All dependencies,
upstream source, Python runtime, and kitchen assets live inside `astra_robotics/`.

The loop is observations → Astra XYZ + gripper command → numerical IK → robosuite joint
position controller → simulated PandaOmron robot → updated observations. By default,
observations are numerical scene state. Add `--vision` to use the three RGB cameras.

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
orientation, arm joint positions, finger positions, and the commanded open/close state.
Action feedback includes robot tracking error and rejected commands. It does **not**
send object/fixture poses or sizes, fixture roles, object-contact labels, object lift,
or simulator task-success status. Pickup names the target by its language description.
The simulator still evaluates task/pickup success locally, without giving those
private measurements to Astra. Output remains absolute world XYZ + open/close.

This is RGB plus proprioception, not image-only control. There is no depth sensor or
camera-calibration input yet, so estimating metric XYZ from images can be difficult;
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

## Run on a Linux server

The macOS `astra_robotics` virtual environment contains Mac binaries and absolute
paths. Transfer the Python files and the two patched source checkouts with their
RoboCasa assets, then use a Linux Conda Python 3.11 environment named
`astra_robotics` on the server. The source and assets are about 10 GB, so check quota first.

From this project directory on the Mac (after SSH resolves/connects):

```sh
ssh sruthi@cv16.cs.columbia.edu 'df -h /proj/vondrick3/sruthi/Appaji/jgdcl; python3.11 --version; nvidia-smi'
ssh sruthi@cv16.cs.columbia.edu 'mkdir -p /proj/vondrick3/sruthi/Appaji/jgdcl/astra_robotics_project'
rsync -aP --relative --exclude='.git/' --exclude='__pycache__/' \
  robot.py run_log.py requirements.txt README.md setup.md \
  astra_robotics/src/robocasa/ astra_robotics/src/robosuite/ \
  sruthi@cv16.cs.columbia.edu:/proj/vondrick3/sruthi/Appaji/jgdcl/astra_robotics_project/
```

The `--relative` option preserves the source paths expected by `requirements.txt`.
`rsync` can be rerun after a connection interruption. It deliberately skips the
macOS environment and unrelated recordings. To move existing recordings too:

```sh
rsync -aP outputs/ sruthi@cv16.cs.columbia.edu:/proj/vondrick3/sruthi/Appaji/jgdcl/astra_robotics_project/outputs/
```

On the Linux server:

```sh
ssh sruthi@cv16.cs.columbia.edu
cd /proj/vondrick3/sruthi/Appaji/jgdcl/astra_robotics_project
conda activate astra_robotics
python --version
test -f astra_robotics/src/robosuite/setup.py
test -f astra_robotics/src/robocasa/setup.py
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
export MUJOCO_GL=egl
python robot.py --task PickPlaceSinkToCounter --seed 6 --mock --max-calls 1 --headless
```

Linux uses `python`, not macOS `mjpython`. `--headless` disables the interactive
viewer while retaining all three camera renders and the MP4 recording. EGL needs
a working GPU graphics driver; on a CPU-only server, try `MUJOCO_GL=osmesa` if
OSMesa is installed. [MuJoCo's rendering documentation](https://mujoco.readthedocs.io/en/stable/python.html#rendering)
describes the offscreen context requirements. A cluster may require a GPU job
allocation before EGL works; use the site's scheduler if `nvidia-smi` is unavailable
on the login node.

After the mock run succeeds, set `OPENAI_API_KEY` in that server shell and run the
same command without `--mock` to call Astra. Do not put the key in a file that you
transfer or commit. To view completed transcripts on the Mac, copy them back:

```sh
rsync -aP sruthi@cv16.cs.columbia.edu:/proj/vondrick3/sruthi/Appaji/jgdcl/astra_robotics_project/outputs/ ./outputs/
```

The local RoboCasa checkout includes the seeded counter placement fix; transferring
that checkout preserves it. A fresh clone from upstream would need the fix again.

## Run the installed project

The environment now uses **Python 3.11**, as recommended by RoboCasa. If an old
terminal still has the previous environment activated, reactivate it:

```sh
source astra_robotics/bin/activate
python --version
mjpython robot.py --mock
```

This opens a RoboCasa365 kitchen and moves the gripper 5 cm upward without calling
Astra. The window closes after success. For a headless check:

```sh
mjpython robot.py --mock --headless
```

To use Astra, set your key privately (zsh):

```sh
read -s 'OPENAI_API_KEY?OpenAI API key: '
export OPENAI_API_KEY
mjpython robot.py
```

Astra runs remotely and needs API access to `gpt-6-astra`. The Python client and
simulation run locally. At most five paid requests are made per reaching run by default
(twelve for pickup);
`--max-calls 2` changes that limit. A successful simple reach usually needs one.
API calls run in a background worker while simulation physics is paused.
Closing during an API request may delay process exit until its 60-second timeout.

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
not a standard RoboCasa benchmark score. Fixed wrist orientation and stationary base
limit which tasks it can solve; some tasks/layouts need additional assets or motion
capabilities. The official evaluator is used unchanged, without claiming universal
task support.

## Pickup

```sh
mjpython robot.py --pickup --object obj
```

`--pickup` defaults to object `obj`, so `mjpython robot.py --pickup` also works.
Astra chooses every XYZ waypoint and an explicit gripper action:

```json
{"x": 0.35, "y": -0.40, "z": 0.94, "gripper": "close"}
```

The gripper command takes effect at the start of the movement and persists while
waiting for the next API response. Approach and descend with `open`, close in a
separate action at the grasp position, then lift with `close`. Each accepted action
runs for 160 controller updates (8 simulated seconds) before checking completion
and requesting the next action. Physics pauses during API requests, while rendering
continues. Failed IK leaves the gripper unchanged and does not advance physics.

Pickup observations include finger joint positions, the last gripper command,
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
performance has not been tested here. Fixed wrist orientation limits possible grasps.
Change the action budget with `--max-calls 16` if you want more attempts/API calls.

The pickup goal lives in `run()` under `if args.pickup`, the model action schema is
`Position`, and `Arm.step()` maps `open` to -1 and `close` to +1 for the gripper.
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

The observation contains every episode object and fixture: name/type, world XYZ,
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
and contacts enabled. Astra controls XYZ and the gripper; wrist orientation stays
fixed and the base/torso receive stationary commands. Pickup does not complete the
benchmark's counter-to-cabinet task: placement/navigation are separate objectives.

IK runs on scratch simulator data and passes joint targets to the real controller;
it does not teleport the robot. Targets must be finite, within 35 cm of the current
tip, and IK-reachable within joint limits. Joint setpoints ramp at 0.5 rad/s.
There is no collision-aware path planner: contacts or unreachable targets can cause
failure, which is reported to Astra. Reaching success requires measured target error
below 1 cm for ten controller updates; pickup uses the grasp-and-lift check above.
These checks are for simulation, not real hardware.

## Recreate the installation

With Python 3.11 installed, from this project directory:

```sh
python3.11 -m venv astra_robotics
source astra_robotics/bin/activate
mkdir -p astra_robotics/src
git clone https://github.com/ARISE-Initiative/robosuite.git astra_robotics/src/robosuite
git -C astra_robotics/src/robosuite checkout 5ce6643f3092639d08f7b0f90ed1c6a84f50552c
git clone https://github.com/robocasa/robocasa.git astra_robotics/src/robocasa
git -C astra_robotics/src/robocasa checkout 4f8a2980def75a55dff96b990745b83540425f09
python -m pip install -r requirements.txt
export DYLD_FALLBACK_LIBRARY_PATH="/usr/lib${DYLD_FALLBACK_LIBRARY_PATH:+:$DYLD_FALLBACK_LIBRARY_PATH}"
python -m robocasa.scripts.setup_macros
python -m robosuite.scripts.setup_macros
python -m robocasa.scripts.download_kitchen_assets --type tex fixtures_lw objs_objaverse objs_lw
```

Accept the asset downloader's prompt. This is a multi-gigabyte download; no training
or demonstration datasets are needed. Upstream pins MuJoCo 3.3.1, NumPy 2.2.5,
Numba 0.61.2 and SciPy 1.15.3. Its full dependency list also includes training tools.
The previous Python 3.14 environment and toy script were preserved under
`astra_robotics_py314_backup/`; the current project does not use them.

## Verified locally

The added pickup mock passed on the default pear/scene: eight actions, about 18 cm
object lift, with two-finger contact maintained for ten updates. Structured API
tests verified both gripper commands and invalid-command rejection without paid calls.

On the default kitchen/seed, headless mock control passed the 5 cm upward reach
(0.5 mm final error) and the object-relative reach (0.9 mm final error, two waypoints).
Scene loading took about 3 seconds on this Mac. The numerical state export contained
3 objects and 44 fixtures. The graphical `mjpython robot.py --mock --object obj`
run also passed (0.9 mm final error). The structured Astra request/response was
checked with a mock HTTP response; no live API call was tested during setup.

On this Mac, MuJoCo 3.3.1's `mjpython` launcher omitted `/usr/lib` from its fallback
library path, preventing llvmlite from loading `@rpath/libz.1.dylib`. The installed
`astra_robotics/bin/mjpython` now preserves that system path. Reinstalling MuJoCo may
replace the launcher; the `DYLD_FALLBACK_LIBRARY_PATH` export in the installation
instructions provides the same fix for a fresh installation.

References: [RoboCasa installation](https://robocasa.ai/docs/build/html/introduction/installation.html),
[RoboCasa source](https://github.com/robocasa/robocasa),
[robosuite source](https://github.com/ARISE-Initiative/robosuite),
[Astra](https://developers.openai.com/api/docs/models/gpt-6-astra).
# astra-play-jgd
