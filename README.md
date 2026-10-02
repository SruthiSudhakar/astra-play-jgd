# Astra + RoboCasa365

Run Astra-controlled reaching, pickup, and kitchen tasks in RoboCasa365 with a
simulated PandaOmron robot. Astra commands position, wrist orientation, and gripper
opening; numerical IK converts them to joint motion. Input is numerical scene
state by default, or three RGB cameras plus robot proprioception with `--vision`.

## Install

Requires **Python 3.11**, Git, macOS or Linux, and well over **10 GB free disk space**
for dependencies and kitchen assets. Live runs also require an `OPENAI_API_KEY`
with access to `gpt-6-astra`; offline tests do not.

```sh
git clone https://github.com/SruthiSudhakar/astra-play-jgd.git
cd astra-play-jgd
python3.11 -m venv astra_robotics
source astra_robotics/bin/activate
mkdir -p astra_robotics/src

# Use the tested upstream revisions.
git clone https://github.com/ARISE-Initiative/robosuite.git astra_robotics/src/robosuite
git -C astra_robotics/src/robosuite checkout 5ce6643f3092639d08f7b0f90ed1c6a84f50552c
git clone https://github.com/robocasa/robocasa.git astra_robotics/src/robocasa
git -C astra_robotics/src/robocasa checkout 4f8a2980def75a55dff96b990745b83540425f09

# Preserve deterministic seeded counter placement.
git -C astra_robotics/src/robocasa apply ../../../patches/robocasa-deterministic-counter.patch
python -m pip install -r requirements.txt
```

Configure rendering for your platform **before running Python/MuJoCo**:

```sh
# macOS: needed for MuJoCo/llvmlite to find system libraries.
export DYLD_FALLBACK_LIBRARY_PATH="/usr/lib${DYLD_FALLBACK_LIBRARY_PATH:+:$DYLD_FALLBACK_LIBRARY_PATH}"

# Linux with an EGL-capable GPU and graphics driver:
export MUJOCO_GL=egl
```

Run only the export for your platform. CPU-only Linux can use
`MUJOCO_GL=osmesa` if OSMesa is installed. On a cluster, run inside a GPU allocation
when using EGL. Even `--headless` needs rendering because runs record video.

Finish installation and accept the asset downloader's prompt:

```sh
python -m robocasa.scripts.setup_macros
python -m robosuite.scripts.setup_macros
python -m robocasa.scripts.download_kitchen_assets --type tex fixtures_lw objs_objaverse objs_lw
```

No training datasets are needed. Some tasks may require additional assets.
Upstream sources, assets, environments, and recordings are excluded from Git.
`requirements.txt` needs the two source checkouts above; it cannot install alone.

## Verify and run

In each new terminal, run `source astra_robotics/bin/activate` and the rendering
export above. On **macOS use `mjpython`**, including for headless runs. On **Linux
replace `mjpython` with `python`** in the examples below.

Start with an offline test (no API calls):

```sh
mjpython robot.py --mock --headless
```

It should move the gripper 5 cm upward, print `SUCCESS`, and save a recording.
This smoke test passed locally; a fresh installation on another machine has not
been verified.

For live runs, set `OPENAI_API_KEY` privately in your shell, then run:

```sh
# Official kitchen task using camera input; at most 20 API requests.
mjpython robot.py --task PickPlaceSinkToCounter --seed 6 --vision --max-calls 20

# Pickup demo using numerical scene state.
mjpython robot.py --pickup --object obj

# Available tasks and CLI options (no API calls).
python robot.py --list-tasks
python robot.py --help
```

Add `--headless` on servers to disable windows. Omit `--vision` to send numerical
object/fixture state instead of camera images. `--inspect --headless` prints the
observation without calling the API. `--instruction "Your goal"` overrides the
prompt but keeps the existing success check. Use `--seed`, `--layout`, and `--style`
to select a scene.

Default API budgets are 5 calls for reaching, 12 for pickup, and 30 for official
tasks; override with `--max-calls`. Task-mode `--mock` holds still and normally
exits with budget exhaustion, so use the simple test above to check installation.

## Results and limitations

Open `outputs/<task>_<timestamp>/transcript.html` for the prompt, actions, outcome,
and video. The same folder contains `video.mp4`, `traces.jsonl`, `run.json`, and
camera snapshots. Keep the folder together when sharing results.

This is a simulation experiment: the base stays stationary, IK does not plan
collision-free paths, and numerical mode uses privileged simulator state.
Successful mock reaching does not establish autonomous kitchen-task performance.

## More detail

- [Control and usage reference](docs/usage.md): coordinates, orientation/gripper
  conventions, camera inputs, task evaluation, pickup, and recording details.
