# Server setup handoff for an AI agent

The user wants this **existing** Astra + RoboCasa365 project running on
`sruthi@cv16.cs.columbia.edu`. Work in
`/proj/vondrick3/sruthi/Appaji/jgdcl/astra_robotics_project` on that server.
Use the existing `robot.py`, `run_log.py`, `requirements.txt`, and patched upstream
checkouts. Preserve the minimal project and user edits. Complete setup and a mock
smoke test; do not make a paid Astra call just to verify installation.

## What to transfer

The Mac project is `/Users/sruthisudhakar/Columbia/test`. Its Python environment
`astra_robotics/` is **macOS-specific** and must not be copied as an environment.
Transfer these files and source directories, preserving relative paths:

```text
robot.py
run_log.py
requirements.txt
README.md
setup.md
astra_robotics/src/robocasa/
astra_robotics/src/robosuite/
```

The source directories include about **10 GB of RoboCasa assets**. They also include
one essential local patch in
`astra_robotics/src/robocasa/robocasa/models/fixtures/counter.py`:

```python
valid_geoms = list(dict.fromkeys(valid_geoms))
```

It replaces `list(set(valid_geoms))` so the same seed chooses the same objects
across fresh processes. The source revisions on the Mac are RoboCasa
`4f8a2980def75a55dff96b990745b83540425f09` and robosuite
`5ce6643f3092639d08f7b0f90ed1c6a84f50552c`. Do not overwrite the patch with
an unmodified upstream clone.

If the transfer has not happened, the Mac-side command below should be run from
`/Users/sruthisudhakar/Columbia/test` while SSH access to Columbia works:

```sh
ssh sruthi@cv16.cs.columbia.edu 'mkdir -p /proj/vondrick3/sruthi/Appaji/jgdcl/astra_robotics_project'
rsync -aP --relative --exclude='.git/' --exclude='__pycache__/' \
  robot.py run_log.py requirements.txt README.md setup.md \
  astra_robotics/src/robocasa/ astra_robotics/src/robosuite/ \
  sruthi@cv16.cs.columbia.edu:/proj/vondrick3/sruthi/Appaji/jgdcl/astra_robotics_project/
```

`rsync` can be resumed. Do not transfer `.venv`, the macOS `astra_robotics/bin`
or `astra_robotics/lib`, the Python 3.14 backup, or unrelated `.mov` files. The
existing `outputs/` folder is optional:

```sh
rsync -aP outputs/ sruthi@cv16.cs.columbia.edu:/proj/vondrick3/sruthi/Appaji/jgdcl/astra_robotics_project/outputs/
```

If `cv16.cs.columbia.edu` fails to resolve, check the Columbia VPN/network. Do not report migration
as completed until files are on the server and the smoke test passes.

## On the server

First inspect the host and project; do not assume the login node has a GPU:

```sh
uname -s
uname -m
python3.11 --version
df -h "$HOME"
nvidia-smi
cd /proj/vondrick3/sruthi/Appaji/jgdcl/astra_robotics_project
ls robot.py run_log.py requirements.txt setup.md
ls astra_robotics/src/robocasa/robocasa/models/assets/objects/objaverse/egg/egg_11/model.xml
rg -n 'valid_geoms = list\(dict.fromkeys\(valid_geoms\)\)' astra_robotics/src/robocasa/robocasa/models/fixtures/counter.py
```

Expected target is Linux with Python 3.11 and sufficient disk quota (allow well
over 10 GB for assets and installed packages). Use the server's Conda environment
named `astra_robotics` with Python 3.11. If a GPU is provided only inside a scheduled
job, obtain the site's normal GPU allocation before the rendering smoke test.
Do not invent scheduler commands without checking the site's configuration.

After `rsync` has fully finished, activate the environment **on the server**:

```sh
cd /proj/vondrick3/sruthi/Appaji/jgdcl/astra_robotics_project
conda activate astra_robotics
python --version
test -f astra_robotics/src/robosuite/setup.py
test -f astra_robotics/src/robocasa/setup.py
test -f astra_robotics/src/robocasa/robocasa/models/assets/objects/objaverse/egg/egg_11/model.xml
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

`requirements.txt` installs the two transferred source trees in editable mode,
plus MuJoCo 3.3.1, NumPy 2.2.5, OpenAI, and video dependencies. Do not substitute
current upstream checkouts or copy installed macOS wheels. If Linux system
libraries are missing, resolve those for the server's distribution or ask its
administrator; do not change project code to hide an import/rendering failure.

Linux uses `python`, **not** macOS `mjpython`. Since even `--headless` records
camera video, choose an offscreen MuJoCo backend before Python imports MuJoCo:

```sh
export MUJOCO_GL=egl
python robot.py --task PickPlaceSinkToCounter --seed 6 --vision --inspect --headless
python robot.py --task PickPlaceSinkToCounter --seed 6 --mock --max-calls 1 --headless
```

`egl` is for headless GPU rendering and requires an EGL-capable graphics driver.
If the machine has no GPU but OSMesa is installed, try `MUJOCO_GL=osmesa` instead.
If the GPU is only visible inside a job, run these commands inside that job. A
task-mode mock is **expected to exit with success=False / action budget exhausted**;
that verifies the loop without calling Astra. The first inspect command should
exit successfully after printing a camera-shape observation.

Verify the newest `outputs/PickPlaceSinkToCounter_<timestamp>/` has
`transcript.html`, `video.mp4`, `traces.jsonl`, `run.json`, and JPEG snapshots.
The mock video should be 960×240, 20 fps, with 161 frames for one 160-step action
plus its initial frame. Check that the transcript contains the mock input/output
and final outcome. Compare two fresh `--inspect --headless --seed 6` runs to
confirm the same object descriptions; tiny settled physics positions may differ.

After setup, the user's real command is for example:

```sh
export MUJOCO_GL=egl
python robot.py --task PickPlaceSinkToCounter --seed 6 --max-calls 40 --vision --headless
```

The user must supply `OPENAI_API_KEY` privately in that server shell. Never print
or save it, and never copy it from the Mac. Astra API calls are remote; simulation
and MP4 recording run on the server. To bring results back to the Mac, run there:

```sh
rsync -aP sruthi@cv16.cs.columbia.edu:/proj/vondrick3/sruthi/Appaji/jgdcl/astra_robotics_project/outputs/ ./outputs/
```

Report the environment path, rendering backend, smoke-test result, and output
folder. If setup is blocked, identify the exact failed command and missing
prerequisite. See `README.md` for project behavior and CLI options.
