"""RoboCasa365: numerical scene state -> Astra XYZ -> IK -> joint controller."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import base64
import io
import os
import random
import time
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Literal

import mujoco
import numpy as np
from openai import OpenAI
from pydantic import BaseModel
from run_log import RunLog

# Offline pickup phases: above, descend, close, lift. Tuned for the default pear.
MOCK_PICKUP_Z = [0.15, -0.005, -0.005, 0.15]
CAMERAS = ["robot0_agentview_left", "robot0_agentview_right", "robot0_eye_in_hand"]


class CameraViews:
    """Local browser display avoids competing macOS GUI event loops."""
    def __init__(self, env, display=True):
        self.env, self.frame, self.last_update = env, b"", 0.0
        self.renderer = mujoco.Renderer(env.sim.model._model, height=240, width=320)
        # Match RoboCasa's viewer: hide collision meshes, show textured visuals.
        self.scene_option = mujoco.MjvOption()
        self.scene_option.geomgroup[0] = 0
        self.scene_option.geomgroup[1] = 1
        self.server = None
        if not display:
            return
        display = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path.startswith("/frame.jpg"):
                    content, kind = display.frame, "image/jpeg"
                    if not content:
                        self.send_error(503)
                        return
                elif self.path == "/":
                    kind = "text/html; charset=utf-8"
                    content = b'''<!doctype html><html><head><title>RoboCasa cameras</title>
                    <style>body{background:#171b22;color:white;font:18px sans-serif;margin:24px}
                    img{width:100%;max-width:1440px}p{color:#bbc5d2}</style></head>
                    <body><h2>RoboCasa live cameras</h2><p>Left view / Right view / Wrist view</p>
                    <img id="views"><p id="status">Connecting...</p><script>
                    const img=document.getElementById('views'),status=document.getElementById('status');
                    let previous;
                    async function refresh(){try{
                    const response=await fetch('/frame.jpg?t='+Date.now());if(!response.ok)throw Error();
                    const next=URL.createObjectURL(await response.blob());img.src=next;
                    if(previous)URL.revokeObjectURL(previous);previous=next;
                    status.textContent='Live cameras - Astra input mode is shown in the terminal';
                    }catch(e){status.textContent='Simulation stopped or unavailable - last frame shown'}
                    setTimeout(refresh,100)}
                    refresh();</script></body></html>'''
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", kind)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                try:
                    self.wfile.write(content)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = f"http://127.0.0.1:{self.server.server_port}"
        self.update()
        print(f"Three live camera views: {self.url}", flush=True)
        webbrowser.open(self.url)

    def capture(self):
        """Fresh, synchronized views; render only on the simulation thread."""
        images = []
        for name in CAMERAS:
            self.renderer.update_scene(self.env.sim.data._data, camera=name,
                                       scene_option=self.scene_option)
            images.append(self.renderer.render().copy())
        return images

    def update(self, images=None):
        if self.server is None:
            return
        if time.monotonic() - self.last_update < 0.1:
            return
        from PIL import Image, ImageDraw
        if images is None:
            images = self.capture()
        canvas = Image.fromarray(np.concatenate(images, axis=1))
        draw = ImageDraw.Draw(canvas)
        for i, title in enumerate(["Left view", "Right view", "Wrist view"]):
            draw.rectangle((i * 320, 0, i * 320 + 110, 24), fill="black")
            draw.text((i * 320 + 8, 6), title, fill="white")
        buffer = io.BytesIO()
        canvas.save(buffer, format="JPEG", quality=85)
        self.frame = buffer.getvalue()
        self.last_update = time.monotonic()

    def close(self):
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
            self.thread.join()
        self.renderer.close()


class Position(BaseModel):
    x: float
    y: float
    z: float
    gripper: Literal["open", "close"]
    note: str = ""


def make_env(args):
    # Also seed legacy helpers that use global RNGs (e.g. camera perturbations).
    random.seed(args.seed)
    np.random.seed(args.seed)
    import robocasa  # Registers RoboCasa tasks with robosuite.
    import robosuite
    from robosuite.controllers import load_composite_controller_config

    config = load_composite_controller_config(robot="PandaOmron")
    config["body_parts"]["right"] = {
        "type": "JOINT_POSITION", "input_type": "delta",
        "input_max": 1, "input_min": -1, "output_max": 1, "output_min": -1,
        "interpolation": None,
        "kp": 150, "damping_ratio": 1, "gripper": {"type": "GRIP"},
    }
    return robosuite.make(
        args.env, robots="PandaOmron", controller_configs=config,
        has_renderer=not args.headless,
        has_offscreen_renderer=False,
        use_camera_obs=False, use_object_obs=True,
        control_freq=20, ignore_done=True, seed=args.seed,
        layout_ids=[args.layout], style_ids=[args.style],
        generative_textures=None, obj_registries=("objaverse",),
        robot_spawn_deviation_pos_x=0, robot_spawn_deviation_pos_y=0,
    )


def scene_state(env, site):
    """All episode objects and fixtures, including articulated joint positions."""
    def state(item):
        body = env.sim.model.body_name2id(item.root_body)
        result = {
            "type": type(item).__name__,
            "xyz": env.sim.data.body_xpos[body].round(5).tolist(),
            "quaternion_wxyz": env.sim.data.body_xquat[body].round(5).tolist(),
            "joints": {
                name: np.asarray(env.sim.data.get_joint_qpos(name)).round(5).tolist()
                for name in item.joints
            },
        }
        if getattr(item, "size", None) is not None:
            result["size"] = np.asarray(item.size).round(5).tolist()
        return result

    objects = {name: state(item) for name, item in env.objects.items()}
    for name in objects:
        objects[name]["description"] = env.get_obj_lang(name)
    return {
        "coordinate_frame": "MuJoCo world; meters; +z up; quaternion order wxyz",
        "tip_xyz": env.sim.data.site_xpos[site].round(5).tolist(),
        "robot_qpos": env.robots[0]._joint_positions.round(5).tolist(),
        "objects": objects,
        "fixtures": {name: state(item) for name, item in env.fixtures.items()},
    }


def task_state(env, instruction):
    """Use RoboCasa's own task predicate, never the custom reach/lift checks."""
    return {
        "name": type(env).__name__, "instruction": instruction,
        "success": bool(env._check_success()),
        "fixture_roles": {
            role: (value[0] if isinstance(value, tuple) else value).name
            for role, value in env.fixture_refs.items()
        },
        "grasped_objects": [name for name, obj in env.objects.items()
                            if env._check_grasp(env.robots[0].gripper["right"], obj)],
    }


def vision_state(env, site):
    # Explicit allowlist: no object/fixture poses, dimensions, contacts or task metrics.
    return {
        "input_mode": "vision",
        "coordinate_frame": "MuJoCo world; meters; +z up",
        "tip_xyz": env.sim.data.site_xpos[site].round(5).tolist(),
        "robot_qpos": env.robots[0]._joint_positions.round(5).tolist(),
        "tip_rotation_matrix": env.sim.data.site_xmat[site].reshape(3, 3).round(5).tolist(),
    }


def choose_position(observation, mock, target, images=None, log=None, call=None):
    if mock:
        tip = np.array(observation["tip_xyz"])
        if "robocasa_task" in observation or observation.get("input_mode") == "vision":
            # Connectivity/evaluator smoke test only; no scripted benchmark solver.
            return Position(x=tip[0], y=tip[1], z=tip[2], gripper="open")
        gripper = "open"
        if "pickup" in observation:
            pickup = observation["pickup"]
            phase = min(pickup["mock_phase"], 3)
            origin = np.array(pickup["initial_object_xyz"])
            target = origin + [0, 0, MOCK_PICKUP_Z[phase]]
            gripper = "close" if phase >= 2 else "open"
        delta = target - tip
        xyz = tip + delta * min(1, 0.15 / max(np.linalg.norm(delta), 1e-9))
        return Position(x=xyz[0], y=xyz[1], z=xyz[2], gripper=gripper)
    content = [{"type": "input_text", "text": json.dumps(observation)}]
    if images is not None:
        from PIL import Image
        for name, pixels in zip(CAMERAS, images, strict=True):
            buffer = io.BytesIO()
            Image.fromarray(pixels).save(buffer, format="JPEG", quality=90)
            content.extend([
                {"type": "input_text", "text": f"Camera: {name}"},
                {"type": "input_image", "detail": "high", "image_url":
                 "data:image/jpeg;base64," + base64.b64encode(buffer.getvalue()).decode("ascii")},
            ])
    with OpenAI(timeout=60, max_retries=0) as client:
        request = dict(
            model="gpt-6-astra", reasoning={"effort": "low"},
            input=[
                {"role": "system", "content": (
                    "You are controlling a PandaOmron robot arm in a RoboCasa kitchen. "
                    "Each observation message gives you the current proprioceptive state and camera images. "
                    "Work toward the user's goal in small, deliberate motions; re-check the observation after every motion. "
                    "Respond only with a JSON object matching the provided schema: an absolute world-frame XYZ waypoint in meters (x, y, z), "
                    "a gripper command (open or close), and a `note`. Do not write any text outside the JSON object. "
                    "In the `note`, in one or two sentences, say what you observe in the current observation and why you chose this motion. "
                    "The user is watching these notes to see what you see and what you decide, so write them for a human reader. "
                    # "Use the supplied observations. In vision mode infer object/fixture locations "
                    # "from the three labeled RGB views; numerical values describe only your robot. "
                    # "Metric depth is uncertain: choose small exploratory waypoints and reobserve. "
                    "Note that +z is up. "
                    "Orientation, torso and base are held by the local controller. "
                    "Follow the user provided instruction to complete each task. "
                    "When provided, use fixture_roles to identify the task's named fixtures. "
                    "If the task requires placing, after placing, open the gripper and retreat so the environment can check completion. "
                    # "Choose small free-space movements, avoid fixtures, and use action feedback. "
                    # "Maximum waypoint distance from current tip is 0.35 meters. "
                    "Object xyz is its root-body origin, not necessarily its top surface."
                )},
                {"role": "user", "content": content},
            ], text_format=Position,
        )
        if log:
            log.request(request, call)
        request_started = time.monotonic()
        response = client.responses.parse(**request)
        if log:
            log.event("Astra output", {"action": response.output_parsed.model_dump() if response.output_parsed else None,
                                       "latency_seconds": round(time.monotonic() - request_started, 3),
                                       "response": response.model_dump(mode="json", warnings=False)}, call)
        if response.output_parsed is None:
            raise RuntimeError("Astra returned no structured position")
        return response.output_parsed


class Arm:
    """Damped least-squares pose IK, then robosuite's joint-position controller."""
    def __init__(self, env):
        self.env = env
        self.robot = env.robots[0]
        self.controller = self.robot.part_controllers["right"]
        self.model = env.sim.model._model
        self.data = env.sim.data._data
        self.site = self.robot.eef_site_id["right"]
        self.qidx = self.controller.qpos_index
        self.vidx = self.controller.qvel_index
        self.limits = self.model.jnt_range[self.controller.joint_index]
        self.rotation = self.data.site_xmat[self.site].reshape(3, 3).copy()
        self.scratch = mujoco.MjData(self.model)
        self.jp = np.zeros((3, self.model.nv))
        self.jr = np.zeros_like(self.jp)
        self.hold = self.data.qpos[self.qidx].copy()
        self.gripper = "open"

    def solve(self, xyz):
        # if not np.isfinite(xyz).all() or np.linalg.norm(xyz - self.xyz) > 0.35:
        #     raise ValueError("Waypoint must be finite and within 0.35 m of the current tip")
        # Solve on scratch data: never teleport the actual simulated robot.
        self.scratch.qpos[:] = self.data.qpos
        for _ in range(150):
            mujoco.mj_kinematics(self.model, self.scratch)
            mujoco.mj_comPos(self.model, self.scratch)
            current = self.scratch.site_xmat[self.site].reshape(3, 3)
            position_error = xyz - self.scratch.site_xpos[self.site]
            rotation_error = 0.5 * np.cross(current.T, self.rotation.T).sum(axis=0)
            if np.linalg.norm(position_error) < 0.002 and np.linalg.norm(rotation_error) < 0.02:
                return self.scratch.qpos[self.qidx].copy()
            mujoco.mj_jacSite(self.model, self.scratch, self.jp, self.jr, self.site)
            jac = np.vstack([self.jp[:, self.vidx], self.jr[:, self.vidx]])
            error = np.r_[position_error, rotation_error]
            dq = jac.T @ np.linalg.solve(jac @ jac.T + 0.0025 * np.eye(6), error)
            self.scratch.qpos[self.qidx] = np.clip(
                self.scratch.qpos[self.qidx] + np.clip(dq, -0.08, 0.08),
                self.limits[:, 0] + 0.01, self.limits[:, 1] - 0.01,
            )
        raise ValueError("IK could not reach waypoint with the fixed gripper orientation")

    @property
    def xyz(self):
        return self.data.site_xpos[self.site].copy()

    def step(self, goal):
        # 0.5 rad/s setpoint ramp at the environment's 20 Hz control rate.
        self.hold += np.clip(goal - self.hold, -0.025, 0.025)
        action = self.robot.composite_controller.create_action_vector({
            "right": self.hold - self.data.qpos[self.qidx],
            "right_gripper": [1 if self.gripper == "close" else -1], "base_mode": -1,
        })
        self.env.step(action)


def run(args):
    if args.list_tasks:
        from robocasa.environments import ALL_KITCHEN_ENVIRONMENTS
        print("\n".join(sorted(set(ALL_KITCHEN_ENVIRONMENTS) - {"Kitchen"})))
        return
    if args.task:
        if args.pickup or args.object:
            raise ValueError("Use --task by itself; --pickup and --object select custom objectives.")
        from robocasa.environments import ALL_KITCHEN_ENVIRONMENTS
        if args.task not in ALL_KITCHEN_ENVIRONMENTS or args.task == "Kitchen":
            raise ValueError(f"Unknown task {args.task!r}; use --list-tasks.")
        args.env = args.task
    if args.pickup and not args.object:
        args.object = "obj"
    if args.max_calls is None:
        args.max_calls = 30 if args.task else (12 if args.pickup else 5)
    if args.max_calls < 1:
        raise ValueError("--max-calls must be positive")
    if not (args.mock or args.inspect) and not os.environ.get("OPENAI_API_KEY"):
        raise SystemExit("Set OPENAI_API_KEY, or run --mock / --inspect without API calls.")
    print(f"Loading RoboCasa365 {args.env} (first load may take a while)...", flush=True)
    load_started = time.monotonic()
    log = RunLog(args)
    env = None
    pool = ThreadPoolExecutor(max_workers=1)
    cameras = None
    status = "completed"
    try:
        env = make_env(args)
        env.reset()
        print(f"Kitchen loaded in {time.monotonic() - load_started:.1f}s.", flush=True)
        arm = Arm(env)
        for _ in range(20):
            arm.step(arm.hold.copy())
        arm.rotation = arm.data.site_xmat[arm.site].reshape(3, 3).copy()
        task_instruction = env.get_ep_meta()["lang"] if args.task else None
        if args.task and not task_instruction.strip():
            raise ValueError(f"{args.task} did not provide a task instruction")
        display_cameras = not args.headless and not args.no_camera_views and not args.inspect
        cameras = CameraViews(env, display=display_cameras)
        log.frame(cameras.capture())
        if args.inspect:
            state = vision_state(env, arm.site) if args.vision else scene_state(env, arm.site)
            if args.task:
                state["robocasa_task"] = ({"name": args.task, "instruction": task_instruction}
                                          if args.vision else task_state(env, task_instruction))
            if args.vision:
                state["camera_images"] = [{"name": name, "shape": list(image.shape)}
                                          for name, image in zip(CAMERAS, cameras.capture())]
            print(json.dumps(state, indent=2))
            log.event("Inspection", state)
            return
        initial_object_xyz = None
        if args.object:
            if args.object not in env.objects:
                raise ValueError(f"Unknown object {args.object}; available: {list(env.objects)}")
            body = env.sim.model.body_name2id(env.objects[args.object].root_body)
            initial_object_xyz = env.sim.data.body_xpos[body].copy()
            target = env.sim.data.body_xpos[body].copy() + [0, 0, 0.70]
            instruction = f"Move the tip to the cabinet" # 0.20 m above the root-body origin of object '{args.object}'."
        else:
            target = arm.xyz + [0, 0, 0.05]
            instruction = f"Move the tip to world XYZ {target.tolist()} (5 cm above its initial position)."
        if args.pickup:
            instruction = (f"Pick up object '{args.object}' and hold it at least 0.08 m above "
                           "its initial height. Approach, descend, close, then lift. Do not release.")
            if args.vision:
                instruction = (f"Pick up the {env.get_obj_lang(args.object)} visible in the images "
                               "and hold it at least 8 cm above its starting height. Do not release.")
        if args.task:
            instruction = task_instruction
            print(f"Task: {args.task}; success evaluator: env._check_success()", flush=True)
            if args.mock:
                print("Mock task mode holds position to test wiring; it does not solve the task.", flush=True)
        if args.instruction:
            instruction = args.instruction
        print(instruction, flush=True)
        log.event("Objective", {"instruction": instruction, "mock": args.mock,
                                "observation_mode": "vision" if args.vision else "numerical"})
        print("Astra observation: " + ("3 RGB images + robot proprioception; no world object state."
              if args.vision else "all objects and fixtures; no camera images sent."), flush=True)
        if args.vision and args.mock:
            print("Vision mock holds position: camera/API wiring test, not a visual policy.", flush=True)
        feedback, future, moving = "Initial state", None, False
        goal, calls, motion_steps, stable = arm.hold.copy(), 0, 0, 0
        mock_phase, lift_stable = 0, 0
        while True:
            started = time.monotonic()
            if args.task and not moving and bool(env._check_success()):
                log.event("Success", {"task": args.task, "official_success": True})
                print(f"SUCCESS: RoboCasa {args.task} env._check_success() = True", flush=True)
                return
            if future is None and not moving:
                if calls >= args.max_calls:
                    result = f"RoboCasa {args.task} success=False. " if args.task else ""
                    raise RuntimeError(f"{result}Action budget exhausted: {feedback}")
                observation = vision_state(env, arm.site) if args.vision else scene_state(env, arm.site)
                observation.update(instruction=instruction, last_action_result=feedback)
                if args.task:
                    observation["robocasa_task"] = ({"name": args.task, "instruction": task_instruction}
                                                   if args.vision else task_state(env, task_instruction))
                observation["gripper"] = {
                    "command": arm.gripper,
                    "finger_joint_positions": [float(env.sim.data.get_joint_qpos(j))
                                               for j in arm.robot.gripper["right"].joints],
                }
                if args.pickup and not args.vision:
                    observation["pickup"] = {
                        "object": args.object, "initial_object_xyz": initial_object_xyz.tolist(),
                        "grasped": bool(env._check_grasp(arm.robot.gripper["right"], env.objects[args.object])),
                        "lift_m": float(env.sim.data.body_xpos[body, 2] - initial_object_xyz[2]),
                    }
                    if args.mock:
                        observation["pickup"]["mock_phase"] = mock_phase
                views = cameras.capture()
                log.snapshot(views, calls + 1)
                images = views if args.vision else None
                print(f"Request {calls + 1}: " + ("3 camera images" if args.vision else
                      f"{len(observation['objects'])} objects, {len(observation['fixtures'])} fixtures"), flush=True)
                if args.mock:
                    log.event("Mock input", {"observation": observation}, calls + 1)
                future = pool.submit(choose_position, observation, args.mock, target, images, log, calls + 1)
                calls += 1
            if future is not None and future.done():
                command = future.result()
                if args.mock:
                    log.event("Mock output", {"action": command.model_dump()}, calls)
                waypoint = np.array([command.x, command.y, command.z])
                future = None
                if command.note:
                    print(f"Note: {command.note}", flush=True)
                try:
                    goal = arm.solve(waypoint)
                    arm.gripper = command.gripper
                    moving, motion_steps, stable = True, 0, 0
                    print(f"XYZ {waypoint.round(4)}, gripper={arm.gripper} -> IK joints {goal.round(3)}", flush=True)
                    log.event("Action accepted", {"action": command.model_dump(), "ik_joints": goal.tolist(),
                                                   "simulation_steps": 160}, calls)
                except ValueError as error:
                    feedback = f"Rejected waypoint {waypoint.tolist()}: {error}"
                    print(feedback, flush=True)
                    log.event("Action rejected", {"reason": feedback}, calls)
            # Freeze physics during API waits; render the unchanged scene below.
            if moving:
                arm.step(goal)
                views = cameras.capture()
                log.frame(views)
                cameras.update(views)
            else:
                cameras.update()
            if args.pickup and moving:
                lift = float(env.sim.data.body_xpos[body, 2] - initial_object_xyz[2])
                grasped = bool(env._check_grasp(arm.robot.gripper["right"], env.objects[args.object]))
                lift_stable = lift_stable + 1 if lift >= 0.08 and grasped and arm.gripper == "close" else 0
                if lift_stable >= 10 and motion_steps + 1 >= 160:
                    log.event("Success", {"object_lift_m": lift, "grasped": grasped}, calls)
                    print(f"SUCCESS: object grasped and lifted {lift:.3f} m for 10 updates.", flush=True)
                    return
            if not args.headless:
                env.render()
                if env.viewer.viewer is not None and not env.viewer.viewer.is_running():
                    print("Viewer closed.")
                    status = "viewer closed"
                    return
            if moving:
                motion_steps += 1
                error = float(np.linalg.norm(arm.xyz - target))
                waypoint_error = float(np.linalg.norm(arm.xyz - waypoint))
                stable = stable + 1 if waypoint_error < 0.01 else 0
                # Each accepted action gets exactly 8 simulated seconds at 20 Hz.
                if motion_steps >= 160:
                    moving = False
                    feedback = (f"Actual tip {arm.xyz.tolist()}; waypoint error {waypoint_error:.4f} m; "
                                f"task target error {error:.4f} m")
                    if args.task:
                        feedback = (f"Actual tip {arm.xyz.tolist()}; waypoint error {waypoint_error:.4f} m; "
                                    f"gripper={arm.gripper}; RoboCasa success={bool(env._check_success())}")
                    if args.pickup:
                        feedback = (f"Actual tip {arm.xyz.tolist()}; waypoint error {waypoint_error:.4f} m; "
                                    f"gripper={arm.gripper}; grasped={grasped}; object lift={lift:.4f} m")
                        if args.mock and stable >= 10:
                            phase_target = initial_object_xyz + [0, 0, MOCK_PICKUP_Z[min(mock_phase, 3)]]
                            if np.linalg.norm(arm.xyz - phase_target) < 0.01:
                                mock_phase += 1
                    if args.vision:
                        feedback = (f"Actual tip {arm.xyz.tolist()}; waypoint error {waypoint_error:.4f} m; "
                                    f"gripper command={arm.gripper}. Inspect the new images to assess progress.")
                    print(feedback, flush=True)
                    log.event("Action result", {"feedback": feedback}, calls)
                    if not args.task and not args.pickup and error < 0.01 and stable >= 10:
                        print("SUCCESS: reaching objective achieved (not the kitchen pick/place task).", flush=True)
                        log.event("Success", {"reaching_error_m": error}, calls)
                        return
            if not args.headless or future is not None:
                time.sleep(max(0, 0.05 - (time.monotonic() - started)))
    except BaseException as error:
        status = ("interrupted" if isinstance(error, KeyboardInterrupt) else
                  "failed" if isinstance(error, RuntimeError) and "Action budget exhausted" in str(error)
                  else "error")
        log.event("Run error", {"type": type(error).__name__, "message": str(error)})
        raise
    finally:
        # Let an in-flight response finish writing before finalizing the transcript.
        pool.shutdown(wait=True, cancel_futures=True)
        try:
            if cameras:
                cameras.close()
            if env is not None:
                env.close()
        finally:
            log.close(status)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mock", action="store_true", help="Deterministic XYZ; no API call")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--vision", action="store_true", help="Send three RGB cameras plus robot proprioception instead of world object state")
    parser.add_argument("--no-camera-views", action="store_true", help="Disable the three-view browser display")
    parser.add_argument("--inspect", action="store_true", help="Print numerical observation and exit")
    parser.add_argument("--object", help="Object name for the custom reaching goal or --pickup")
    parser.add_argument("--instruction", help="Override the goal sent to Astra; existing success checks still apply")
    parser.add_argument("--pickup", action="store_true", help="Pick up --object (default: obj) and hold it lifted")
    parser.add_argument("--env", default="PickPlaceCounterToCabinet")
    parser.add_argument("--task", metavar="NAME", help="Run a RoboCasa task's instruction and official success check")
    parser.add_argument("--list-tasks", action="store_true", help="List installed RoboCasa task names and exit")
    parser.add_argument("--layout", type=int, default=1)
    parser.add_argument("--style", type=int, default=1)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-calls", type=int, default=None, help="Action budget (default: 5 reaching, 12 pickup, 30 task)")
    try:
        run(parser.parse_args())
    except (RuntimeError, ValueError) as error:
        raise SystemExit(str(error)) from error
