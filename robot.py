"""
RoboCasa365: numerical scene state -> Astra XYZ -> IK -> joint controller.

Usage:
mjpython robot.py --task TurnOnElectricKettle \
 --seed 6 --vision --max-calls 20 \
 --instruction "Press down the lever under the kettle's handle to turn on the electric kettle."

mjpython robot.py --task PickPlaceCounterToBlender \
 --seed 6 --vision --max-calls 20 \
 --instruction "Pick the pear by closing the gripper securely around the big base of the pear. Then place it in the blender. "

"""

import argparse
from collections import deque
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

import mujoco
import numpy as np
from openai import OpenAI
from pydantic import BaseModel, Field
from scipy.spatial.transform import Rotation
from run_log import RunLog

# Offline pickup phases: above, descend, close, lift. Tuned for the default pear.
MOCK_PICKUP_Z = [0.15, -0.005, -0.005, 0.15]
CAMERAS = ["robot0_agentview_left", "robot0_agentview_right", "robot0_eye_in_hand"]


class RobotFrame:
    """Episode-fixed, upright frame at the Panda mounting body (link0)."""
    description = ("episode_robot; meters; origin at initial Panda arm mount; "
                   "+X forward, +Y left, +Z up; fixed throughout episode. "
                   "All body XYZ and orientations below use this frame; arm joints remain joint-local.")

    def __init__(self, env):
        self.body_name = env.robots[0].robot_model.naming_prefix + "link0"
        body = env.sim.model.body_name2id(self.body_name)
        self.origin = env.sim.data.body_xpos[body].copy()
        mount = env.sim.data.body_xmat[body].reshape(3, 3)
        # Panda link0 +X is forward. Remove tiny settling tilt so +Z is exactly up.
        forward = mount[:, 0].copy()
        forward[2] = 0
        forward /= np.linalg.norm(forward)
        up = np.array([0., 0., 1.])
        self.rotation = np.column_stack([forward, np.cross(up, forward), up])

    def to_robot(self, world_xyz):
        return self.rotation.T @ (np.asarray(world_xyz) - self.origin)

    def to_world(self, robot_xyz):
        return self.rotation @ np.asarray(robot_xyz) + self.origin

    def orientation(self, world_rotation):
        return self.rotation.T @ np.asarray(world_rotation).reshape(3, 3)

    def target_rotation(self, yaw_pitch_roll, start_rotation):
        """Absolute offsets: R_world = R_frame Rz(yaw) Ry(pitch) Rx(roll) R_start_robot."""
        delta = Rotation.from_euler("ZYX", yaw_pitch_roll).as_matrix()
        return self.rotation @ delta @ self.rotation.T @ start_rotation

    def measured_ypr(self, world_rotation, start_rotation):
        delta = self.rotation.T @ world_rotation @ start_rotation.T @ self.rotation
        return Rotation.from_matrix(delta).as_euler("ZYX")


class CameraViews:
    """Local browser display avoids competing macOS GUI event loops."""
    def __init__(self, env, robot_frame, display=True):
        self.env, self.frame, self.last_update = env, b"", 0.0
        self.robot_frame = robot_frame
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
            images.append(self.draw_axes(self.renderer.render().copy(), name))
        return images

    def draw_axes(self, pixels, camera):
        """Project virtual rulers using the current camera pose, including wrist motion."""
        from PIL import Image, ImageDraw
        model, data = self.env.sim.model, self.env.sim.data
        camera_id = model.camera_name2id(camera)
        rotation = data.cam_xmat[camera_id].reshape(3, 3)
        height, width = pixels.shape[:2]
        focal = height / (2 * np.tan(np.deg2rad(model.cam_fovy[camera_id]) / 2))

        def project(world):
            point = rotation.T @ (world - data.cam_xpos[camera_id])
            if point[2] >= -0.01:
                return None
            return np.array([width / 2 + focal * point[0] / -point[2],
                             height / 2 - focal * point[1] / -point[2]])

        canvas = Image.fromarray(pixels)
        draw = ImageDraw.Draw(canvas)
        tip = data.site_xpos[self.env.robots[0].eef_site_id["right"]]
        # The mount may be outside the wrist view; a translated copy at the tip
        # shows the same fixed directions without changing the action origin.
        for origin, length, tag in [(self.robot_frame.origin, 0.15, "mount"),
                                    (tip, 0.10, "tip")]:
            start = project(origin)
            if start is None or not (0 <= start[0] < width and 24 <= start[1] < height - 25):
                continue
            draw.ellipse(tuple(start - 2) + tuple(start + 2), fill="white")
            for axis, color in enumerate(["#ff5050", "#50ff70", "#5599ff"]):
                end = project(origin + self.robot_frame.rotation[:, axis] * length)
                if end is None or not (4 <= end[0] < width - 20 and 25 <= end[1] < height - 25):
                    continue
                draw.line([tuple(start), tuple(end)], fill=color, width=2)
                delta = end - start
                norm = np.linalg.norm(delta)
                if norm > 5:
                    direction = delta / norm
                    side = np.array([-direction[1], direction[0]])
                    draw.polygon([tuple(end), tuple(end - 6 * direction + 3 * side),
                                  tuple(end - 6 * direction - 3 * side)], fill=color)
                draw.text(tuple(end + [2, -5]), "+" + "XYZ"[axis], fill=color, stroke_width=1, stroke_fill="black")
            if tag == "mount":
                draw.text(tuple(start + [3, 3]), tag, fill="white", stroke_width=1, stroke_fill="black")
        draw.rectangle((0, height - 23, width, height), fill="black")
        draw.text((4, height - 21), "Robot axes: mount 15cm / tip copy 10cm", fill="white")
        return np.asarray(canvas)

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
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    gripper: float = Field(ge=0.0, le=1.0, description="Absolute opening: 0 fully closed, 1 fully open")
    note: str = ""


def action_result(call, command, start, end, rejection=None, orientation_error=None,
                  measured_ypr=None):
    """Robot-only feedback; reaching a waypoint does not imply task success."""
    target = np.array([command.x, command.y, command.z])
    requested = target - start
    measured = end - start
    error = float(np.linalg.norm(target - end))
    requested_distance = float(np.linalg.norm(requested))
    # Use signed progress in the requested direction. Sideways or opposite
    # motion cannot make a short command look successful merely because its
    # target began within the 1 cm endpoint tolerance.
    progress = (1.0 if requested_distance < 0.001 else
                float(np.dot(measured, requested) / requested_distance**2))
    reached = (rejection is None and error < 0.01 and progress >= 0.8
               and orientation_error is not None and orientation_error < 0.05)
    result = (f"Action rejected; no motion executed: {rejection}" if rejection else
              "Pose reached (within 1 cm, 0.05 rad, and at least 80% requested position progress)." if reached else
              "Target not reached; motion may be obstructed or there may be another cause.")
    return {
        "call": call, "coordinate_frame": "episode_robot; meters",
        "action": command.model_dump(),
        "start_tip_xyz": np.asarray(start).round(5).tolist(),
        "end_tip_xyz": np.asarray(end).round(5).tolist(),
        "requested_displacement_xyz": requested.round(5).tolist(),
        "measured_displacement_xyz": measured.round(5).tolist(),
        "requested_progress_fraction": round(progress, 4),
        "waypoint_error_m": round(error, 5),
        "orientation_error_rad": None if orientation_error is None else round(orientation_error, 5),
        "measured_yaw_pitch_roll": None if measured_ypr is None else np.asarray(measured_ypr).round(5).tolist(),
        "status": "rejected" if rejection else "reached" if reached else "not_reached",
        "result": result,
    }


def prompt_history(actions):
    """Compact robot feedback for Astra; detailed summaries remain in local traces."""
    compact = []
    note_from = max(0, len(actions) - 2)
    for index, item in enumerate(actions):
        entry = {
            "call": item["call"],
            "target_xyz": [item["action"][axis] for axis in "xyz"],
            "target_ypr": [item["action"][axis] for axis in ("yaw", "pitch", "roll")],
            "measured_ypr": item["measured_yaw_pitch_roll"],
            "orientation_error_rad": item["orientation_error_rad"],
            "gripper": item["action"]["gripper"],
            "requested_delta_xyz": item["requested_displacement_xyz"],
            "measured_delta_xyz": item["measured_displacement_xyz"],
            "progress_fraction": item["requested_progress_fraction"],
            "error_m": item["waypoint_error_m"],
            "status": item["status"],
        }
        if index >= note_from and item["action"].get("note"):
            entry["note"] = item["action"]["note"]
        compact.append(entry)
    return compact


def make_env(args):
    # Also seed legacy helpers that use global RNGs (e.g. camera perturbations).
    random.seed(args.seed)
    np.random.seed(args.seed)
    import robocasa  # Registers RoboCasa tasks with robosuite.
    import robosuite
    from robosuite.controllers import load_composite_controller_config
    from robosuite.models.grippers import PandaGripper, register_gripper

    @register_gripper
    class AbsolutePandaGripper(PandaGripper):
        """Keep Panda geometry/actuators; replace directional input with aperture targets."""
        def format_action(self, action):
            # [-1, +1] here means closed -> open. The two finger position
            # actuators have opposite ranges, so their normalized goals differ.
            opening = float(np.clip(action[0], -1, 1))
            target = np.array([opening, -opening])
            self.current_action = self.current_action + np.clip(
                target - self.current_action, -self.speed, self.speed)
            return self.current_action

    config = load_composite_controller_config(robot="PandaOmron")
    config["body_parts"]["right"] = {
        "type": "JOINT_POSITION", "input_type": "delta",
        "input_max": 1, "input_min": -1, "output_max": 1, "output_min": -1,
        "interpolation": None,
        "kp": 150, "damping_ratio": 1, "gripper": {"type": "GRIP"},
    }
    return robosuite.make(
        args.env, robots="PandaOmron", gripper_types="AbsolutePandaGripper", controller_configs=config,
        has_renderer=not args.headless,
        has_offscreen_renderer=False,
        use_camera_obs=False, use_object_obs=True,
        control_freq=20, ignore_done=True, seed=args.seed,
        layout_ids=[args.layout], style_ids=[args.style],
        generative_textures=None, obj_registries=("objaverse",),
        robot_spawn_deviation_pos_x=0, robot_spawn_deviation_pos_y=0,
    )


def scene_state(env, site, frame):
    """All episode objects and fixtures, including articulated joint positions."""
    def joint_state(name):
        value = np.asarray(env.sim.data.get_joint_qpos(name)).copy()
        joint = env.sim.model.joint_name2id(name)
        if env.sim.model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE:
            value[:3] = frame.to_robot(value[:3])
            rotation = np.empty(9)
            mujoco.mju_quat2Mat(rotation, value[3:])
            mujoco.mju_mat2Quat(value[3:], frame.orientation(rotation).ravel())
        return value.round(5).tolist()

    def state(item):
        body = env.sim.model.body_name2id(item.root_body)
        quaternion = np.empty(4)
        mujoco.mju_mat2Quat(quaternion, frame.orientation(env.sim.data.body_xmat[body]).ravel())
        result = {
            "type": type(item).__name__,
            "xyz": frame.to_robot(env.sim.data.body_xpos[body]).round(5).tolist(),
            "quaternion_wxyz": quaternion.round(5).tolist(),
            "joints": {
                name: joint_state(name)
                for name in item.joints
            },
            "joint_coordinates": "free joints: episode_robot xyz + wxyz; other joints: local joint coordinates",
        }
        if getattr(item, "size", None) is not None:
            result["size"] = np.asarray(item.size).round(5).tolist()
            result["size_frame"] = "item local axes; meters"
        return result

    objects = {name: state(item) for name, item in env.objects.items()}
    for name in objects:
        objects[name]["description"] = env.get_obj_lang(name)
    return {
        "coordinate_frame": frame.description + " Quaternion order wxyz.",
        "tip_xyz": frame.to_robot(env.sim.data.site_xpos[site]).round(5).tolist(),
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


def vision_state(env, site, frame):
    # Explicit allowlist: no object/fixture poses, dimensions, contacts or task metrics.
    return {
        "input_mode": "vision",
        "coordinate_frame": frame.description,
        "tip_xyz": frame.to_robot(env.sim.data.site_xpos[site]).round(5).tolist(),
        "robot_qpos": env.robots[0]._joint_positions.round(5).tolist(),
        "tip_rotation_matrix": frame.orientation(env.sim.data.site_xmat[site]).round(5).tolist(),
    }


def choose_position(observation, mock, target, images=None, log=None, call=None):
    if mock:
        tip = np.array(observation["tip_xyz"])
        if "robocasa_task" in observation or observation.get("input_mode") == "vision":
            # Connectivity/evaluator smoke test only; no scripted benchmark solver.
            return Position(x=tip[0], y=tip[1], z=tip[2], gripper=1.0)
        gripper = 1.0
        if "pickup" in observation:
            pickup = observation["pickup"]
            phase = min(pickup["mock_phase"], 3)
            origin = np.array(pickup["initial_object_xyz"])
            target = origin + [0, 0, MOCK_PICKUP_Z[phase]]
            gripper = 0.0 if phase >= 2 else 1.0
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
                    "Each observation gives proprioception and either numerical scene state or three camera images. "
                    "Work toward the user's goal in small, deliberate motions; re-check the observation after every motion. "
                    "Respond only with a JSON object matching the provided schema: an absolute episode-robot-frame XYZ waypoint in meters (x, y, z), "
                    "yaw, pitch, roll in radians, a continuous gripper opening from 0 to 1, and a `note`. "
                    "Do not write any text outside the JSON object. "
                    "In the `note`, in one or two sentences, say what you observe in the current observation and why you chose this motion. "
                    "The user is watching these notes to see what you see and what you decide, so write them for a human reader. "
                    # "Use the supplied observations. In vision mode infer object/fixture locations "
                    # "from the three labeled RGB views; numerical values describe only your robot. "
                    # "Metric depth is uncertain: choose small exploratory waypoints and reobserve. "
                    "Embodiment: one 7-DoF Panda arm on an Omron mobile base, with a parallel-jaw gripper. "
                    "The coordinate origin is the Panda arm mounting base after initialization, not the floor. "
                    "+X is forward out of the robot, +Y is the robot's left, +Z is up. "
                    "This frame is frozen for the episode; these are not image-left/right directions. "
                    "XYZ controls the grasp point between the fingers. Targets are absolute positions, not displacements. "
                    "Gripper is an absolute opening target: 0 fully closed, 1 fully open, 0.5 half open. "
                    "Full finger travel gives approximately 0.08 meters opening; 0.5 corresponds to about 0.04 meters. "
                    "It is not a velocity or force command. Contact can stop the fingers before the requested opening. "
                    "Compare measured_opening with command to assess aperture; this alone does not prove a secure grasp. "
                    "You control XYZ, wrist yaw/pitch/roll, and gripper opening. Choose wrist orientation as needed for good alignment and avoiding collisions. Angles are ABSOLUTE offsets from "
                    "the episode's starting gripper orientation, not increments from the latest pose. "
                    "All zero angles restore that starting orientation. To keep the current orientation, "
                    "copy tip_yaw_pitch_roll from the observation. Use radians, not degrees. "
                    "The rotation convention is Rz(yaw) Ry(pitch) Rx(roll) applied to the starting "
                    "orientation in the fixed episode robot frame: roll about robot +X, then pitch "
                    "about robot +Y, then yaw about robot +Z. Positive angles follow the right-hand rule; "
                    "positive yaw turns counterclockwise viewed from above. Prefer small angular changes "
                    "(about 0.1-0.2 radians) and reobserve. Rotation is about the grasp point; account "
                    "for the palm and fingers sweeping through space. IK does not check collisions. "
                    "The controller does not intentionally drive the base or torso, but small physical drift is possible. "
                    "Camera overlays show +X red, +Y green, +Z blue: 15cm arrows at the mount, "
                    "and a parallel 10cm copy at the current grasp point. The tip copy is not the coordinate origin. "
                    "These virtual rulers may be hidden by the image boundary and are drawn over scene objects. "
                    "IK converts targets to joint motion; unreachable targets are rejected and contacts may block motion. "
                    "Use modest steps and the measured result, rather than assuming every target was reached. "
                    "recent_actions contains up to five previous actions, oldest first, with targets, gripper commands, "
                    "requested versus measured XYZ displacement, progress fraction along the requested direction, "
                    "tracking error, and status in the episode robot frame. "
                    "Pose reached requires position error below 1 cm, orientation error below 0.05 radians, "
                    "and at least 80% position progress (waived for position changes below 1 mm). "
                    "Only the latest two actions retain your notes to keep the context concise. "
                    "Use this history to assess progress and change your approach when movements repeatedly fail. "
                    "Target not reached means motion may be obstructed or there may be another cause; "
                    "tracking error alone does not prove contact. Do not simply repeat blocked movements. "
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
        self.gripper = 1.0

    @property
    def gripper_opening(self):
        fingers = [float(self.env.sim.data.get_joint_qpos(j)) for j in self.robot.gripper["right"].joints]
        return float(np.clip((fingers[0] - fingers[1]) / 0.08, 0, 1))

    def solve(self, xyz, rotation=None):
        rotation = self.rotation if rotation is None else rotation
        # if not np.isfinite(xyz).all() or np.linalg.norm(xyz - self.xyz) > 0.35:
        #     raise ValueError("Waypoint must be finite and within 0.35 m of the current tip")
        # Solve on scratch data: never teleport the actual simulated robot.
        self.scratch.qpos[:] = self.data.qpos
        for _ in range(150):
            mujoco.mj_kinematics(self.model, self.scratch)
            mujoco.mj_comPos(self.model, self.scratch)
            current = self.scratch.site_xmat[self.site].reshape(3, 3)
            position_error = xyz - self.scratch.site_xpos[self.site]
            # SO(3) rotation vector remains valid at 180 degrees, where the old
            # cross-product error vanishes despite the orientation being wrong.
            rotation_error = Rotation.from_matrix(rotation @ current.T).as_rotvec()
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
        raise ValueError("IK could not reach the requested position and orientation")

    @property
    def xyz(self):
        return self.data.site_xpos[self.site].copy()

    def step(self, goal):
        # 0.5 rad/s setpoint ramp at the environment's 20 Hz control rate.
        self.hold += np.clip(goal - self.hold, -0.025, 0.025)
        action = self.robot.composite_controller.create_action_vector({
            "right": self.hold - self.data.qpos[self.qidx],
            "right_gripper": [2 * self.gripper - 1], "base_mode": -1,
        })
        self.env.step(action)


def test_axes(arm, frame, cameras, log):
    """Offline integration check through the same transform, IK and controller."""
    home = arm.xyz.copy()
    for axis in range(3):
        for sign in (1, -1):
            before = arm.xyz
            delta = np.eye(3)[axis] * sign * 0.03
            target = frame.to_world(frame.to_robot(before) + delta)
            goal = arm.solve(target)
            for _ in range(160):
                arm.step(goal)
                views = cameras.capture()
                log.frame(views)
                cameras.update(views)
            actual_delta = arm.xyz - before
            expected_delta = frame.rotation @ delta
            error = float(np.linalg.norm(actual_delta - expected_delta))
            label = f"{'+' if sign > 0 else '-'}{'XYZ'[axis]}"
            result = {"axis": label, "expected_world_delta": expected_delta.tolist(),
                      "actual_world_delta": actual_delta.tolist(), "error_m": error}
            log.event("Axis test", result)
            print(f"{label} 3cm: expected world {expected_delta.round(5)}, "
                  f"measured {actual_delta.round(5)}; error {error:.4f} m", flush=True)
            if error > 0.005:
                raise RuntimeError(f"Axis test {label} exceeded 5mm error; check reachability or contact.")
            goal = arm.solve(home)
            for _ in range(160):
                arm.step(goal)
                views = cameras.capture()
                log.frame(views)
                cameras.update(views)
    log.event("Success", {"axis_test": "All six 3cm movements tracked within 5mm"})


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
    if not (args.mock or args.inspect or args.test_axes) and not os.environ.get("OPENAI_API_KEY"):
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
        frame = RobotFrame(env)
        log.event("Coordinate frame", {"name": "episode_robot", "mount_body": frame.body_name,
                    "origin_world_xyz": frame.origin.tolist(), "rotation_robot_to_world": frame.rotation.tolist(),
                    "start_gripper_rotation_world": arm.rotation.tolist()})
        task_instruction = env.get_ep_meta()["lang"] if args.task else None
        if args.task and not task_instruction.strip():
            raise ValueError(f"{args.task} did not provide a task instruction")
        display_cameras = not args.headless and not args.no_camera_views and not args.inspect
        cameras = CameraViews(env, frame, display=display_cameras)
        log.frame(cameras.capture())
        if args.test_axes:
            test_axes(arm, frame, cameras, log)
            return
        if args.inspect:
            state = vision_state(env, arm.site, frame) if args.vision else scene_state(env, arm.site, frame)
            state["tip_yaw_pitch_roll"] = frame.measured_ypr(
                arm.data.site_xmat[arm.site].reshape(3, 3), arm.rotation).round(5).tolist()
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
            instruction = f"Move the tip to episode_robot XYZ {frame.to_robot(target).tolist()} (5 cm above its initial position)."
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
        recent_actions = deque(maxlen=5)
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
                observation = vision_state(env, arm.site, frame) if args.vision else scene_state(env, arm.site, frame)
                observation["tip_yaw_pitch_roll"] = frame.measured_ypr(
                    arm.data.site_xmat[arm.site].reshape(3, 3), arm.rotation).round(5).tolist()
                observation.update(instruction=instruction,
                                   recent_actions=prompt_history(recent_actions))
                if args.task:
                    observation["robocasa_task"] = ({"name": args.task, "instruction": task_instruction}
                                                   if args.vision else task_state(env, task_instruction))
                observation["gripper"] = {
                    "command": arm.gripper,
                    "measured_opening": arm.gripper_opening,
                    "opening_convention": "0 closed, 1 open; absolute aperture target",
                    "finger_joint_positions": [float(env.sim.data.get_joint_qpos(j))
                                               for j in arm.robot.gripper["right"].joints],
                }
                if args.pickup and not args.vision:
                    observation["pickup"] = {
                        "object": args.object, "initial_object_xyz": frame.to_robot(initial_object_xyz).tolist(),
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
                future = pool.submit(choose_position, observation, args.mock, frame.to_robot(target), images, log, calls + 1)
                calls += 1
            if future is not None and future.done():
                command = future.result()
                if args.mock:
                    log.event("Mock output", {"action": command.model_dump()}, calls)
                robot_waypoint = np.array([command.x, command.y, command.z])
                waypoint = frame.to_world(robot_waypoint)
                action_start = frame.to_robot(arm.xyz)
                future = None
                if command.note:
                    print(f"Note: {command.note}", flush=True)
                try:
                    ypr = np.array([command.yaw, command.pitch, command.roll])
                    if not np.isfinite(np.r_[waypoint, ypr]).all():
                        raise ValueError("Position and orientation must be finite")
                    target_rotation = frame.target_rotation(ypr, arm.rotation)
                    goal = arm.solve(waypoint, target_rotation)
                    arm.gripper = command.gripper
                    moving, motion_steps, stable = True, 0, 0
                    print(f"Robot XYZ {robot_waypoint.round(4)}, YPR {ypr.round(4)} rad -> world XYZ {waypoint.round(4)}, gripper={arm.gripper} -> IK joints {goal.round(3)}", flush=True)
                    log.event("Action accepted", {"action": command.model_dump(), "ik_joints": goal.tolist(),
                                                   "simulation_steps": 160, "coordinate_frame": "episode_robot",
                                                   "world_target_xyz": waypoint.tolist(),
                                                   "world_target_rotation": target_rotation.tolist()}, calls)
                except ValueError as error:
                    feedback = f"Rejected episode_robot waypoint {robot_waypoint.tolist()}, YPR {ypr.tolist()}: {error}"
                    summary = action_result(calls, command, action_start, frame.to_robot(arm.xyz), str(error),
                                            measured_ypr=frame.measured_ypr(
                                                arm.data.site_xmat[arm.site].reshape(3, 3), arm.rotation))
                    recent_actions.append(summary)
                    print(feedback, flush=True)
                    log.event("Action rejected", {"reason": feedback, "action_summary": summary}, calls)
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
                lift_stable = lift_stable + 1 if lift >= 0.08 and grasped else 0
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
                    feedback = (f"Actual tip (episode_robot) {frame.to_robot(arm.xyz).tolist()}; waypoint error {waypoint_error:.4f} m; "
                                f"task target error {error:.4f} m")
                    if args.task:
                        feedback = (f"Actual tip (episode_robot) {frame.to_robot(arm.xyz).tolist()}; waypoint error {waypoint_error:.4f} m; "
                                    f"gripper={arm.gripper}; RoboCasa success={bool(env._check_success())}")
                    if args.pickup:
                        feedback = (f"Actual tip (episode_robot) {frame.to_robot(arm.xyz).tolist()}; waypoint error {waypoint_error:.4f} m; "
                                    f"gripper={arm.gripper}; grasped={grasped}; object lift={lift:.4f} m")
                        if args.mock and stable >= 10:
                            phase_target = initial_object_xyz + [0, 0, MOCK_PICKUP_Z[min(mock_phase, 3)]]
                            if np.linalg.norm(arm.xyz - phase_target) < 0.01:
                                mock_phase += 1
                    if args.vision:
                        feedback = (f"Actual tip (episode_robot) {frame.to_robot(arm.xyz).tolist()}; waypoint error {waypoint_error:.4f} m; "
                                    f"gripper command={arm.gripper}. Inspect the new images to assess progress.")
                    current_rotation = arm.data.site_xmat[arm.site].reshape(3, 3)
                    orientation_error = float(Rotation.from_matrix(target_rotation @ current_rotation.T).magnitude())
                    summary = action_result(calls, command, action_start, frame.to_robot(arm.xyz),
                                            orientation_error=orientation_error,
                                            measured_ypr=frame.measured_ypr(current_rotation, arm.rotation))
                    recent_actions.append(summary)
                    feedback += (f" Requested displacement {summary['requested_displacement_xyz']} m; "
                                 f"measured displacement {summary['measured_displacement_xyz']} m "
                                 f"(episode_robot); actual YPR {summary['measured_yaw_pitch_roll']} rad; "
                                 f"orientation error {orientation_error:.4f} rad. {summary['result']}")
                    print(feedback, flush=True)
                    log.event("Action result", {"feedback": feedback, "tip_robot_xyz": frame.to_robot(arm.xyz).tolist(),
                                               "tip_world_xyz": arm.xyz.tolist(), "waypoint_error_m": waypoint_error,
                                               "action_summary": summary}, calls)
                    if not args.task and not args.pickup and error < 0.01 and stable >= 10 and summary["status"] == "reached":
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
    parser.add_argument("--test-axes", action="store_true", help="Test +/-3cm on each robot axis without Astra; record results and video")
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
