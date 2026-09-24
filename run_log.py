"""Local run artifacts: readable transcript, API traces, and three-view video."""
import base64
from datetime import datetime
from html import escape
import json
from pathlib import Path
import re
import threading
import time

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw


class RunLog:
    def __init__(self, args):
        task = re.sub(r"[^A-Za-z0-9_-]", "_", args.task or args.env)
        stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
        self.path = Path(__file__).resolve().parent / "outputs" / f"{task}_{stamp}"
        self.path.mkdir(parents=True)
        self.events = []
        self.lock = threading.Lock()
        self.writer = None
        self.frames = 0
        self.started = time.monotonic()
        self.metadata = {"started": datetime.now().astimezone().isoformat(),
                         "settings": vars(args), "status": "running"}
        self.event("Run started", self.metadata)
        print(f"Run outputs: {self.path}", flush=True)

    def event(self, kind, data, call=None):
        with self.lock:
            event = {"kind": kind, "call": call, "wall_seconds": round(time.monotonic() - self.started, 3),
                     "video_seconds": max(0, self.frames - 1) / 20, "data": data}
            self.events.append(event)
            with (self.path / "traces.jsonl").open("a") as file:
                file.write(json.dumps(event, ensure_ascii=False) + "\n")
            self._html()

    def request(self, kwargs, call):
        # Persist exact JPEG bytes sent to the API, referenced by local path.
        saved = {**kwargs, "text_format": kwargs["text_format"].model_json_schema()}
        saved = json.loads(json.dumps(saved))
        for message in saved["input"]:
            if not isinstance(message["content"], list):
                continue
            image_number = 0
            for item in message["content"]:
                if item["type"] == "input_image":
                    image_number += 1
                    name = f"call_{call:03d}_camera_{image_number}.jpg"
                    (self.path / name).write_bytes(base64.b64decode(item["image_url"].split(",", 1)[1]))
                    item["image_url"] = name
        self.event("Astra input", saved, call)

    def snapshot(self, images, call):
        names = []
        for number, pixels in enumerate(images, 1):
            name = f"call_{call:03d}_view_{number}.jpg"
            Image.fromarray(pixels).save(self.path / name, quality=90)
            names.append(name)
        self.event("Camera views (recording only unless vision is enabled)", {"images": names}, call)

    def frame(self, images):
        canvas = Image.fromarray(np.concatenate(images, axis=1))
        draw = ImageDraw.Draw(canvas)
        width = images[0].shape[1]
        for number, title in enumerate(("Left view", "Right view", "Wrist view")):
            draw.rectangle((number * width, 0, number * width + 115, 24), fill="black")
            draw.text((number * width + 8, 6), title, fill="white")
        if self.writer is None:
            self.writer = imageio.get_writer(self.path / "video.mp4", fps=20,
                                             codec="libx264", pixelformat="yuv420p")
        self.writer.append_data(np.asarray(canvas))
        self.frames += 1

    def close(self, status):
        try:
            if self.writer is not None:
                self.writer.close()
        finally:
            self.metadata["status"] = status
            self.metadata["video_frames"] = self.frames
            self.metadata["reason"] = self._outcome()[1]
            (self.path / "run.json").write_text(json.dumps(self.metadata, indent=2))
            self.event("Run finished", {"status": status, "reason": self.metadata["reason"]})

    def _outcome(self):
        status = self.metadata.get("status", "running")
        if status == "running":
            return "Running", "The run is still in progress."
        if status == "viewer closed":
            return "Interrupted", "The simulation viewer was closed."
        if status == "interrupted":
            return "Interrupted", "The run was interrupted before it finished."
        if status in ("failed", "error"):
            error = next((e["data"] for e in reversed(self.events)
                          if e["kind"] == "Run error"), {})
            reason = error.get("message") or "The run ended with an error."
            label = "Failed" if status == "failed" or "Action budget exhausted" in reason else "Error"
            return label, reason
        success = next((e["data"] for e in reversed(self.events)
                        if e["kind"] == "Success"), None)
        if success is not None:
            if success.get("official_success"):
                return "Success", "RoboCasa's task success check passed."
            if "object_lift_m" in success:
                return "Success", f"Object grasped and lifted {success['object_lift_m']:.3f} m."
            if "reaching_error_m" in success:
                return "Success", f"Reaching goal achieved; error {success['reaching_error_m']:.4f} m."
            return "Success", "The goal was achieved."
        if any(e["kind"] == "Inspection" for e in self.events):
            return "Inspection complete", "The scene was inspected without running an action."
        return "Completed", "The run ended normally."

    def _html(self):
        def pretty(value):
            return escape(json.dumps(value, indent=2, ensure_ascii=False))

        system_prompt = None
        objective = None
        calls = {}
        for event in self.events:
            if event["kind"] == "Objective":
                objective = event["data"].get("instruction")
            if event["kind"] == "Astra input" and system_prompt is None:
                system_prompt = next((m["content"] for m in event["data"].get("input", [])
                                      if m.get("role") == "system"), None)
            if event["call"] is not None:
                calls.setdefault(event["call"], {})[event["kind"]] = event

        steps = []
        for number, events in sorted(calls.items()):
            request = events.get("Astra input")
            mock = events.get("Mock input")
            if request is None and mock is None:
                continue
            source = request or mock
            body = [f'<article><h2>Step {number}</h2>',
                    f'<p class="muted">Wall {source["wall_seconds"]:.2f}s · '
                    f'<button onclick="seek({source["video_seconds"]})">Video {source["video_seconds"]:.2f}s</button></p>',
                    '<h3>Input to Astra</h3>' if request else '<h3>Input to mock policy</h3>']
            if request:
                observation = None
                images = []
                camera_name = "Camera"
                for message in request["data"].get("input", []):
                    if message.get("role") != "user":
                        continue
                    for item in message.get("content", []):
                        if item.get("type") == "input_text":
                            value = item["text"]
                            if value.startswith("Camera:"):
                                camera_name = value.removeprefix("Camera:").strip()
                            else:
                                try:
                                    observation = json.loads(value)
                                except ValueError:
                                    observation = value
                        elif item.get("type") == "input_image":
                            images.append((camera_name, item["image_url"]))
                if images:
                    body.append('<div class="views">' + ''.join(
                        f'<figure><img src="{escape(path, quote=True)}" alt="{escape(name, quote=True)}">'
                        f'<figcaption>{escape(name)}</figcaption></figure>' for name, path in images
                    ) + '</div>')
                if isinstance(observation, dict):
                    body.append(f'<p><strong>Goal:</strong> {escape(str(observation.get("instruction", "")))}</p>')
                    feedback = observation.get("last_action_result")
                    if feedback:
                        body.append(f'<p><strong>Previous result:</strong> {escape(str(feedback))}</p>')
                body.append(f'<pre class="observation">{pretty(observation)}</pre>')
            else:
                observation = mock["data"].get("observation", {})
                body.append(f'<p><strong>Goal:</strong> {escape(str(observation.get("instruction", "")))}</p>')
                body.append(f'<pre class="observation">{pretty(observation)}</pre>')

            response = events.get("Astra output") or events.get("Mock output")
            body.append('<h3>Full model output</h3>' if request else '<h3>Mock output</h3>')
            if response:
                data = response["data"]
                body.append(f'<pre>{pretty(data.get("action"))}</pre>')
                if data.get("latency_seconds") is not None:
                    body.append(f'<p class="muted">API duration: {data["latency_seconds"]:.2f}s</p>')
            else:
                body.append('<p class="muted">No response was recorded.</p>')
            result = events.get("Action result") or events.get("Action rejected")
            if result:
                feedback = result["data"].get("feedback") or result["data"].get("reason")
                body.append(f'<p><strong>Execution result:</strong> {escape(str(feedback))}</p>')
            body.append('</article>')
            steps.append(''.join(body))

        label, reason = self._outcome()
        prompt = system_prompt or ("No Astra system prompt was used: this was a mock run."
                                   if self.metadata.get("settings", {}).get("mock") else
                                   "No Astra request was made in this run.")
        document = '''<!doctype html><html><head><meta charset="utf-8"><title>Astra robotics run</title>
<style>body{background:#12161e;color:#e6edf3;font:16px system-ui;max-width:1150px;margin:32px auto;padding:0 20px}
article,.summary{background:#1c2330;padding:20px;margin:18px 0;border-radius:10px}h2{font-size:19px}h3{font-size:16px;margin:22px 0 8px}
pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:13px;line-height:1.5}.observation{max-height:360px;overflow:auto;background:#121923;padding:14px;border-radius:6px}
video{width:100%;background:black}.views{display:flex;gap:8px}.views figure{margin:0;flex:1;min-width:0}.views img{width:100%}figcaption{font-size:13px;color:#aab8ca;overflow-wrap:anywhere}
button{cursor:pointer}.muted{color:#aab8ca}a{color:#91caff}</style></head><body>'''
        document += f'<h1>{escape(self.path.name)}</h1>'
        document += '<p><a href="traces.jsonl">JSON traces</a> · <a href="run.json">Run settings</a></p>'
        document += f'<section class="summary"><h2>Final status: {escape(label)}</h2><p>{escape(reason)}</p></section>'
        if objective:
            document += f'<p><strong>Goal:</strong> {escape(objective)}</p>'
        document += f'<section class="summary"><h2>System prompt</h2><pre>{escape(prompt)}</pre></section>'
        document += '<p class="muted">Left / right / wrist · 20 fps, simulation time; API waits omitted.</p>'
        document += '<video id="video" controls preload="metadata" src="video.mp4"></video>'
        document += ''.join(steps) if steps else '<p>No model steps were recorded.</p>'
        document += '<script>function seek(t){const v=document.getElementById("video");v.currentTime=t;v.scrollIntoView({behavior:"smooth"});}</script></body></html>'
        temporary = self.path / "transcript.tmp"
        temporary.write_text(document)
        temporary.replace(self.path / "transcript.html")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Rebuild readable transcripts from saved JSON traces")
    parser.add_argument("--rebuild", action="store_true", required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent / "outputs"
    for trace in root.glob("*/traces.jsonl"):
        run = RunLog.__new__(RunLog)
        run.path = trace.parent
        run.events = [json.loads(line) for line in trace.read_text().splitlines()]
        settings = trace.parent / "run.json"
        run.metadata = json.loads(settings.read_text()) if settings.exists() else {"status": "running"}
        run._html()
        print(run.path / "transcript.html")
