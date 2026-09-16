# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Thin access to a running Gazebo world over gz-transport, through the `gz` CLI.

The CLI is used rather than Python bindings because it always matches the
simulator's gz-transport version (bindings for the installed version are not
packaged for this Python). Messages are exchanged as JSON (subscriptions) and
protobuf text format (service requests).
"""

from __future__ import annotations

import json
import subprocess
import threading
import time
from dataclasses import dataclass, field

WORLD = "chessbot"


class GzError(RuntimeError):
    pass


def service(name: str, reqtype: str, reptype: str, req: str, timeout_ms: int = 5000) -> str:
    """Call a gz service; returns the text-format reply."""
    out = subprocess.run(
        ["gz", "service", "-s", name, "--reqtype", reqtype, "--reptype", reptype, "--timeout", str(timeout_ms), "--req", req],
        capture_output=True,
        text=True,
        timeout=timeout_ms / 1000 + 5,
    )
    if out.returncode != 0 or "timed out" in out.stdout.lower():
        raise GzError(f"{name}: {out.stdout.strip()} {out.stderr.strip()}")
    return out.stdout


def topic_once(topic: str, timeout_s: float = 5.0) -> dict:
    out = subprocess.run(
        ["gz", "topic", "-e", "-t", topic, "-n", "1", "--json-output"],
        capture_output=True,
        text=True,
        timeout=timeout_s,
    )
    if not out.stdout.strip():
        raise GzError(f"no message on {topic}")
    return json.loads(out.stdout.strip().splitlines()[0])


def topics(pattern: str = "") -> list[str]:
    out = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True, timeout=10)
    return [t for t in out.stdout.split() if pattern in t]


@dataclass
class Recorder:
    """Records every message on some topics, as parsed JSON, while in a `with` block."""

    topics: list[str]
    messages: list[dict] = field(default_factory=list)

    def __enter__(self):
        self._lock = threading.Lock()
        self._procs = []
        self._threads = []
        for topic in self.topics:
            proc = subprocess.Popen(
                ["gz", "topic", "-e", "-t", topic, "--json-output"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
            )
            thread = threading.Thread(target=self._read, args=(proc,), daemon=True)
            thread.start()
            self._procs.append(proc)
            self._threads.append(thread)
        time.sleep(1.0)  # let the subscriptions connect
        return self

    def _read(self, proc):
        for line in proc.stdout:
            line = line.strip()
            if line:
                try:
                    message = json.loads(line)
                except json.JSONDecodeError:
                    continue
                with self._lock:
                    self.messages.append(message)

    def __exit__(self, *exc):
        time.sleep(1.0)  # let the last messages arrive
        for proc in self._procs:
            proc.terminate()
        for proc, thread in zip(self._procs, self._threads):
            proc.wait(timeout=5)
            thread.join(timeout=5)


def sim_time(message: dict) -> float:
    stamp = message.get("header", {}).get("stamp", {})
    return float(stamp.get("sec", 0)) + float(stamp.get("nsec", 0)) / 1e9


def model_poses(world: str = WORLD) -> dict[str, tuple[tuple[float, float, float], tuple[float, float, float, float]]]:
    """Pose of every entity by name: ((x, y, z), (x, y, z, w))."""
    msg = topic_once(f"/world/{world}/pose/info")
    poses = {}
    for p in msg.get("pose", []):
        pos = p.get("position", {})
        ori = p.get("orientation", {})
        poses[p["name"]] = (
            (pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0)),
            (ori.get("x", 0.0), ori.get("y", 0.0), ori.get("z", 0.0), ori.get("w", 1.0)),
        )
    return poses


def spawn(models: list[tuple[str, str, tuple[float, float, float], float]], world: str = WORLD):
    """Spawn (name, sdf_file, xyz, yaw) models in one call."""
    import math

    entries = []
    for name, sdf_file, (x, y, z), yaw in models:
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        entries.append(
            f'data {{ sdf_filename: "{sdf_file}" name: "{name}" allow_renaming: false '
            f"pose {{ position {{ x: {x} y: {y} z: {z} }} orientation {{ z: {qz} w: {qw} }} }} }}"
        )
    service(f"/world/{world}/create_multiple/blocking", "gz.msgs.EntityFactory_V", "gz.msgs.Boolean", " ".join(entries), 20000)


def remove(name: str, world: str = WORLD):
    service(f"/world/{world}/remove/blocking", "gz.msgs.Entity", "gz.msgs.Boolean", f'name: "{name}" type: MODEL')
