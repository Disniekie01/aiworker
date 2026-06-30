#!/usr/bin/env python3
"""Web launcher for pick-place Isaac Sim workflow.

Buttons to start/stop Isaac stack, YAML animation, and joint pose web tuner.
Each service includes an in-page usage guide.

  /usr/bin/python3.12 scripts/pick_place_launcher.py

Open http://127.0.0.1:8760
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

SCRIPT_ROOT = Path(__file__).resolve().parents[1]
STATIC_DIR = Path(__file__).resolve().parent / "static"
RUN_STACK = SCRIPT_ROOT / "scripts" / "run_stack.sh"
LOG_DIR = SCRIPT_ROOT / ".run_logs"
PID_DIR = SCRIPT_ROOT / ".run_pids"
DEFAULT_CONFIG = SCRIPT_ROOT / "config" / "pick_place.yaml"
PYTHON = "/usr/bin/python3.12"
TUNER_PORT = 8765

_lock = threading.Lock()
_yaml_proc: subprocess.Popen | None = None
_tuner_proc: subprocess.Popen | None = None


def _resolve_usd_path(explicit: str | None) -> str:
    if explicit:
        p = Path(explicit).expanduser().resolve()
        if p.is_file():
            return str(p)
        raise FileNotFoundError(f"USD not found: {p}")
    candidates = [
        SCRIPT_ROOT / "scenes" / "newscene" / "newscene.usda",
    ]
    for c in candidates:
        if c.is_file():
            return str(c.resolve())
    raise FileNotFoundError(
        "No scene USD found. Install scenes/newscene assets or pass --usd-path."
    )


def _ros_env(domain_id: int) -> str:
    return (
        f"source /opt/ros/jazzy/setup.bash && "
        f'source "{SCRIPT_ROOT}/ros2_ws/install/setup.bash" && '
        f'source "{SCRIPT_ROOT}/env_local.bash" && '
        f"export ROS_DOMAIN_ID={domain_id}"
    )


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def _read_pid_file(name: str) -> int | None:
    pidf = PID_DIR / f"{name}.pid"
    if not pidf.is_file():
        return None
    try:
        pid = int(pidf.read_text().strip())
    except ValueError:
        return None
    return pid if _pid_alive(pid) else None


def _stack_status() -> dict[str, Any]:
    try:
        out = subprocess.check_output(
            [str(RUN_STACK), "status"],
            cwd=SCRIPT_ROOT,
            text=True,
            timeout=10,
        )
    except (subprocess.SubprocessError, OSError) as exc:
        return {"running": False, "error": str(exc), "udp_bridge": False, "isaac": False}

    udp = any(line.strip().startswith("[up]") and "udp_bridge" in line for line in out.splitlines())
    isaac = any(line.strip().startswith("[up]") and "isaac" in line for line in out.splitlines())
    return {
        "running": udp and isaac,
        "udp_bridge": udp,
        "isaac": isaac,
        "raw": out.strip(),
    }


def _yaml_status() -> dict[str, Any]:
    global _yaml_proc
    running = _yaml_proc is not None and _yaml_proc.poll() is None
    if not running:
        _yaml_proc = None
    return {"running": running, "pid": _yaml_proc.pid if running and _yaml_proc else None}


def _tuner_status() -> dict[str, Any]:
    global _tuner_proc
    running = _tuner_proc is not None and _tuner_proc.poll() is None
    if not running:
        # Also detect tuner started outside launcher
        try:
            out = subprocess.check_output(
                ["pgrep", "-f", "joint_pose_web_tuner.py"],
                text=True,
                timeout=5,
            ).strip()
            if out:
                pid = int(out.splitlines()[0])
                if _pid_alive(pid):
                    return {"running": True, "pid": pid, "external": True}
        except (subprocess.SubprocessError, ValueError, OSError):
            pass
        _tuner_proc = None
    return {
        "running": running,
        "pid": _tuner_proc.pid if running and _tuner_proc else None,
        "url": f"http://127.0.0.1:{TUNER_PORT}",
        "external": False,
    }


def _tail_log(name: str, lines: int = 40) -> str:
    path = LOG_DIR / f"{name}.log"
    if not path.is_file():
        return ""
    try:
        content = path.read_text(errors="replace").splitlines()
        return "\n".join(content[-lines:])
    except OSError as exc:
        return f"(read error: {exc})"


def _start_stack(usd_path: str, domain_id: int) -> str:
    cmd = [
        str(RUN_STACK),
        "start",
        "--pick-place",
        "--with-isaac",
        "--physics-grasp",
        "--usd-path",
        usd_path,
        "--domain-id",
        str(domain_id),
    ]
    out = subprocess.check_output(cmd, cwd=SCRIPT_ROOT, text=True, timeout=120)
    return out.strip()


def _stop_stack() -> str:
    out = subprocess.check_output(
        [str(RUN_STACK), "stop"], cwd=SCRIPT_ROOT, text=True, timeout=60
    )
    return out.strip()


def _spawn_logged(cmd: str, log_name: str) -> subprocess.Popen:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / f"{log_name}.log"
    log_path.write_text(f"--- started {time.strftime('%Y-%m-%d %H:%M:%S')} ---\n")
    log_f = open(log_path, "a", encoding="utf-8")
    return subprocess.Popen(
        ["bash", "-lc", cmd],
        cwd=SCRIPT_ROOT,
        stdout=log_f,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _silence_tuner_publish() -> bool:
    """Turn off tuner Live publish so YAML owns /ffw_isaac/joint_targets."""
    try:
        req = urllib.request.Request(
            f"http://127.0.0.1:{TUNER_PORT}/api/publish/disable",
            data=b"{}",
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        urllib.request.urlopen(req, timeout=3)
        return True
    except (urllib.error.URLError, OSError):
        return False


def _start_yaml(config: Path, domain_id: int) -> str:
    global _yaml_proc
    if _yaml_proc is not None and _yaml_proc.poll() is None:
        raise RuntimeError("YAML animation already running")
    silenced = _silence_tuner_publish()
    cmd = (
        f"{_ros_env(domain_id)} && "
        f'{PYTHON} "{SCRIPT_ROOT}/scripts/publish_pose_from_yaml.py" '
        f'--config "{config}"'
    )
    _yaml_proc = _spawn_logged(cmd, "yaml_anim")
    if silenced:
        return "YAML animation started (tuner live publish disabled)"
    return "YAML animation started"


def _stop_yaml() -> None:
    global _yaml_proc
    if _yaml_proc is not None and _yaml_proc.poll() is None:
        os.killpg(_yaml_proc.pid, signal.SIGTERM)
        try:
            _yaml_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(_yaml_proc.pid, signal.SIGKILL)
    _yaml_proc = None
    subprocess.run(
        ["pkill", "-f", "[p]ublish_pose_from_yaml.py"],
        check=False,
        timeout=5,
    )


def _start_tuner(config: Path, domain_id: int) -> None:
    global _tuner_proc
    if _yaml_proc is not None and _yaml_proc.poll() is None:
        raise RuntimeError(
            "YAML animation is playing — wait for it to finish before using the tuner"
        )
    if _tuner_proc is not None and _tuner_proc.poll() is None:
        raise RuntimeError("Web tuner already running")
    st = _tuner_status()
    if st.get("running"):
        raise RuntimeError("Web tuner already running (started outside launcher)")
    cmd = (
        f"{_ros_env(domain_id)} && "
        f'{PYTHON} "{SCRIPT_ROOT}/scripts/joint_pose_web_tuner.py" '
        f'--config "{config}" --physics-grasp --port {TUNER_PORT}'
    )
    _tuner_proc = _spawn_logged(cmd, "web_tuner")


def _stop_tuner() -> None:
    global _tuner_proc
    if _tuner_proc is not None and _tuner_proc.poll() is None:
        os.killpg(_tuner_proc.pid, signal.SIGTERM)
        try:
            _tuner_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(_tuner_proc.pid, signal.SIGKILL)
    _tuner_proc = None
    subprocess.run(
        ["pkill", "-f", "[j]oint_pose_web_tuner.py"],
        check=False,
        timeout=5,
    )


def make_handler(
    usd_path: str,
    config_path: Path,
    domain_id: int,
) -> type[BaseHTTPRequestHandler]:
    static_html = STATIC_DIR / "pick_place_launcher.html"

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: Any) -> None:
            return

        def _json(self, code: int, payload: dict[str, Any]) -> None:
            body = json.dumps(payload).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _read_json(self) -> dict[str, Any]:
            length = int(self.headers.get("Content-Length", 0))
            if length <= 0:
                return {}
            raw = self.rfile.read(length)
            try:
                return json.loads(raw.decode("utf-8"))
            except json.JSONDecodeError:
                return {}

        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                data = static_html.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return

            if path == "/api/status":
                with _lock:
                    self._json(
                        200,
                        {
                            "isaac_stack": _stack_status(),
                            "yaml": _yaml_status(),
                            "tuner": _tuner_status(),
                            "usd_path": usd_path,
                            "config": str(config_path),
                            "tuner_port": TUNER_PORT,
                        },
                    )
                return

            if path.startswith("/api/logs/"):
                name = path.split("/")[-1]
                allowed = {"isaac", "udp_bridge", "yaml_anim", "web_tuner", "isaac_stack"}
                if name not in allowed:
                    self._json(404, {"error": "unknown log"})
                    return
                if name == "isaac_stack":
                    udp = _tail_log("udp_bridge", 15)
                    isaac = _tail_log("isaac", 25)
                    combined = ""
                    if udp:
                        combined += "=== udp_bridge ===\n" + udp
                    if isaac:
                        combined += ("\n\n" if combined else "") + "=== isaac ===\n" + isaac
                    self._json(200, {"log": combined or "(no log yet)"})
                    return
                self._json(200, {"log": _tail_log(name)})
                return

            self._json(404, {"error": "not found"})

        def do_POST(self) -> None:
            path = urlparse(self.path).path
            try:
                with _lock:
                    if path == "/api/isaac/start":
                        msg = _start_stack(usd_path, domain_id)
                        self._json(200, {"ok": True, "message": msg})
                        return
                    if path == "/api/isaac/stop":
                        msg = _stop_stack()
                        self._json(200, {"ok": True, "message": msg})
                        return
                    if path == "/api/yaml/start":
                        msg = _start_yaml(config_path, domain_id)
                        self._json(200, {"ok": True, "message": msg})
                        return
                    if path == "/api/yaml/stop":
                        _stop_yaml()
                        self._json(200, {"ok": True, "message": "YAML animation stopped"})
                        return
                    if path == "/api/tuner/start":
                        _start_tuner(config_path, domain_id)
                        self._json(
                            200,
                            {
                                "ok": True,
                                "message": f"Web tuner started at http://127.0.0.1:{TUNER_PORT}",
                                "url": f"http://127.0.0.1:{TUNER_PORT}",
                            },
                        )
                        return
                    if path == "/api/tuner/stop":
                        _stop_tuner()
                        self._json(200, {"ok": True, "message": "Web tuner stopped"})
                        return
                self._json(404, {"error": "not found"})
            except Exception as exc:
                self._json(500, {"ok": False, "error": str(exc)})

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description="Pick-place workflow web launcher")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8760)
    parser.add_argument("--usd-path", default="", help="Scene USD (default: auto-detect)")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--domain-id", type=int, default=0)
    args = parser.parse_args()

    usd_path = _resolve_usd_path(args.usd_path or None)
    config_path = Path(args.config).resolve()
    if not config_path.is_file():
        raise SystemExit(f"Config not found: {config_path}")
    if not RUN_STACK.is_file():
        raise SystemExit(f"run_stack.sh not found: {RUN_STACK}")

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    PID_DIR.mkdir(parents=True, exist_ok=True)

    handler = make_handler(usd_path, config_path, args.domain_id)
    server = ThreadingHTTPServer((args.host, args.port), handler)
    url = f"http://{args.host}:{args.port}"
    print(f"Pick-place launcher: {url}")
    print(f"  USD: {usd_path}")
    print(f"  Config: {config_path}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down launcher (services keep running).")


if __name__ == "__main__":
    main()
