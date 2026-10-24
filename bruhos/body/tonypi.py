"""TonyPi / TonyPi Pro body: Hiwonder action groups + camera + IMU.

Primitives map to action-group files (``.d6a``) shipped with the robot image. Names differ between
firmware images, so the mapping lives in ``configs/tonypi*.toml`` — check them against the
``ActionGroups/`` directory on your robot before the first run.
"""

from __future__ import annotations

import importlib
import os
import sqlite3
import subprocess
from pathlib import Path
import sys
import threading
import time
from typing import Any

from bruhos.body.base import PRIMITIVES, Body
from bruhos.schemas import Imu, LineReading, Observation

SDK_MODULES = ("hiwonder.ActionGroupControl", "HiwonderSDK.ActionGroupControl", "ActionGroupControl")


def _load_agc(sdk_path: str | None):
    if sdk_path and sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)
    errors = []
    for mod in SDK_MODULES:
        try:
            return importlib.import_module(mod)
        except ImportError as e:
            errors.append(f"{mod}: {e}")
    raise ImportError("Hiwonder ActionGroupControl not found; set [body].sdk_path. Tried: " + "; ".join(errors))


def check_actions(cfg):
    """Inspect configured vendor action files without importing hardware or moving."""
    root = Path(cfg.get('body', {}).get('action_path', '/home/pi/TonyPi/ActionGroups'))
    results = {}
    for primitive, group in cfg.get('actions', {}).items():
        if primitive not in PRIMITIVES or not isinstance(group, str) or not group.replace('_', '').isalnum():
            raise ValueError('Invalid action mapping')
        path = root / (group + '.d6a')
        try:
            with sqlite3.connect(path.resolve().as_uri() + '?mode=ro', uri=True) as db:
                ready = db.execute('SELECT count(*) FROM ActionGroup').fetchone()[0] > 0
        except sqlite3.Error:
            ready = False
        results[primitive] = ready
    return results


def _claim_controller():
    """Refuse known owners; never stop Ari or vendor services implicitly."""
    import fcntl
    for user, unit in ((True, 'tonypi-voice.service'), (False, 'tonypi.service'), (False, 'joystick.service')):
        command = ['systemctl'] + (['--user'] if user else [])
        state = subprocess.run(command + ['is-active', unit], capture_output=True, text=True, timeout=3)
        if state.stdout.strip() not in ('inactive', 'failed', 'unknown'):
            raise RuntimeError(f'Controller ownership unavailable: {unit}. Stop it explicitly before running bruhOS.')
    runtime = Path(os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}'))
    lock = (runtime / 'bruhos-tonypi.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        lock.close()
        raise RuntimeError('Another bruhOS process owns TonyPi') from None
    return lock


class TonyPiBody(Body):
    name = "tonypi"

    def __init__(self, cfg: dict[str, Any]):
        body = cfg.get("body", {})
        self.name = body.get("name", "tonypi")
        self.turn_step_deg = float(body.get("turn_step_deg", 15.0))
        self.actions: dict[str, str | None] = {p: None for p in PRIMITIVES} | cfg.get("actions", {})
        self._ownership = None
        self.cap = self.imu = None
        if body.get('exclusive_control', False):
            self._ownership = _claim_controller()
        try:
            self.agc = _load_agc(body.get("sdk_path"))
        except BaseException:
            self.close()
            raise
        self.action_path = body.get("action_path")
        self._check_files = body.get("exclusive_control", False)
        self._lock = threading.Lock()
        self._halted = threading.Event()
        self._generation = 0
        self.frame = 0
        self.holding: str | None = None
        self._last_seen: dict[str, Any] = {}
        self.controller_imu = cfg.get('imu', {}).get('driver') == 'controller'
        self.imu_verified = cfg.get('imu', {}).get('upright_reference_verified', False)
        self._imu_at = 0.0
        self._imu_value, self._imu_raw = Imu(), None
        try:
            self._setup_sensors(cfg)
        except BaseException:
            self.close()
            raise

    def _setup_sensors(self, cfg: dict[str, Any]) -> None:
        cam = cfg.get("camera", {})
        self.cap = self.detector = self.line = self.imu = None
        if cam.get("enabled", True):
            import cv2

            from .perception import CameraModel, ColorBlobDetector, LineDetector
            self.cap = cv2.VideoCapture(cam.get("device", 0))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam.get("width", 640))
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam.get("height", 480))
            model = CameraModel(cam.get("width", 640), cam.get("height", 480), cam.get("hfov_deg", 62.0))
            hsv = cfg.get("perception", {}).get("hsv")
            self.detector = ColorBlobDetector(model, **({"hsv": hsv} if hsv else {}))
            self.line = LineDetector()
        imu = cfg.get("imu", {})
        if imu.get("enabled", True):
            if self.controller_imu:
                from .controller_imu import ControllerIMU
                orientation = dict(imu.get('orientation', {}))
                orientation.setdefault('fall_deg', imu.get('fall_deg', 55.0))
                orientation.setdefault('forward_sign', imu.get('forward_sign', 1))
                self.imu = ControllerIMU(self.agc.board, **orientation)
            else:
                from .imu import MPU6050
                self.imu = MPU6050(imu.get("bus", 1), int(str(imu.get("address", "0x68")), 0),
                                   imu.get("fall_deg", 55.0), imu.get("forward_sign", 1))

    def _read_imu(self):
        if not self.controller_imu:
            return self.imu.read() if self.imu else Imu(), True
        deadline = time.monotonic() + 0.15
        while True:
            try:
                sample = self.imu.read()
            except (ValueError, OSError):
                self._imu_at = 0
                sample = None
                break
            if sample is not None:
                self._imu_value, self._imu_raw = sample
                self._imu_at = time.monotonic()
                break
            if time.monotonic() >= deadline:
                break
            time.sleep(0.01)
        return self._imu_value, bool(self._imu_at and time.monotonic() - self._imu_at < 0.5)

    def observe(self) -> Observation:
        self.frame += 1
        objects, line = [], LineReading()
        if self.cap is not None:
            ok, frame = self.cap.read()
            if ok:
                objects = self.detector.detect(frame)
                line = self.line.read(frame)
        imu, valid = self._read_imu()
        return Observation(t=round(time.monotonic(), 3), frame=self.frame, objects=objects,
                           imu=imu, line=line, holding=self.holding,
                           sensors={'imu': {'valid': valid, 'raw': self._imu_raw}} if self.controller_imu else {})

    def act(self, primitive: str) -> bool:
        recovery = primitive in ("stand_up_front", "stand_up_back")
        ticket = self._generation
        if self._halted.is_set():
            return False
        group = self.actions.get(primitive)
        if not group:
            return False
        if self._check_files and not check_actions({
            'body': {'action_path': self.action_path}, 'actions': {primitive: group}
        })[primitive]:
            return False
        with self._lock:
            if self._halted.is_set() or ticket != self._generation:
                return False
            if self.controller_imu:
                imu, valid = self._read_imu()
                if not self.imu_verified or not valid or (imu.fallen and not recovery):
                    return False
            kw = {"path": str(self.action_path).rstrip('/') + '/'} if self.action_path else {}
            if self.controller_imu:
                kw.update(times=1, with_stand=True)
            finished = threading.Event()
            def watch_stop():
                while not finished.wait(0.02):
                    if self._halted.is_set() or ticket != self._generation:
                        for name in ("stopAction", "stopActionGroup", "stop_action_group"):
                            stop = getattr(self.agc, name, None)
                            if callable(stop):
                                stop()
            watcher = threading.Thread(target=watch_stop, daemon=True)
            watcher.start()
            try:
                self.agc.runActionGroup(group, **kw)
            except Exception:
                # Latch stop: runtime replanning must not replay a partial SDK call.
                self.halt()
                return False
            finally:
                finished.set()
                watcher.join(1)
        if self._halted.is_set() or ticket != self._generation:
            return False
        if primitive == "grab":
            self.holding = "object"
        elif primitive == "release":
            self.holding = None
        return True

    def halt(self) -> None:
        self._generation += 1
        self._halted.set()
        for name in ("stopAction", "stopActionGroup", "stop_action_group"):
            stop = getattr(self.agc, name, None)
            if callable(stop):
                stop()

    def resume(self) -> None:
        self._halted.clear()

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()
        if self.imu and hasattr(self.imu, 'close'):
            self.imu.close()
        if self._ownership is not None:
            self._ownership.close()
            self._ownership = None
