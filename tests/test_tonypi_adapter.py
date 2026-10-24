import sqlite3
import threading
import tomllib
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from bruhos.body.tonypi import TonyPiBody, check_actions, _claim_controller
from bruhos.brain import make_planner
from bruhos.runtime import Runtime, RuntimeConfig
from bruhos.schemas import LineReading, Observation, SkillCall, Status
from bruhos.skills.registry import SkillContext, run_skill
from bruhos.trace import TraceWriter, load_episode, verify_episode


def config(tmp_path):
    with sqlite3.connect(tmp_path/'wave.d6a') as db:
        db.execute('CREATE TABLE ActionGroup (duration INTEGER)')
        db.execute('INSERT INTO ActionGroup VALUES (100)')
    return {'body': {'action_path': str(tmp_path)}, 'actions': {'wave': 'wave'},
            'camera': {'enabled': False}, 'imu': {'enabled': True, 'driver': 'controller',
            'upright_reference_verified': True, 'orientation': {'upright_sign': -1}}}


def sdk(sample=(-1, 0, 0, 0, 0, 0), run=lambda *a, **kw: None):
    return SimpleNamespace(board=SimpleNamespace(enable_reception=lambda: None, get_imu=lambda: sample),
                           runActionGroup=run, stopAction=lambda: None, stopActionGroup=lambda: None)


def test_vendor_wave_generates_dashboard_episode_without_voice(tmp_path):
    cfg = config(tmp_path)
    calls = []
    with patch('bruhos.body.tonypi._load_agc', return_value=sdk(run=lambda *a, **kw: calls.append((a, kw)))):
        body = TonyPiBody(cfg)
        try:
            ep = Runtime(body, make_planner('rules'), config=RuntimeConfig(max_recoveries=0, max_replans=0)).run('Wave hello.')
        finally:
            body.close()
    assert ep.success
    assert calls == [(('wave',), {'path': str(tmp_path)+'/', 'times': 1, 'with_stand': True})]
    assert ep.steps[0].obs_before['sensors']['imu']['valid']
    TraceWriter(tmp_path/'traces').write(ep)
    assert verify_episode(load_episode(tmp_path/'traces'/ep.episode_id))
    assert check_actions(cfg) == {'wave': True}
    cfg['actions']['wave'] = 'missing'
    assert check_actions(cfg) == {'wave': False}
    assert not (tmp_path/'missing.d6a').exists()


@pytest.mark.parametrize('sample,verified', [(None, True), ((0, 0, 1, 0, 0, 0), True), ((-1, 0, 0, 0, 0, 0), False)])
def test_invalid_tilt_or_unverified_imu_never_moves(tmp_path, sample, verified):
    cfg = config(tmp_path);cfg['imu']['upright_reference_verified'] = verified
    calls = []
    with patch('bruhos.body.tonypi._load_agc', return_value=sdk(sample, run=lambda *a, **kw: calls.append(a))):
        body = TonyPiBody(cfg)
        try:
            assert not body.act('wave')
        finally:
            body.close()
    assert not calls


def test_active_ari_refused_before_hardware_import(tmp_path):
    with patch('bruhos.body.tonypi.subprocess.run', return_value=SimpleNamespace(stdout='active\n')):
        with pytest.raises(RuntimeError, match='tonypi-voice'):
            _claim_controller()


def test_pro_profile_preserves_brain_camera_actions_and_existing_runtime_defaults():
    cfg = tomllib.loads((Path(__file__).parents[1]/'configs/tonypi_pro.toml').read_text())
    assert cfg['brain'] == {'name': 'qwen_vl', 'base_url': 'http://192.168.1.50:8000/v1',
                            'model': 'Qwen/Qwen2.5-VL-7B-Instruct'}
    assert cfg['camera']['enabled']
    assert set(cfg['actions']) == {'stand', 'forward', 'turn_left', 'turn_right', 'grab',
        'release', 'kick_left', 'kick_right', 'wave', 'stand_up_front', 'stand_up_back'}
    assert 'runtime' not in cfg
    assert cfg['imu']['bus'] == 1 and cfg['imu']['address'] == '0x68'
    assert not cfg['imu']['upright_reference_verified']


def test_controller_lock_rejects_second_owner_and_can_be_released(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    with patch('bruhos.body.tonypi.subprocess.run', return_value=SimpleNamespace(stdout='inactive\n')):
        first = _claim_controller()
        try:
            with pytest.raises(RuntimeError, match='Another bruhOS'):
                _claim_controller()
        finally:
            first.close()
        _claim_controller().close()


def test_unrelated_missing_mapping_does_not_block_wave(tmp_path):
    cfg = config(tmp_path)
    cfg['body']['exclusive_control'] = True
    cfg['actions']['forward'] = 'missing'
    lock = SimpleNamespace(close=lambda: None)
    with patch('bruhos.body.tonypi._claim_controller', return_value=lock), patch(
        'bruhos.body.tonypi._load_agc', return_value=sdk()
    ):
        body = TonyPiBody(cfg)
        try:
            assert body.act('wave')
            assert not body.act('forward')
        finally:
            body.close()


def test_runtime_replan_cannot_replay_partial_sdk_action(tmp_path):
    calls = []
    def fail(*args, **kwargs):
        calls.append(args)
        raise TypeError('partially executed')
    with patch('bruhos.body.tonypi._load_agc', return_value=sdk(run=fail)):
        body = TonyPiBody(config(tmp_path))
        try:
            ep = Runtime(body, make_planner('rules')).run('Wave hello.')
        finally:
            body.close()
    assert not ep.success
    assert len(calls) == 1


def test_controller_recovery_preserves_alternate_standup_and_runtime_defaults(tmp_path):
    cfg = config(tmp_path)
    cfg['actions'].update(stand_up_front='missing', stand_up_back='wave')
    upright = False
    calls = []
    def run(*args, **kwargs):
        nonlocal upright
        calls.append(args)
        upright = True
    driver = sdk(run=run)
    driver.board.get_imu = lambda: (-1, 0, 0, 0, 0, 0) if upright else (0, 0, 1, 0, 0, 0)
    cfg['body']['exclusive_control'] = True
    lock = SimpleNamespace(close=lambda: None)
    with patch('bruhos.body.tonypi._claim_controller', return_value=lock), patch(
        'bruhos.body.tonypi._load_agc', return_value=driver
    ):
        body = TonyPiBody(cfg)
        try:
            ep = Runtime(body, make_planner('rules')).run('Wave hello.')
        finally:
            body.close()
    assert ep.success and ep.recoveries == 1
    assert len(calls) == 2  # alternate stand-up, then requested wave


def test_fall_after_wave_still_uses_existing_runtime_recovery(tmp_path):
    cfg = config(tmp_path)
    cfg['actions']['stand_up_front'] = 'wave'
    calls = []
    fallen = False
    def run(*args, **kwargs):
        nonlocal fallen
        calls.append(args)
        fallen = len(calls) == 1
    driver = sdk(run=run)
    driver.board.get_imu = lambda: (0, 0, 1, 0, 0, 0) if fallen else (-1, 0, 0, 0, 0, 0)
    with patch('bruhos.body.tonypi._load_agc', return_value=driver):
        body = TonyPiBody(cfg)
        try:
            ep = Runtime(body, make_planner('rules')).run('Wave hello.')
        finally:
            body.close()
    assert ep.success and ep.recoveries == 1
    assert [s.kind for s in ep.steps] == ['skill', 'recovery', 'skill']


class Body:
    turn_step_deg = 15
    def __init__(self, result=True):
        self.result = result
        self.calls = []
        self.stopped = threading.Event()
    def observe(self):
        return Observation(t=0, frame=1, line=LineReading(True, 1))
    def act(self, name):
        self.calls.append(name)
        return self.result
    def halt(self): self.stopped.set()


@pytest.mark.parametrize('call', [SkillCall('wave'), SkillCall('kick'),
    SkillCall('detect', {'target':'red'}), SkillCall('follow_line', {'steps':1})])
def test_failed_primitive_never_reports_success(call):
    body = Body(False)
    result = run_skill(SkillContext(body), call)
    assert result.status is Status.FAILED
    assert len(body.calls) == 1


def test_stop_never_commands_stand_and_invalidates_queued_action():
    body = TonyPiBody.__new__(TonyPiBody)
    body._lock = threading.Lock()
    body._halted = threading.Event()
    body._generation = 0
    body.controller_imu = False
    body._check_files = False
    calls = []
    body.agc = SimpleNamespace(stopAction=lambda:calls.append('stop'),
        stopActionGroup=lambda:calls.append('stop_group'), runActionGroup=lambda *a,**kw:calls.append('MOVE'))
    body.actions = {'wave':'wave', 'stand':'stand'}
    body.action_path = None
    body._lock.acquire()
    results = []
    attempted = threading.Event()
    # A lock wrapper gives deterministic proof the worker passed the first halt check.
    lock = body._lock
    class Lock:
        def __enter__(self): attempted.set(); lock.acquire()
        def __exit__(self,*args): lock.release()
    body._lock = Lock()
    worker = threading.Thread(target=lambda:results.append(body.act('wave')))
    worker.start()
    assert attempted.wait(1)
    body.halt()
    body.resume()
    lock.release()
    worker.join(1)
    assert results == [False]
    assert calls == ['stop','stop_group']
