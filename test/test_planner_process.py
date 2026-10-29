import queue
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from h12_ros2_controller.core.planner import planner_process
from h12_ros2_controller.core.planner.planner_process import PlannerClient
from h12_ros2_controller.core.planner.reduced_joint_planner import PlanResult, PlannerConfig


class FakeQueue:
    def __init__(self):
        self.items = queue.Queue()
        self.on_put = None
        self.closed = False

    def put(self, value):
        self.items.put(value)
        if self.on_put is not None:
            self.on_put(value)

    def get(self, timeout=None):
        return self.items.get(timeout=timeout)

    def cancel_join_thread(self):
        pass

    def close(self):
        self.closed = True


class FakeProcess:
    def __init__(self, init_args, requests, responses, behavior):
        self.init_args = init_args
        self.requests = requests
        self.responses = responses
        self.behavior = behavior
        self.alive = False
        self.terminated = False
        self.pending = None

    def start(self):
        self.alive = True
        generation = self.init_args['generation']
        if self.behavior == 'startup_error':
            self.responses.put({'type': 'startup_error', 'error': 'bad model',
                                'generation': generation})
        elif self.behavior == 'startup_dead':
            self.alive = False
        elif self.behavior != 'startup_hung':
            self.responses.put({'type': 'ready', 'generation': generation,
                                'ompl_seed': self.init_args.get('ompl_seed')})

        def respond(request):
            if request['command'] == 'shutdown':
                self.alive = False
                return
            if self.behavior == 'dead':
                self.alive = False
                return
            message = {
                'type': 'result',
                'result': PlanResult(True, np.zeros((1, 1)), reason=self.behavior),
                'generation': generation,
                'request_id': request['request_id'],
            }
            if self.behavior == 'delayed':
                self.pending = message
            elif self.behavior == 'wrong_id':
                self.responses.put({**message, 'request_id': -1})
            elif self.behavior != 'hung':
                self.responses.put(message)

        self.requests.on_put = respond
        self.started.set()

    def is_alive(self):
        return self.alive

    def join(self, timeout=None):
        pass

    def terminate(self):
        self.terminated = True
        self.alive = False

    def kill(self):
        self.terminate()


class FakeContext:
    def __init__(self, behaviors):
        self.behaviors = iter(behaviors)
        self.queues = []
        self.processes = []
        self.started = threading.Event()

    def Queue(self):
        value = FakeQueue()
        self.queues.append(value)
        return value

    def Process(self, target, args, name, daemon):
        process = FakeProcess(*args, next(self.behaviors))
        process.started = self.started
        self.processes.append(process)
        return process


def client(behaviors):
    value = PlannerClient('robot', 'spheres', 'srdf', PlannerConfig(),
                          request_timeout=0.1, ompl_seed=42)
    value._ctx = FakeContext(behaviors)
    return value


def test_hung_worker_times_out_and_next_request_restarts():
    planner = client(['hung', 'recovered'])
    with pytest.raises(TimeoutError, match='response timed out'):
        planner._request({'command': 'plan_configuration'})
    first = planner._ctx.processes[0]
    assert first.terminated
    assert all(item.closed for item in planner._ctx.queues[:2])
    result = planner._request({'command': 'plan_configuration'})
    assert result.reason == 'recovered'
    assert result.metadata['client_timings']['total'] >= 0.0
    assert planner._ctx.processes[1].init_args['ompl_seed'] == 42
    assert planner.ompl_seed_effective == 42
    planner.shutdown()
    assert planner.ompl_seed_effective is None


def test_response_timeout_override_is_shorter_than_configured_timeout():
    planner = client(['hung'])
    planner.request_timeout = 1.0
    started = time.perf_counter()
    with pytest.raises(TimeoutError, match='response timed out'):
        planner.plan_configuration(np.zeros(1), np.zeros(1), np.zeros(1),
                                   response_timeout=0.025)
    assert time.perf_counter() - started < 0.5
    assert planner._ctx.processes[0].terminated


@pytest.mark.parametrize('timeout', [0, -1, float('inf'), float('nan')])
def test_invalid_response_timeout_does_not_start_worker(timeout):
    planner = client(['ready'])
    with pytest.raises(ValueError, match='response_timeout'):
        planner.plan_configuration(np.zeros(1), np.zeros(1), np.zeros(1),
                                   response_timeout=timeout)
    assert not planner._ctx.processes


def test_late_response_from_timed_out_worker_cannot_poison_next_request():
    planner = client(['delayed', 'new worker'])
    with pytest.raises(TimeoutError):
        planner._request({'command': 'plan_configuration'})
    old = planner._ctx.processes[0]
    old.responses.put(old.pending)
    assert planner._request({'command': 'plan_configuration'}).reason == 'new worker'
    assert old.init_args['generation'] != planner._ctx.processes[1].init_args['generation']
    assert old.responses is not planner._ctx.processes[1].responses
    planner.shutdown()


def test_dead_worker_recovers_on_next_request():
    planner = client(['dead', 'recovered'])
    with pytest.raises(RuntimeError, match='exited unexpectedly'):
        planner._request({'command': 'plan_configuration'})
    assert planner._request({'command': 'plan_configuration'}).reason == 'recovered'
    planner.shutdown()


def test_wrong_request_id_is_ignored_until_deadline():
    planner = client(['wrong_id'])
    with pytest.raises(TimeoutError):
        planner._request({'command': 'plan_configuration'})
    assert planner._ctx.processes[0].terminated


def test_startup_error_cleans_up_and_warmup_can_retry():
    planner = client(['startup_error', 'ready'])
    with pytest.raises(RuntimeError, match='bad model'):
        planner.warmup()
    assert planner._ctx.processes[0].terminated
    assert all(item.closed for item in planner._ctx.queues[:2])
    assert planner.warmup() >= 0.0
    assert planner.warmup() >= 0.0
    assert len(planner._ctx.processes) == 2
    planner.shutdown()


def test_concurrent_requests_are_serialized():
    planner = client(['hung', 'recovered'])
    results = []

    def request():
        try:
            results.append(planner._request({'command': 'plan_configuration'}).reason)
        except TimeoutError:
            results.append('timeout')

    first = threading.Thread(target=request)
    second = threading.Thread(target=request)
    first.start()
    second.start()
    first.join(timeout=2)
    second.join(timeout=2)
    assert not first.is_alive() and not second.is_alive()
    assert sorted(results) == ['recovered', 'timeout']
    planner.shutdown()


def test_background_warmup_is_nonblocking_and_idempotent():
    planner = client(['startup_hung'])
    assert planner.startup_duration is None
    assert planner.startup_error is None
    assert not planner.ready
    started = time.perf_counter()
    assert planner.warmup_background()
    assert time.perf_counter() - started < 0.5
    assert planner._ctx.started.wait(timeout=1)
    assert planner._warmup_thread.is_alive()
    assert not planner.ready
    assert not planner.warmup_background()
    process = planner._ctx.processes[0]
    process.responses.put({'type': 'ready', 'generation': planner._generation})
    planner._warmup_thread.join(timeout=1)
    assert not planner._warmup_thread.is_alive()
    assert planner.ready
    assert planner.startup_duration >= 0.0
    assert not planner.warmup_background()
    duration = planner.startup_duration
    assert planner.warmup() >= 0.0
    assert planner.startup_duration == duration
    assert len(planner._ctx.processes) == 1
    planner.shutdown()
    assert not planner.ready


def test_plan_waits_for_background_startup_without_spawning_again():
    planner = client(['startup_hung'])
    planner.warmup_background()
    assert planner._ctx.started.wait(timeout=1)
    results = queue.Queue()
    entered = threading.Event()

    def request():
        entered.set()
        try:
            results.put(planner._request({'command': 'plan_configuration'}))
        except Exception as exc:
            results.put(exc)

    thread = threading.Thread(target=request)
    thread.start()
    assert entered.wait(timeout=1)
    with pytest.raises(queue.Empty):
        results.get(timeout=0.025)
    process = planner._ctx.processes[0]
    assert process.requests.items.empty()
    process.responses.put({'type': 'ready', 'generation': planner._generation})
    result = results.get(timeout=1)
    thread.join(timeout=1)
    planner._warmup_thread.join(timeout=1)
    assert isinstance(result, PlanResult)
    assert len(planner._ctx.processes) == 1
    planner.shutdown()


def test_concurrent_background_warmup_calls_schedule_one_attempt():
    planner = client(['startup_hung'])
    barrier = threading.Barrier(6)
    scheduled = queue.Queue()

    def warmup():
        barrier.wait(timeout=1)
        scheduled.put(planner.warmup_background())

    threads = [threading.Thread(target=warmup) for _ in range(6)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=1)
        assert not thread.is_alive()
    assert sum(scheduled.get(timeout=1) for _ in threads) == 1
    assert planner._ctx.started.wait(timeout=1)
    assert len(planner._ctx.processes) == 1
    planner.shutdown()
    planner._warmup_thread.join(timeout=1)


@pytest.mark.parametrize('behavior, error', [
    ('startup_error', 'bad model'),
    ('startup_hung', 'did not start in time'),
    ('startup_dead', 'exited during startup'),
])
def test_background_startup_errors_are_observable_and_retryable(
        monkeypatch, behavior, error):
    unhandled = []
    monkeypatch.setattr(threading, 'excepthook', unhandled.append)
    planner = client([behavior, 'recovered'])
    planner.startup_timeout = 0.025
    assert planner.warmup_background()
    planner._warmup_thread.join(timeout=1)
    assert not planner._warmup_thread.is_alive()
    assert not unhandled
    assert not planner.ready
    assert isinstance(planner.startup_error, RuntimeError)
    assert error in str(planner.startup_error)
    assert planner.startup_duration >= 0.0
    assert all(item.closed for item in planner._ctx.queues)
    assert not planner.warmup_background()
    assert planner.warmup() >= 0.0
    assert planner.ready
    assert planner.startup_error is None
    planner.shutdown()


def test_shutdown_interrupts_background_startup_and_prevents_restart():
    planner = client(['startup_hung'])
    planner.warmup_background()
    assert planner._ctx.started.wait(timeout=1)
    process = planner._ctx.processes[0]
    started = time.perf_counter()
    planner.shutdown()
    assert time.perf_counter() - started < 1.0
    planner._warmup_thread.join(timeout=1)
    assert not planner._warmup_thread.is_alive()
    assert process.terminated
    assert not planner.ready
    assert planner._process is None
    assert all(item.closed for item in planner._ctx.queues)
    process.responses.put({'type': 'ready', 'generation': planner._generation})
    for operation in (planner.warmup_background, planner.warmup,
                      lambda: planner._request({'command': 'plan_configuration'})):
        with pytest.raises(RuntimeError, match='shut down'):
            operation()
    assert len(planner._ctx.processes) == 1
    planner.shutdown()


def test_shutdown_before_warmup_thread_runs_prevents_late_spawn(monkeypatch):
    planner = client(['ready'])
    release = threading.Event()
    warmup = planner._warmup_background

    def delayed_warmup():
        assert release.wait(timeout=1)
        warmup()

    monkeypatch.setattr(planner, '_warmup_background', delayed_warmup)
    planner.warmup_background()
    try:
        planner.shutdown()
    finally:
        release.set()
        planner._warmup_thread.join(timeout=1)
    assert not planner._warmup_thread.is_alive()
    assert not planner._ctx.processes
    assert not planner.ready


def test_ready_message_after_stop_signal_does_not_mark_worker_ready(monkeypatch):
    planner = client(['ready'])
    get = FakeQueue.get

    def get_and_stop(self, timeout=None):
        message = get(self, timeout=timeout)
        if message.get('type') == 'ready':
            planner._shutdown.set()
        return message

    monkeypatch.setattr(FakeQueue, 'get', get_and_stop)
    planner.warmup_background()
    planner._warmup_thread.join(timeout=1)
    assert not planner._warmup_thread.is_alive()
    assert not planner.ready
    assert planner._process is None
    assert planner._ctx.processes[0].terminated
    assert all(item.closed for item in planner._ctx.queues)
    assert 'shut down' in str(planner.startup_error)
    planner.shutdown()


def test_worker_messages_include_generation_and_request_id(monkeypatch):
    class StubWorker:
        def __init__(self, init_args):
            pass

        def handle(self, request):
            if request['request_id'] == 2:
                raise ValueError('failed')
            return PlanResult(True, np.zeros((1, 1)))

    monkeypatch.setattr(planner_process, '_PlannerWorker', StubWorker)
    requests = queue.Queue()
    responses = queue.Queue()
    thread = threading.Thread(
        target=planner_process._planner_worker_main,
        args=({'generation': 7, 'ompl_seed': None}, requests, responses),
    )
    thread.start()
    assert responses.get(timeout=1) == {
        'type': 'ready', 'generation': 7, 'ompl_seed': None,
    }
    requests.put({'command': 'plan_configuration', 'request_id': 1})
    result = responses.get(timeout=1)
    assert result['type'] == 'result'
    assert result['generation'] == 7 and result['request_id'] == 1
    assert isinstance(result['result'], PlanResult)
    requests.put({'command': 'plan_configuration', 'request_id': 2})
    error = responses.get(timeout=1)
    assert error['type'] == 'error'
    assert error['generation'] == 7 and error['request_id'] == 2
    assert 'failed' in error['error']
    requests.put({'command': 'shutdown'})
    thread.join(timeout=1)
    assert not thread.is_alive()


@pytest.mark.parametrize('timeout', [0, -1, float('inf'), float('nan')])
def test_invalid_request_timeout(timeout):
    with pytest.raises(ValueError, match='request_timeout'):
        PlannerClient('robot', 'spheres', 'srdf', PlannerConfig(),
                      request_timeout=timeout)


@pytest.fixture
def clock(monkeypatch):
    value = [0.0]
    monkeypatch.setattr(planner_process, 'perf_counter', lambda: value[0])

    def get(self, timeout=None):
        try:
            return self.items.get_nowait()
        except queue.Empty:
            value[0] += timeout
            raise

    monkeypatch.setattr(FakeQueue, 'get', get)
    return value


@pytest.mark.parametrize('deadline', [float('inf'), float('nan')])
def test_invalid_absolute_deadline_does_not_spawn(deadline):
    planner = client(['ready'])
    with pytest.raises(ValueError, match='deadline must be finite'):
        planner._request({'command': 'plan_configuration'}, deadline=deadline)
    assert not planner._ctx.processes


def test_expired_absolute_deadline_does_not_spawn(clock):
    planner = client(['ready'])
    with pytest.raises(TimeoutError, match='deadline exceeded'):
        planner._request({'command': 'plan_configuration'}, deadline=0.0)
    assert not planner._ctx.processes


def test_absolute_deadline_bounds_lock_wait_without_stopping_active_worker(clock):
    planner = client(['ready'])
    planner.warmup()
    lock = planner._lock
    waits = []

    def acquire(timeout):
        waits.append(timeout)
        clock[0] += timeout
        return False

    planner._lock = SimpleNamespace(acquire=acquire)
    try:
        with pytest.raises(TimeoutError, match='waiting for worker lock'):
            planner._request({'command': 'plan_configuration'}, deadline=0.25)
        assert waits == [0.25]
        assert planner.ready
        assert not planner._ctx.processes[0].terminated
        assert planner._ctx.processes[0].requests.items.empty()
    finally:
        planner._lock = lock
        planner.shutdown()


def test_absolute_deadline_stops_cold_start_before_dispatch_and_recovers(clock, monkeypatch):
    planner = client(['ready', 'recovered'])
    start = FakeProcess.start

    def slow_start(self):
        start(self)
        clock[0] += 0.6

    monkeypatch.setattr(FakeProcess, 'start', slow_start)
    with pytest.raises(TimeoutError, match='deadline exceeded during startup'):
        planner._request({'command': 'plan_configuration'}, deadline=0.5)
    first = planner._ctx.processes[0]
    assert first.requests.items.empty()
    assert first.terminated
    result = planner._request({'command': 'plan_configuration'}, deadline=2.0)
    assert result.reason == 'recovered'
    assert planner._generation == 2
    planner.shutdown()


def test_absolute_deadline_caps_startup_polling(clock):
    planner = client(['startup_hung'])
    with pytest.raises(TimeoutError, match='deadline exceeded during startup'):
        planner._request({'command': 'plan_configuration'}, deadline=0.25)
    assert clock[0] == pytest.approx(0.25)
    assert planner._ctx.processes[0].terminated


def test_legacy_response_cap_still_excludes_startup(clock, monkeypatch):
    planner = client(['ready'])
    start = FakeProcess.start

    def slow_start(self):
        start(self)
        clock[0] += 2.0

    monkeypatch.setattr(FakeProcess, 'start', slow_start)
    result = planner._request({'command': 'plan_configuration'}, response_timeout=0.1)
    assert result.success
    assert result.metadata['client_timings']['startup'] == 2.0
    planner.shutdown()


def test_absolute_deadline_caps_response_wait_after_startup(clock, monkeypatch):
    planner = client(['hung'])
    planner.request_timeout = 5.0
    start = FakeProcess.start

    def slow_start(self):
        start(self)
        clock[0] += 0.2

    monkeypatch.setattr(FakeProcess, 'start', slow_start)
    with pytest.raises(TimeoutError, match='response timed out'):
        planner._request({'command': 'plan_configuration'}, deadline=0.3)
    assert clock[0] == pytest.approx(0.3)
    assert planner._ctx.processes[0].terminated


@pytest.mark.parametrize('phase', ['dispatch', 'response'])
def test_absolute_deadline_rejects_late_dispatch_or_response(clock, monkeypatch, phase):
    planner = client(['ready'])
    planner.warmup()
    if phase == 'dispatch':
        respond = planner._request_queue.on_put

        def slow_dispatch(request):
            clock[0] += 0.3
            respond(request)

        planner._request_queue.on_put = slow_dispatch
    else:
        get = FakeQueue.get

        def slow_response(self, timeout=None):
            message = get(self, timeout=timeout)
            clock[0] += 0.3
            return message

        monkeypatch.setattr(FakeQueue, 'get', slow_response)
    with pytest.raises(TimeoutError, match='response timed out'):
        planner._request({'command': 'plan_configuration'}, deadline=0.25)
    process = planner._ctx.processes[0]
    assert process.requests.items.get_nowait()['deadline'] == 0.25
    assert process.terminated


@pytest.mark.parametrize('sync_time, success', [(0.2, True), (0.3, False)])
def test_worker_model_sync_consumes_relative_and_absolute_budget(clock, sync_time, success):
    worker = object.__new__(planner_process._PlannerWorker)
    calls = []

    def sync(q):
        clock[0] += sync_time

    def plan(*args, **kwargs):
        calls.append(kwargs)
        return PlanResult(True, np.zeros((2, 1)))

    worker._set_current_q = sync
    worker.planner = SimpleNamespace(config=PlannerConfig(), plan=plan)
    result = worker._plan_configuration({
        'current_q': np.zeros(1), 'start': np.zeros(1), 'goal': np.zeros(1),
        'planning_budget': 0.25, 'deadline': 1.0,
    })
    assert result.success is success
    if success:
        assert calls[0]['deadline'] == 0.25
    else:
        assert not calls
        assert result.path.shape == (0, 1)
        assert 'during model sync' in result.reason
