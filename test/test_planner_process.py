import queue
import threading

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
        else:
            self.responses.put({'type': 'ready', 'generation': generation})

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

    def Queue(self):
        value = FakeQueue()
        self.queues.append(value)
        return value

    def Process(self, target, args, name, daemon):
        process = FakeProcess(*args, next(self.behaviors))
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
    planner.shutdown()


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
    assert responses.get(timeout=1) == {'type': 'ready', 'generation': 7}
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
