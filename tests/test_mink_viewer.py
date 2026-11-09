"""Viewer failures must not hang startup or race the physics worker."""
from contextlib import contextmanager
import sys
from types import SimpleNamespace

import pytest

from gripper_jog.errors import JogError
from gripper_jog.mink_sim import check_viewer_display, run_viewer


@pytest.mark.parametrize('initialized,monitors', [(False, []), (True, [])])
def test_no_display_output_rejected_before_creating_window(monkeypatch, initialized, monitors):
    monkeypatch.delenv('DISPLAY', raising=False)
    terminated = []
    monkeypatch.setitem(sys.modules, 'glfw', SimpleNamespace(
        init=lambda: initialized, get_monitors=lambda: monitors,
        terminate=lambda: terminated.append(True)))
    with pytest.raises(JogError, match='显示输出'):
        check_viewer_display()
    assert terminated == [True]


def test_xwayland_phantom_monitor_does_not_hang_startup(monkeypatch):
    monkeypatch.setenv('DISPLAY', ':test')
    monkeypatch.setattr('gripper_jog.mink_sim.shutil.which', lambda _: '/usr/bin/xrandr')
    monkeypatch.setattr('gripper_jog.mink_sim.subprocess.run',
                        lambda *a, **kw: SimpleNamespace(returncode=0, stdout='Monitors: 0\n'))
    with pytest.raises(JogError, match='显示输出'):
        check_viewer_display()


def test_viewer_launch_failure_leaves_physics_and_http_unstarted():
    def unexpected_start():
        pytest.fail('Startup continued after the viewer failed')

    @contextmanager
    def failed_viewer(model, data):
        raise RuntimeError('Viewer initialization failed')
        yield  # Mark this as a context manager without starting a window.

    jog = SimpleNamespace(backend=SimpleNamespace(model=object(), data=object()),
                          managed_viewer=failed_viewer, start=unexpected_start)
    server = SimpleNamespace(jog=jog, serve_forever=unexpected_start, shutdown=unexpected_start)
    with pytest.raises(RuntimeError, match='Viewer initialization failed'):
        run_viewer(server)
