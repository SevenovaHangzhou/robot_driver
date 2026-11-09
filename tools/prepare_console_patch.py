"""Generate a reviewable UI-only console patch; never install a motion backend."""
import argparse
import difflib
import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PREFIX = 'src/rt_control/rt_control_arm_console/'


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError('Console source changed; inspect before preparing integration')
    return text.replace(old, new)


def prepare(repo):
    base = repo / PREFIX
    changes = {}
    baselines = {}
    for relative in ('rt_control_arm_console/server.py', 'web/index.html', 'CMakeLists.txt'):
        baselines[relative] = subprocess.check_output(
            ['git', 'show', 'HEAD:' + PREFIX + relative], cwd=repo).decode()
    manifest_path = ROOT / 'integration/source-manifest.json'
    previous = {entry['path']: entry for entry in json.loads(manifest_path.read_text())} if manifest_path.exists() else {}
    server = baselines['rt_control_arm_console/server.py']
    server = replace_once(server, '        elif path.startswith("/static/"):', '''        elif path in ("/jog", "/jog/"):
            self._file(self.server.web_dir / "jog/jog.html")
        elif path == "/api/jog/state":
            if self._session() is None:
                self._json(HTTPStatus.UNAUTHORIZED, {"error": "login_required"})
                return
            self._json(HTTPStatus.OK, {
                "mode": "unavailable", "hardware_connected": False,
                "lease": "none", "armed": False, "epoch": "", "busy": False,
                "can_arm": False, "result": "末端点动实机后端尚未开放",
                "blockers": ["待核验 TCP、参考系和运行模型身份",
                             "待接入 Motion 笛卡尔路径与全模型碰撞检查",
                             "待完成统一执行仲裁及故障停车契约"],
            })
        elif path.startswith("/static/"):''')
    server = replace_once(server, '        if path == "/api/acquire":', '''        if path.startswith("/api/jog/"):
            raise ConsoleError("jog_unavailable", "末端点动实机后端尚未开放")
        if path == "/api/acquire":''')
    changes['rt_control_arm_console/server.py'] = server
    changes['CMakeLists.txt'] = replace_once(baselines['CMakeLists.txt'],
        '  ament_add_pytest_test(server test/test_server.py TIMEOUT 60)',
        '  ament_add_pytest_test(server test/test_server.py TIMEOUT 60)\n'
        '  ament_add_pytest_test(jog_ui test/test_jog_ui.py TIMEOUT 60)')
    index = baselines['web/index.html']
    changes['web/index.html'] = replace_once(index, '    <div class="chips">',
        '    <div class="chips">\n      <a class="chip" href="/jog">末端点动 · 待核验</a>')
    for filename in ('jog.html', 'jog.css', 'jog.mjs', 'input.mjs', 'model-view.mjs'):
        changes['web/jog/' + filename] = (ROOT / 'web' / filename).read_text()
    changes['test/test_jog_ui.py'] = '''"""UI-only integration must never submit any arm trajectory."""
import json
from test_server import server, request, login  # noqa: F401
from test_console import setup  # noqa: F401


def test_jog_page_and_authentication(server):
    instance, _, _ = server
    response, body = request(instance, "GET", "/jog")
    assert response.status == 200 and "末端点动" in body.decode()
    response, _ = request(instance, "GET", "/static/jog/input.mjs")
    assert response.status == 200
    response, _ = request(instance, "GET", "/api/jog/state")
    assert response.status == 401
    cookie = login(instance)
    response, body = request(instance, "GET", "/api/jog/state", cookie=cookie)
    state = json.loads(body)
    assert response.status == 200 and state["mode"] == "unavailable"
    assert state["armed"] is False and state["can_arm"] is False


def test_jog_commands_fail_closed(server):
    instance, console, driver = server
    cookie = login(instance)
    initial = list(driver.sent)
    for command in ("acquire", "arm", "step", "heartbeat", "stop"):
        response, body = request(instance, "POST", f"/api/jog/{command}", {}, cookie)
        assert response.status == 409 and json.loads(body)["error"] == "jog_unavailable"
    assert driver.sent == initial
'''
    patch, manifest = [], []
    for relative, updated in changes.items():
        path = base / relative
        original = baselines.get(relative, '')
        current = path.read_text() if path.exists() else ''
        previous_entry = previous.get(PREFIX + relative, {})
        before_hash = hashlib.sha256(original.encode()).hexdigest() if original else None
        if previous_entry and previous_entry.get('before_sha256') != before_hash:
            raise RuntimeError('Source baseline changed; review and rebase integration')
        known = {hashlib.sha256(original.encode()).hexdigest(), hashlib.sha256(updated.encode()).hexdigest(),
                 previous_entry.get('after_sha256')}
        if hashlib.sha256(current.encode()).hexdigest() not in known:
            raise RuntimeError(f'Unrelated local changes in {path}; preserve them before integrating')
        name = PREFIX + relative
        patch.extend(difflib.unified_diff(original.splitlines(True), updated.splitlines(True),
                     fromfile='a/' + name if original else '/dev/null', tofile='b/' + name))
        manifest.append({'path': name, 'before_sha256': hashlib.sha256(original.encode()).hexdigest() if original else None,
                         'after_sha256': hashlib.sha256(updated.encode()).hexdigest()})
    (ROOT / 'integration/console-ui.patch').write_text(''.join(patch))
    (ROOT / 'integration/source-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(f'Prepared {len(changes)} files; no files in the target repository were edited.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', required=True, type=Path)
    prepare(parser.parse_args().repo)
