"""Loopback-only HTTP console; motion backends remain local simulations."""
import argparse
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
from pathlib import Path
import secrets
import sys
import threading
import time
from urllib.parse import parse_qs, urlsplit, unquote

from .control import MockJog
from .errors import JogError
from .model import Chain
from .model_assets import ModelAssets
from .planner import Limits, Planner
from .rolling_mock import RollingExecutor

ROOT = Path(__file__).resolve().parent.parent
if not (ROOT / "web/jog.html").is_file():
    ROOT = Path(sys.prefix) / "share/gripper-jog"
HEADERS = {
    "Cache-Control": "no-store", "X-Content-Type-Options": "nosniff",
    "Content-Security-Policy": "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'",
    "Referrer-Policy": "no-referrer", "X-Frame-Options": "DENY",
}


class JogHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, jog, web_dir=ROOT / "web", model_assets=None, *, login_token):
        super().__init__(address, Handler)
        self.login_token = login_token
        self.jog, self.web_dir = jog, Path(web_dir)
        self.model_assets = model_assets
        self.sessions, self.sessions_lock = set(), threading.Lock()
        self.stopping = threading.Event()
        self.worker = threading.Thread(target=self._tick, daemon=True)
        self.worker.start()

    def _tick(self):
        while not self.stopping.wait(.02):
            self.jog.tick()

    def server_close(self):
        self.stopping.set()
        self.worker.join(timeout=3)
        # Exercise the same controlled-stop path on an orderly Mock shutdown.
        self.jog.dispatch('stop', 'server-shutdown', {})
        for _ in range(100):
            self.jog.tick()
            if self.jog.active is None:
                break
            time.sleep(.02)
        close = getattr(self.jog, 'close', None)
        if close:
            close()
        super().server_close()


class MockServer(JogHTTPServer):
    def __init__(self, address, jog, web_dir=ROOT / 'web', model_assets=None):
        if address[0] != '127.0.0.1':
            raise ValueError('Mock server only binds IPv4 loopback')
        super().__init__(address, jog, web_dir, model_assets, login_token='mock')


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *_args):
        pass

    def send(self, status, payload, content_type="application/json; charset=utf-8", extra=None):
        raw = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode() if isinstance(payload, dict) else payload
        self.send_response(status)
        for k, v in {**HEADERS, "Content-Type": content_type, "Content-Length": str(len(raw)), **(extra or {})}.items():
            self.send_header(k, v)
        self.end_headers()
        try:
            self.wfile.write(raw)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def session(self):
        try:
            cookie = SimpleCookie(self.headers.get("Cookie", ""))
            session = cookie["jog_mock_session"].value
        except (KeyError, ValueError):
            return None
        with self.server.sessions_lock:
            return session if session in self.server.sessions else None

    def do_GET(self):
        url = urlsplit(self.path)
        if url.path in ("/", "/jog", "/jog/"):
            path = self.server.web_dir / "jog.html"
        elif url.path.startswith("/static/jog/"):
            name = url.path.removeprefix("/static/jog/")
            if name not in {"jog.css", "jog.mjs", "input.mjs", "model-view.mjs", "gateway.mjs", "diagnostics.mjs"}:
                self.send(404, {"error": "not_found"})
                return
            path = self.server.web_dir / name
        elif url.path.startswith("/static/vendor/"):
            name = url.path.removeprefix("/static/vendor/")
            if name not in {"three.module.min.js", "OrbitControls.js", "STLLoader.js", "ColladaLoader.js", "TGALoader.js", "URDFLoader.js", "URDFClasses.js"}:
                self.send(404, {"error": "not_found"})
                return
            path = self.server.web_dir / "vendor" / name
        elif url.path == "/api/jog/model" or url.path.startswith("/mesh/"):
            if self.session() is None:
                self.send(401, {"error": "login_required"})
                return
            assets = self.server.model_assets
            if assets is None:
                self.send(404, {"error": "model_unavailable"})
                return
            if url.path == "/api/jog/model":
                self.send(200, assets.info(self.server.jog.planner.chain))
                return
            path = assets.meshes.get(unquote(url.path.removeprefix("/mesh/")))
            if path is None:
                self.send(404, {"error": "not_found"})
                return
        elif url.path == '/api/jog/config':
            self.send(200, {'authority_gateway': False,
                            'readonly_login': self.server.jog.mode == 'real-observe',
                            'preview_token': self.server.login_token == 'mock'})
            return
        elif url.path == "/api/jog/state":
            session = self.session()
            if session is None:
                self.send(401, {"error": "login_required", "message": "请登录"})
                return
            client = parse_qs(url.query).get("client", [""])[0]
            self.send(200, self.server.jog.state(session, client))
            return
        else:
            self.send(404, {"error": "not_found"})
            return
        self.send(200, path.read_bytes(), mimetypes.guess_type(path.name)[0] or "text/javascript")

    def do_POST(self):
        origin = self.headers.get("Origin")
        host = self.headers.get("Host")
        if origin is not None and origin != "http://" + str(host):
            self.send(403, {"error": "cross_origin", "message": "拒绝跨源请求"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 4096:
                self.close_connection = True
                raise ValueError("body size")
            body = json.loads(self.rfile.read(length), parse_constant=lambda _: None)
            if not isinstance(body, dict):
                raise ValueError("object required")
        except (ValueError, TypeError):
            self.send(400, {"error": "bad_request", "message": "请求格式无效"})
            return
        path = urlsplit(self.path).path
        if path == "/api/login":
            token = body.get('token')
            if not isinstance(token, str) or not secrets.compare_digest(token.encode(), self.server.login_token.encode()):
                self.send(401, {"error": "auth_failed", "message": "访问令牌无效"})
                return
            session = secrets.token_urlsafe(24)
            with self.server.sessions_lock:
                if len(self.server.sessions) >= 128:
                    self.send(429, {"error": "session_limit", "message": "Mock 会话已满，请重启预览服务"})
                    return
                self.server.sessions.add(session)
            self.send(200, {"ok": True}, extra={"Set-Cookie": f"jog_mock_session={session}; HttpOnly; SameSite=Strict; Path=/"})
            return
        session = self.session()
        if session is None:
            self.send(401, {"error": "login_required", "message": "请登录"})
            return
        if not path.startswith("/api/jog/"):
            self.send(404, {"error": "not_found", "message": "未知操作"})
            return
        try:
            result = self.server.jog.dispatch(path.removeprefix("/api/jog/"), session, body)
            self.send(200, {"ok": True, **result})
        except JogError as exc:
            self.send(409, {"error": exc.code, "message": str(exc)})
        except (ValueError, TypeError, KeyError):
            self.send(400, {"error": "bad_request", "message": "请求参数无效"})


def build_mock(urdf, config):
    if config.get("mode") != "mock":
        raise JogError("hardware_disabled", "当前版本仅提供 Mock，不能启用实机后端")
    xml = Path(urdf).read_text()
    arms = {}
    for name, arm_config in config.get('arms', {'single': {}}).items():
        settings = {**config, **arm_config}
        chain = Chain(xml, settings['base_frame'], settings['tip_frame'],
                      settings['tcp_xyz_m'], settings['tcp_rpy_rad'])
        arms[name] = (Planner(chain, Limits(tcp_speed=settings['tcp_speed_mm_s'] / 1000)), settings['initial_q_rad'])
    selected = config.get('selected_arm', next(iter(arms)))
    if selected not in arms:
        raise JogError('config_invalid', '默认机械臂不在已配置列表中')
    planner, initial_q = arms[selected]
    publication = config.get('publication_mode', 'step')
    if publication not in ('step', 'rolling'):
        raise JogError('config_invalid', '未知轨迹发布模式')
    return MockJog(planner, initial_q, config['heartbeat_timeout_s'], config['lease_timeout_s'],
                   arms=arms, selected_arm=selected,
                   execution=RollingExecutor() if publication == 'rolling' else None,
                   direction_frame=config.get('direction_frame'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('mink-rolling', 'mink', 'mock'), default='mink-rolling', help='默认 Mink → Rolling 连续会话；mink 为直接仿真，mock 为原分段演示')
    parser.add_argument('--real2sim', type=Path, default=ROOT.parents[1] / 'sim/real2sim_v3', help='demo_mink.sh 所在目录')
    parser.add_argument('--gravity-mode', choices=('static', 'mujoco', 'off'), default='off',
                        help='仅 Mink：off 关闭补偿（默认），static 调用静态重力库，mujoco 使用原 bias 前馈')
    parser.add_argument('--gravity-source', type=Path,
                        help='仅 Mink：static_gravity 源码目录，默认同工作区 sim/Dynamic_Parameter_Identification/static_gravity')
    parser.add_argument('--viewer', action='store_true', help='同时打开本地 MuJoCo 窗口，仅适用于 Mink')
    parser.add_argument('--headless', action='store_true', help='只打开网页，不启动 MuJoCo 窗口')
    parser.add_argument("--urdf", type=Path, help="仅 Mock：权威 URDF，默认同工作区 sim/robot_description 模型")
    parser.add_argument("--config", type=Path, default=ROOT / "config/mock.json", help="仅 Mock：规划配置")
    parser.add_argument("--model-share", type=Path, help="仅 Mock：与 --urdf 同版本的模型 package 根目录")
    parser.add_argument("--port", type=int, default=18131)
    args = parser.parse_args()
    mink_backend = args.backend in ('mink', 'mink-rolling')
    if args.viewer and not mink_backend:
        parser.error('--viewer 仅适用于 Mink 后端')
    if args.viewer and not args.headless and not os.environ.get('DISPLAY'):
        parser.error('MuJoCo 原生窗口需要桌面 DISPLAY；请在桌面终端运行，或追加 --headless 使用网页三维视图')
    viewer_enabled = args.viewer and not args.headless
    if mink_backend:
        if args.urdf or args.model_share or args.config != ROOT / 'config/mock.json':
            parser.error('Mink 使用 --real2sim 的配套模型和配置；URDF/config 覆盖选项仅适用于 --backend mock')
        from .mink_sim import build_mink, check_viewer_display
        try:
            if viewer_enabled:
                check_viewer_display()
            jog, assets = build_mink(args.real2sim, autostart=not viewer_enabled, rolling=args.backend == 'mink-rolling',
                                     gravity_mode=args.gravity_mode, gravity_source=args.gravity_source)
        except (JogError, ValueError, OSError) as exc:
            parser.error(str(exc))
    elif args.urdf is None:
        share = args.model_share or ROOT.parents[1] / 'sim/robot_description'
        args.urdf = share / 'urdf/robot_alfav3_mobile.urdf'
        if not args.urdf.is_file():
            parser.error('未找到 robot_description/main 模型，请用 --model-share 指定该仓库根目录')
    if args.backend == 'mock':
        config = json.loads(args.config.read_text())
        jog, assets = build_mock(args.urdf, config), ModelAssets(args.urdf, args.model_share)
    try:
        server = MockServer(("127.0.0.1", args.port), jog, model_assets=assets)
    except OSError:
        if hasattr(jog, 'close'):
            jog.close()
        raise
    label = {'mink-rolling': 'Mink → Rolling + MuJoCo / 本地协议模拟器', 'mink': 'Mink + MuJoCo / 无 ROS', 'mock': 'Mock Rolling'}[args.backend]
    print(f"{label}: http://127.0.0.1:{server.server_port}/jog  (login token: mock)", flush=True)
    try:
        if viewer_enabled:
            from .mink_sim import run_viewer
            run_viewer(server)
        else:
            server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
