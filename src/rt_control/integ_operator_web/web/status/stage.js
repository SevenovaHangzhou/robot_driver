// Existing 3D view of the whole robot (unchanged rendering; the canvas is transparent).
import * as THREE from "./vendor/three.module.min.js";
import { OrbitControls } from "./vendor/OrbitControls.js";
import URDFLoader from "./vendor/URDFLoader.js";

// ------------------------------------------------------------------ 3D stage
export class Stage {
  constructor(host) {
    this.host = host;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    host.appendChild(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(30, 1, 0.01, 50);
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.enablePan = false;
    this.scene.add(new THREE.HemisphereLight(0xcfd8e3, 0x0b0d10, 1.6));
    const key = new THREE.DirectionalLight(0xffffff, 1.8);
    key.position.set(2.5, -2.5, 4);
    this.scene.add(key);
    const rim = new THREE.DirectionalLight(0x7cc4ff, 1.4);
    rim.position.set(-3, 3, 1.5);
    this.scene.add(rim);
    this.scene.add(floorRings());
    new ResizeObserver(() => this.resize()).observe(host);
    this.resize();
    const loop = () => { this.controls.update(); this.renderer.render(this.scene, this.camera); requestAnimationFrame(loop); };
    loop();
  }
  resize() {
    const { clientWidth: w, clientHeight: h } = this.host;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    if (this.robot) this.frame();
  }
  load(url, meshPrefix) {
    return new Promise((resolve, reject) => {
      const manager = new THREE.LoadingManager();
      const loader = new URDFLoader(manager);
      loader.packages = { robot_description: meshPrefix };
      loader.onError = reject;
      this.body = new THREE.MeshStandardMaterial({ color: 0x7a8696, metalness: 0.4, roughness: 0.45 });
      this.dark = new THREE.MeshStandardMaterial({ color: 0x1a1e24, metalness: 0.2, roughness: 0.7 });
      loader.load(url, (robot) => {
        if (!robot) { reject(new Error("model parse failed")); return; }
        this.robot = robot;
        manager.onLoad = () => {
          robot.traverse((c) => {
            if (!c.isMesh) return;
            const link = c.parent?.parent?.urdfName || "";
            c.material = /wheel|steer/.test(link) ? this.dark : this.body;
          });
          this.frame();
          resolve();
        };
        this.scene.add(robot);
      });
    });
  }
  frame() {
    const sphere = new THREE.Box3().setFromObject(this.robot).getBoundingSphere(new THREE.Sphere());
    const v = THREE.MathUtils.degToRad(this.camera.fov);
    const hz = 2 * Math.atan(Math.tan(v / 2) * this.camera.aspect);
    const dist = (sphere.radius / Math.sin(Math.min(v, hz) / 2)) * 0.78;
    const dir = new THREE.Vector3(1, -0.35, 0.3).normalize();
    this.controls.target.copy(sphere.center);
    this.camera.position.copy(sphere.center).addScaledVector(dir, dist);
    this.camera.near = dist / 100;
    this.camera.far = dist * 20;
    this.camera.updateProjectionMatrix();
  }
  set(values) {
    if (!this.robot) return;
    for (const [n, v] of Object.entries(values)) this.robot.joints[n]?.setJointValue(v);
  }
}

function floorRings() {
  const g = new THREE.Group();
  const ring = (r, color, opacity) => {
    const pts = [];
    for (let i = 0; i <= 160; i++) { const a = (i / 160) * Math.PI * 2; pts.push(new THREE.Vector3(Math.cos(a) * r, Math.sin(a) * r, 0.002)); }
    g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color, transparent: true, opacity })));
  };
  ring(0.55, 0x7cc4ff, 0.9);
  ring(0.85, 0x2b3340, 1);
  ring(1.3, 0x2b3340, 1);
  ring(1.9, 0x2b3340, 1);
  const cross = [new THREE.Vector3(-1.9, 0, 0.002), new THREE.Vector3(1.9, 0, 0.002), new THREE.Vector3(0, -1.9, 0.002), new THREE.Vector3(0, 1.9, 0.002)];
  g.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(cross), new THREE.LineBasicMaterial({ color: 0x2b3340 })));
  return g;
}

