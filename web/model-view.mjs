import * as THREE from '../vendor/three.module.min.js';
import {OrbitControls} from '../vendor/OrbitControls.js';
import URDFLoader from '../vendor/URDFLoader.js';

export class ModelView {
  constructor(container, status) {
    this.status = status;
    this.renderer = new THREE.WebGLRenderer({antialias: true, alpha: true});
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.setClearColor(0x111b1e, 1);
    container.appendChild(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(38, 1, .001, 30);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(2, -2, 1.6);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.scene.add(new THREE.HemisphereLight(0xf4fff8, 0x3b474b, 2.4));
    const sun = new THREE.DirectionalLight(0xffffff, 2);
    sun.position.set(3, -4, 5); this.scene.add(sun);
    const grid = new THREE.GridHelper(4, 40, 0x45665c, 0x253a3d);
    grid.rotation.x = Math.PI / 2; this.scene.add(grid);
    this.scene.add(new THREE.AxesHelper(.2));
    this.robot = null; this.frame = null; this.lastState = null;
    this.points = []; this.count = 0;
    this.trails = new Map(); this.currentIdentity = null; this.activeLinks = new Set();
    this.lineGeometry = new THREE.BufferGeometry();
    this.traceBuffer = new Float32Array(4000 * 3);
    this.lineGeometry.setAttribute('position', new THREE.BufferAttribute(this.traceBuffer, 3));
    this.lineGeometry.setDrawRange(0, 0);
    this.trace = new THREE.Line(this.lineGeometry,
      new THREE.LineBasicMaterial({color: 0x62d9ff, depthTest: false, transparent: true, opacity: .95}));
    this.trace.frustumCulled = false; this.trace.renderOrder = 10;
    this.marker = new THREE.Mesh(new THREE.SphereGeometry(.0008, 16, 12),
      new THREE.MeshBasicMaterial({color: 0x78ffc2, depthTest: false}));
    this.marker.renderOrder = 11;
    this.axes = new THREE.AxesHelper(.07);
    this.failedMeshes = new Set();
    new ResizeObserver(() => {
      const w = container.clientWidth, h = container.clientHeight;
      if (w && h) { this.renderer.setSize(w, h, false); this.camera.aspect = w/h; this.camera.updateProjectionMatrix(); }
    }).observe(container);
    this.renderer.setAnimationLoop(() => {
      this.controls.update(); this.renderer.render(this.scene, this.camera);
    });
  }

  async load(info) {
    this.modelHash = info.model_hash;
    this.tipName = info.tip;
    this.tcpOffset = new THREE.Vector3(...(info.tcp_xyz_m || [0, 0, 0]));
    const manager = new THREE.LoadingManager();
    manager.onError = url => { this.failedMeshes.add(url); this.status('模型 mesh 加载失败，请检查资源'); };
    const loader = new URDFLoader(manager);
    let resolveMeshes;
    const ready = new Promise(resolve => { resolveMeshes = resolve; });
    manager.onLoad = resolveMeshes;
    this.robot = loader.parse(info.urdf, '');
    this.frame = this.robot.links[info.frame];
    if (!this.frame) throw new Error('模型中缺少点动参考系，无法对齐轨迹');
    this.scene.add(this.robot);
    this.frame.add(this.trace, this.marker, this.axes);
    if (this.lastState) this.update(this.lastState);
    // Establish an initial camera even before external meshes finish loading.
    this.fit();
    const hasMeshes = new DOMParser().parseFromString(info.urdf, 'text/xml').querySelector('visual mesh');
    if (hasMeshes) await ready;
    this.robot.traverse(child => {
      if (child.isMesh && child !== this.marker) {
        child.material = new THREE.MeshStandardMaterial({color: 0xb9c8c8, metalness: .2, roughness: .55});
      }
    });
    if (this.lastState) this.update(this.lastState);
    this.fit();
    if (this.failedMeshes.size) throw new Error('部分模型 mesh 未能加载');
    this.status('完整机器人已加载 · 浅绿高亮当前臂 · 轨迹绑定 TCP');
  }

  fit(selectedOnly = false) {
    if (!this.robot) return;
    this.robot.updateMatrixWorld(true);
    // Frame visuals only: the trail and coordinate helpers must not affect camera fit.
    const box = new THREE.Box3();
    this.robot.traverse(child => {
      if (child.isMesh && child !== this.marker && (!selectedOnly || this.activeLinks.has(child))) box.expandByObject(child);
    });
    if (box.isEmpty()) return;
    const sphere = box.getBoundingSphere(new THREE.Sphere());
    const fov = Math.min(THREE.MathUtils.degToRad(this.camera.fov),
      2 * Math.atan(Math.tan(THREE.MathUtils.degToRad(this.camera.fov)/2) * this.camera.aspect));
    const distance = sphere.radius / Math.sin(fov / 2) * 1.08;
    this.controls.target.copy(sphere.center);
    this.camera.position.copy(sphere.center).addScaledVector(new THREE.Vector3(.9, -1.2, .65).normalize(), distance);
    this.camera.near = .001; this.camera.far = Math.max(30, distance * 10); this.camera.updateProjectionMatrix();
  }

  focusTCP() {
    if (!this.frame || !this.lastState?.tcp_mm) return;
    const target = this.frame.localToWorld(new THREE.Vector3(...this.lastState.tcp_mm.map(v => v / 1000)));
    const direction = this.camera.position.clone().sub(this.controls.target).normalize();
    this.controls.target.copy(target); this.camera.position.copy(target).addScaledVector(direction, .35);
  }

  clear() {
    this.points = []; this.count = 0; this.lineGeometry.setDrawRange(0, 0);
    if (this.currentIdentity) this.trails.set(this.currentIdentity, this.points);
    if (this.lastState) this.update(this.lastState);
  }

  update(state) {
    this.lastState = state;
    if (!this.robot || !this.frame || !state.tcp_mm) return;
    if (this.modelHash && state.model_hash !== this.modelHash) { this.status('显示模型与状态身份不一致'); return; }
    const changed = this.currentIdentity !== state.model_identity;
    if (changed) {
      const frame = this.robot.links[state.frame], tip = this.robot.links[state.tip];
      if (!frame || !tip) { this.status('所选臂的参考系或 TCP 缺失'); return; }
      this.frame = frame; this.tipName = state.tip;
      this.tcpOffset = new THREE.Vector3(...(state.tcp_xyz_m || [0, 0, 0]));
      this.frame.add(this.trace, this.marker, this.axes);
      this.currentIdentity = state.model_identity;
      this.points = this.trails.get(this.currentIdentity) || [];
      this.trails.set(this.currentIdentity, this.points);
    }
    this.activeLinks.clear();
    this.robot.joints[state.joint_names[0]]?.traverse(child => { if (child.isMesh) this.activeLinks.add(child); });
    this.robot.traverse(child => {
      if (child.isMesh && child !== this.marker) child.material.color.setHex(this.activeLinks.has(child) ? 0x99d9c1 : 0xb9c8c8);
    });
    const jointNames = state.model_joint_names || state.joint_names, q = state.model_q_rad || state.q_rad;
    jointNames.forEach((name, i) => this.robot.joints[name]?.setJointValue(q[i]));
    this.robot.updateMatrixWorld(true);
    const position = new THREE.Vector3(...state.tcp_mm.map(v => v / 1000));
    this.marker.position.copy(position); this.axes.position.copy(position);
    const previous = this.points.at(-1);
    if (!previous || position.distanceToSquared(previous) > 1e-14) {
      this.points.push(position.clone());
      if (this.points.length > 4000) this.points.shift();
    }
    this.points.forEach((p, i) => p.toArray(this.traceBuffer, i * 3));
    this.lineGeometry.attributes.position.needsUpdate = true;
    this.lineGeometry.setDrawRange(0, this.points.length); this.count = this.points.length;
    this.renderer.domElement.dataset.activeArm = state.selected_arm || '';
    this.renderer.domElement.dataset.activeTip = state.tip;
    // Independently compare the model's transformed TCP with backend FK in the same frame.
    const tip = this.robot.links[this.tipName];
    if (tip) {
      const expected = this.frame.worldToLocal(tip.localToWorld(this.tcpOffset.clone()));
      const error = expected.distanceTo(position) * 1000;
      this.renderer.domElement.dataset.tcpAlignmentMm = error.toFixed(6);
      this.renderer.domElement.dataset.trailPoints = String(this.count);
      if (error > .01) this.status(`TCP 与模型对齐误差 ${error.toFixed(3)} mm`);
    }
  }
}
