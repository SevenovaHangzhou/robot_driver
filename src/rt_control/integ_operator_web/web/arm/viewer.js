// Robot Model view: the whole vehicle at the measured pose (solid) and the controlled
// arm at the target pose (translucent ghost).
import * as THREE from "./vendor/three.module.min.js";
import { OrbitControls } from "./vendor/OrbitControls.js";
import URDFLoader from "./vendor/URDFLoader.js";

THREE.Cache.enabled = true; // both copies share one download per mesh

const MEASURED = { color: 0xc9ccd6, metalness: 0.15, roughness: 0.55 };
const GHOST = { color: 0x56dcff, transparent: true, opacity: 0.32, depthWrite: false };

export class ArmViewer {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.appendChild(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(40, 1, 0.01, 50);
    this.camera.up.set(0, 0, 1); // URDF is Z-up
    this.camera.position.set(1.4, -1.6, 1.1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x30303a, 2.2));
    const sun = new THREE.DirectionalLight(0xffffff, 1.6);
    sun.position.set(2, -3, 4);
    this.scene.add(sun);
    const grid = new THREE.GridHelper(4, 40, 0x45454c, 0x2a2a2e);
    grid.rotation.x = Math.PI / 2;
    this.scene.add(grid);
    this.scene.add(new THREE.AxesHelper(0.15));
    this.measured = null;
    this.ghost = null;
    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    const loop = () => {
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
      requestAnimationFrame(loop);
    };
    loop();
  }

  resize() {
    const { clientWidth: w, clientHeight: h } = this.container;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  async load(urdf, armLinks = []) {
    // The ghost only needs the arm: skip the other meshes there (they are tens of MB).
    const keep = new Set(armLinks);
    const armMeshes = keep.size ? meshesOf(urdf, keep) : null;
    // Frame again once every mesh is in: the first frame may come before large meshes load.
    const [measured, ghost] = await Promise.all([
      parse(urdf, MEASURED, null, (robot) => this.frame(robot)),
      parse(urdf, GHOST, armMeshes),
    ]);
    if (keep.size) {
      ghost.traverse((child) => {
        if (child.isURDFVisual && !(child.parent && keep.has(child.parent.urdfName))) {
          child.visible = false;
        }
      });
    }
    for (const previous of [this.measured, this.ghost]) {
      if (previous) this.scene.remove(previous);
    }
    this.measured = measured;
    this.ghost = ghost;
    this.scene.add(measured, ghost);
    this.frame(measured);
  }

  frame(robot) {
    // Fit the bounding sphere into the narrower of the vertical and horizontal fields of view.
    const box = new THREE.Box3().setFromObject(robot);
    if (box.isEmpty()) return;
    const sphere = box.getBoundingSphere(new THREE.Sphere());
    const vertical = THREE.MathUtils.degToRad(this.camera.fov);
    const horizontal = 2 * Math.atan(Math.tan(vertical / 2) * this.camera.aspect);
    const distance = (sphere.radius / Math.sin(Math.min(vertical, horizontal) / 2)) * 1.05;
    const direction = new THREE.Vector3(0.9, -1.1, 0.6).normalize();
    this.controls.target.copy(sphere.center);
    this.camera.position.copy(sphere.center).addScaledVector(direction, distance);
    this.camera.near = distance / 100;
    this.camera.far = distance * 20;
    this.camera.updateProjectionMatrix();
  }

  setMeasured(values) { apply(this.measured, values); }

  setGhost(values, visible) {
    if (!this.ghost) return;
    this.ghost.visible = visible;
    if (visible) apply(this.ghost, values);
  }
}

function apply(robot, values) {
  if (!robot) return;
  for (const [name, value] of Object.entries(values)) {
    if (Number.isFinite(value) && robot.joints[name]) robot.joints[name].setJointValue(value);
  }
}

function meshesOf(urdf, links) {
  const xml = new DOMParser().parseFromString(urdf, "text/xml");
  const files = new Set();
  for (const link of xml.querySelectorAll("robot > link")) {
    if (!links.has(link.getAttribute("name"))) continue;
    for (const mesh of link.querySelectorAll("visual mesh")) files.add(mesh.getAttribute("filename"));
  }
  return files;
}

function parse(urdf, style, onlyMeshes, onAllLoaded = null) {
  return new Promise((resolve) => {
    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    if (onlyMeshes) {
      loader.loadMeshCb = (path, meshManager, material, done) => {
        if (!onlyMeshes.has(path)) {
          done(null); // not part of the ghost
          return;
        }
        loader.defaultMeshLoader(path, meshManager, material, done);
      };
    }
    const material = new THREE.MeshStandardMaterial(style);
    let robot = null;
    let settled = false;
    const finish = () => {
      // One material per copy, for meshes and URDF primitives alike.
      robot.traverse((child) => { if (child.isMesh) child.material = material; });
      if (!settled) resolve(robot);
      settled = true;
    };
    manager.onLoad = () => {
      finish();
      if (onAllLoaded) onAllLoaded(robot);
    };
    robot = loader.parse(urdf, "");
    // A model without mesh files never triggers onLoad.
    setTimeout(finish, 4000);
  });
}
