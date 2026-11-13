// scene.js — the 3D view (three.js). It only draws what it is told:
// poses, paths, arrows, axes, ellipsoids. All physics lives elsewhere.
//
// The space frame {0} has z up, so the camera's "up" is +z.
//
// TO ADD A VISUALIZATION: add a method here that owns its three.js objects
// (see setPath / setArrows / setEllipsoid for the pattern: create on first
// use, update in place, hide when given nothing), then call it from
// App.updateScene() in app.js.

import * as THREE from 'three';
import { OrbitControls } from '../../vendor/three/controls/OrbitControls.js';
import { Line2 } from '../../vendor/three/lines/Line2.js';
import { LineGeometry } from '../../vendor/three/lines/LineGeometry.js';
import { LineMaterial } from '../../vendor/three/lines/LineMaterial.js';
import { skeleton } from './geometry.js';

// Colours per theme. Series colours come from a palette validated for
// colour-vision-deficiency separation on each surface (see DECISIONS.md).
export const PALETTE = {
  dark: {
    background: 0x151c26, grid: 0x2a3748, gridMajor: 0x3b4b60, link: 0x8fa3b8, bone: 0xdfe7ef, base: 0x3a4658,
    joints: [0x199e70, 0xc98500, 0xd55181], desired: 0xe8edf3, predicted: 0x3987e5, belief: 0x9085e9, actual: 0xd95926,
    wrench: 0xe66767, bearing: 0xc3cedb, twist: 0xe8edf3, manip: 0x199e70, force: 0xe66767, ghost: 0x3987e5, text: '#dbe4ee',
  },
  light: {
    background: 0xfbfcfd, grid: 0xdfe5ec, gridMajor: 0xc3ccd6, link: 0x7c8b9b, bone: 0x1f2a37, base: 0xb8c2cd,
    joints: [0x1baf7a, 0xeda100, 0xe87ba4], desired: 0x1f2a37, predicted: 0x2a78d6, belief: 0x6250d6, actual: 0xeb6834,
    wrench: 0xe34948, bearing: 0x52606f, twist: 0x1f2a37, manip: 0x1baf7a, force: 0xe34948, ghost: 0x2a78d6, text: '#1a2433',
  },
};

const Z = new THREE.Vector3(0, 0, 1);
const Y = new THREE.Vector3(0, 1, 0);
const v3 = (p) => new THREE.Vector3(p[0], p[1], p[2]);

function matrixFromT(T) {
  return new THREE.Matrix4().set(
    T[0][0], T[0][1], T[0][2], T[0][3],
    T[1][0], T[1][1], T[1][2], T[1][3],
    T[2][0], T[2][1], T[2][2], T[2][3],
    0, 0, 0, 1,
  );
}

export class Scene3D {
  constructor(container, theme = 'dark') {
    this.container = container;
    this.theme = theme;
    this.colors = PALETTE[theme];
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    container.append(this.renderer.domElement);
    this.labelLayer = document.createElement('div');
    this.labelLayer.className = 'scene-labels';
    container.append(this.labelLayer);

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(38, 1, 0.01, 50);
    this.camera.up.copy(Z);
    this.camera.position.set(1.35, -1.55, 1.05);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0.25, 0, 0.3);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.12;
    this.controls.update();

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 1.1));
    const sun = new THREE.DirectionalLight(0xffffff, 1.6);
    sun.position.set(1.5, -1, 3);
    this.scene.add(sun);

    this.robotGroup = new THREE.Group();
    this.scene.add(this.robotGroup);
    this.lines = new Map();      // name -> Line2
    this.arrowGroups = new Map();// name -> { group, pool }
    this.ellipsoids = new Map(); // name -> Mesh
    this.ghostLines = [];
    this.axisLines = [];
    this.frames = [];
    this.labels = new Map();     // key -> { div, pos }
    this.resolution = new THREE.Vector2(1, 1);
    this.buildGround();
    this.setTheme(theme);
    this.resize();
    new ResizeObserver(() => this.resize()).observe(container);
  }

  // ----- static furniture -----

  buildGround() {
    if (this.grid) { this.scene.remove(this.grid); this.grid.geometry.dispose(); }
    this.grid = new THREE.GridHelper(2.4, 24, this.colors.gridMajor, this.colors.grid);
    this.grid.rotation.x = Math.PI / 2; // GridHelper lies in x–z; we want x–y
    this.scene.add(this.grid);
    if (!this.worldFrame) { this.worldFrame = this.makeTriad(0.16); this.scene.add(this.worldFrame); }
  }

  setTheme(theme) {
    this.theme = theme;
    this.colors = PALETTE[theme];
    this.scene.background = new THREE.Color(this.colors.background);
    this.buildGround();
    if (this.robot) this.setRobot(this.robot);
    for (const [name, line] of this.lines) if (this.colors[name]) line.material.color.set(this.colors[name]);
    for (const g of this.ghostLines) g.material.color.set(this.colors.ghost);
    this.labelLayer.style.color = this.colors.text;
  }

  // x red, y green, z blue: the usual convention for frame triads.
  makeTriad(size) {
    const g = new THREE.Group();
    const cols = [0xe05555, 0x3fae5a, 0x4a8be0];
    [[1, 0, 0], [0, 1, 0], [0, 0, 1]].forEach((d, i) => {
      const geo = new THREE.CylinderGeometry(size * 0.035, size * 0.035, size, 8);
      geo.translate(0, size / 2, 0);
      const m = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: cols[i] }));
      m.quaternion.setFromUnitVectors(Y, v3(d));
      g.add(m);
    });
    return g;
  }

  // ----- robot -----

  // Build meshes for a robot description. Link shapes come straight from the
  // spec, so editing a link's size in the Robot tab changes what you see.
  setRobot(robot) {
    this.robot = robot;
    this.robotGroup.clear();
    this.linkMeshes = [];
    this.jointMeshes = [];
    const linkMat = new THREE.MeshStandardMaterial({ color: this.colors.link, roughness: 0.55, metalness: 0.25, transparent: true, opacity: 0.82 });
    robot.spec.links.forEach((link) => {
      const s = link.shape;
      let geo;
      if (s.type === 'cylinder') { geo = new THREE.CylinderGeometry(s.r, s.r, s.h, 28); geo.rotateX(Math.PI / 2); }
      else if (s.type === 'box') geo = new THREE.BoxGeometry(s.lx, s.ly, s.lz);
      else if (s.type === 'ellipsoid') { geo = new THREE.SphereGeometry(1, 24, 16); geo.scale(s.a, s.b, s.c); }
      else geo = new THREE.SphereGeometry(0.045, 20, 14);
      const mesh = new THREE.Mesh(geo, linkMat);
      mesh.matrixAutoUpdate = false;
      this.robotGroup.add(mesh);
      this.linkMeshes.push(mesh);
    });
    for (let i = 0; i < robot.N; i++) {
      const geo = new THREE.CylinderGeometry(0.043, 0.043, 0.1, 24);
      const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: this.colors.joints[i % 3], roughness: 0.4, metalness: 0.2 }));
      this.robotGroup.add(mesh);
      this.jointMeshes.push(mesh);
    }
    const base = new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.13, 0.03, 36), new THREE.MeshStandardMaterial({ color: this.colors.base, roughness: 0.8 }));
    base.rotation.x = Math.PI / 2;
    base.position.z = 0.015;
    this.robotGroup.add(base);
    this.toolMesh = new THREE.Mesh(new THREE.SphereGeometry(0.022, 16, 12), new THREE.MeshBasicMaterial({ color: this.colors.bone }));
    this.robotGroup.add(this.toolMesh);
    // link frames {1}…{N} and the tool frame
    for (const f of this.frames) this.scene.remove(f);
    this.frames = [];
    for (let i = 0; i <= robot.N; i++) {
      const f = this.makeTriad(i === robot.N ? 0.09 : 0.06);
      f.matrixAutoUpdate = false;
      f.visible = false;
      this.scene.add(f);
      this.frames.push(f);
    }
  }

  updatePose(geom, { showFrames = false } = {}) {
    geom.T0.forEach((T, i) => {
      const m = matrixFromT(T);
      if (i < this.linkMeshes.length) this.linkMeshes[i].matrix.copy(m);
      this.frames[i].matrix.copy(m);
      this.frames[i].visible = showFrames || i === geom.T0.length - 1;
    });
    geom.jointPoints.forEach((p, i) => {
      this.jointMeshes[i].position.set(p[0], p[1], p[2]);
      this.jointMeshes[i].quaternion.setFromUnitVectors(Y, v3(geom.jointAxes[i]).normalize());
    });
    this.toolMesh.position.set(...geom.tool);
    this.setLine('bone', skeleton(geom), { color: this.colors.bone, width: 3.2 });
  }

  // ----- lines -----

  // Create or update a fat polyline. opts: { color, width (px), dashed, opacity }
  setLine(name, points, opts = {}) {
    let line = this.lines.get(name);
    if (!points || points.length < 2) { if (line) line.visible = false; return; }
    if (!line) {
      const mat = new LineMaterial({
        color: opts.color ?? this.colors[name] ?? 0xffffff, linewidth: opts.width ?? 2.5, transparent: true,
        opacity: opts.opacity ?? 1, dashed: !!opts.dashed, dashSize: 0.025, gapSize: 0.02, depthTest: opts.depthTest ?? true,
      });
      mat.resolution.copy(this.resolution);
      line = new Line2(new LineGeometry(), mat);
      line.frustumCulled = false;
      this.scene.add(line);
      this.lines.set(name, line);
    }
    const flat = new Float32Array(points.length * 3);
    for (let i = 0; i < points.length; i++) { flat[3 * i] = points[i][0]; flat[3 * i + 1] = points[i][1]; flat[3 * i + 2] = points[i][2]; }
    line.geometry.dispose();
    line.geometry = new LineGeometry();
    line.geometry.setPositions(flat);
    if (line.material.dashed) line.computeLineDistances();
    if (opts.color !== undefined) line.material.color.set(opts.color);
    line.visible = true;
  }

  // The four tool-point paths. Line style doubles the colour coding:
  // desired is dashed, belief is thin, predicted and actual are solid.
  setPath(name, points, visible = true) {
    const style = {
      desired: { dashed: true, width: 2, opacity: 0.85 },
      predicted: { width: 3 },
      belief: { width: 1.8, dashed: true },
      actual: { width: 3.4 },
    }[name];
    this.setLine(name, visible ? points : null, { ...style, color: this.colors[name] });
  }

  // Faint stick figures, e.g. future poses along the prediction or the PoE build-up.
  setGhosts(geoms, opacity = 0.35) {
    geoms.forEach((g, i) => {
      const name = `ghost${i}`;
      this.setLine(name, skeleton(g), { color: this.colors.ghost, width: 2, opacity });
      if (!this.ghostLines.includes(this.lines.get(name))) this.ghostLines.push(this.lines.get(name));
    });
    for (let i = geoms.length; i < this.ghostLines.length; i++) this.ghostLines[i].visible = false;
  }

  // Infinite-looking axis lines (screw axes): [{ point, dir, color, label }]
  setAxes(axes, length = 0.45) {
    axes.forEach((a, i) => {
      const p0 = a.point.map((x, k) => x - a.dir[k] * length), p1 = a.point.map((x, k) => x + a.dir[k] * length);
      this.setLine(`axis${i}`, [p0, p1], { color: a.color, width: 1.6, dashed: true, opacity: 0.9 });
      this.setLabel(`axis${i}`, a.label, p1);
    });
    for (let i = axes.length; i < 12; i++) {
      const l = this.lines.get(`axis${i}`);
      if (l) l.visible = false;
      this.setLabel(`axis${i}`, null);
    }
  }

  // ----- arrows -----

  // A named group of arrows: [{ from, vec, color, label, double }].
  // `vec` is already in metres (callers scale forces/torques/velocities).
  // `double` draws a second head: the convention for a moment or angular velocity.
  setArrows(name, arrows) {
    let entry = this.arrowGroups.get(name);
    if (!entry) {
      entry = { group: new THREE.Group(), pool: [] };
      this.scene.add(entry.group);
      this.arrowGroups.set(name, entry);
    }
    arrows.forEach((a, i) => {
      let obj = entry.pool[i];
      if (!obj) {
        const mat = new THREE.MeshBasicMaterial({ color: 0xffffff });
        const shaft = new THREE.Mesh(new THREE.CylinderGeometry(1, 1, 1, 10).translate(0, 0.5, 0), mat);
        const head = new THREE.Mesh(new THREE.ConeGeometry(1, 1, 14).translate(0, 0.5, 0), mat);
        const head2 = new THREE.Mesh(new THREE.ConeGeometry(1, 1, 14).translate(0, 0.5, 0), mat);
        obj = new THREE.Group();
        obj.add(shaft, head, head2);
        obj.userData = { shaft, head, head2, mat };
        entry.group.add(obj);
        entry.pool[i] = obj;
      }
      const len = Math.hypot(...a.vec);
      obj.visible = len > 1e-5;
      this.setLabel(`${name}${i}`, obj.visible ? a.label : null, a.from.map((x, k) => x + a.vec[k] * 1.08));
      if (!obj.visible) return;
      const { shaft, head, head2, mat } = obj.userData;
      mat.color.set(a.color);
      const r = 0.007, hl = Math.min(0.045, len * 0.4), hr = 0.018;
      shaft.scale.set(r, Math.max(len - hl, 1e-4), r);
      head.scale.set(hr, hl, hr);
      head.position.y = len - hl;
      head2.visible = !!a.double;
      head2.scale.set(hr, hl, hr);
      head2.position.y = Math.max(len - 2 * hl, 0);
      obj.position.set(...a.from);
      obj.quaternion.setFromUnitVectors(Y, v3(a.vec).normalize());
    });
    for (let i = arrows.length; i < entry.pool.length; i++) {
      entry.pool[i].visible = false;
      this.setLabel(`${name}${i}`, null);
    }
  }

  // ----- ellipsoids -----

  // spec: { center, axes: [3 unit vectors], radii: [3 lengths in metres], color } or null to hide.
  setEllipsoid(name, spec) {
    let mesh = this.ellipsoids.get(name);
    if (!spec) { if (mesh) mesh.visible = false; return; }
    if (!mesh) {
      mesh = new THREE.Mesh(
        new THREE.SphereGeometry(1, 28, 18),
        new THREE.MeshBasicMaterial({ color: spec.color, wireframe: true, transparent: true, opacity: 0.38 }),
      );
      mesh.matrixAutoUpdate = false;
      this.scene.add(mesh);
      this.ellipsoids.set(name, mesh);
    }
    mesh.material.color.set(spec.color);
    const [a, b, c] = spec.axes, [ra, rb, rc] = spec.radii;
    mesh.matrix.set(
      a[0] * ra, b[0] * rb, c[0] * rc, spec.center[0],
      a[1] * ra, b[1] * rb, c[1] * rc, spec.center[1],
      a[2] * ra, b[2] * rb, c[2] * rc, spec.center[2],
      0, 0, 0, 1,
    );
    mesh.visible = true;
  }

  // ----- text labels (HTML over the canvas) -----

  setLabel(key, text, pos) {
    let label = this.labels.get(key);
    if (!text) { if (label) label.div.style.display = 'none'; return; }
    if (!label) {
      label = { div: document.createElement('span'), pos: new THREE.Vector3() };
      label.div.className = 'scene-label';
      this.labelLayer.append(label.div);
      this.labels.set(key, label);
    }
    if (label.div.textContent !== text) label.div.textContent = text;
    label.pos.set(pos[0], pos[1], pos[2]);
    label.div.style.display = '';
  }

  // ----- frame loop -----

  resize() {
    const w = this.container.clientWidth || 1, h = this.container.clientHeight || 1;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.resolution.set(w, h);
    for (const line of this.lines.values()) line.material.resolution.copy(this.resolution);
  }

  render() {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    const w = this.resolution.x, h = this.resolution.y;
    const p = new THREE.Vector3();
    for (const label of this.labels.values()) {
      if (label.div.style.display === 'none') continue;
      p.copy(label.pos).project(this.camera);
      const hidden = p.z > 1 || Math.abs(p.x) > 1.05 || Math.abs(p.y) > 1.05;
      label.div.style.visibility = hidden ? 'hidden' : 'visible';
      label.div.style.transform = `translate(${((p.x + 1) / 2) * w}px, ${((1 - p.y) / 2) * h}px)`;
    }
  }

  resetView() {
    this.camera.position.set(1.35, -1.55, 1.05);
    this.controls.target.set(0.25, 0, 0.3);
    this.controls.update();
  }
}
