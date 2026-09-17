// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0
//
// 3D view of the brain's belief world: the board where calibration says it is,
// pieces where the game state says they are (plus the robot's graveyards), and
// the robot drawn from its URDF, posed by live joint states.
//
// Everything is in ROS coordinates (metres, z up). The board frame has its
// origin at a1's outer corner, +x towards the h-file, +y towards rank 8.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { loadUrdf } from "./urdf.js";

const COLORS = {
  background: 0xfdfcfc,
  table: 0xf5f3f1,
  boardEdge: 0xd9d3cb,
  lightSquare: "#efebe6",
  darkSquare: "#b9b1a7",
  labelOnLight: "#777169",
  labelOnDark: "#fdfcfc",
  whitePiece: 0xf1ece5,
  blackPiece: 0x2b2825,
  printedPart: 0xe9e4dd,
  motor: 0x34312d,
};
const BOARD_THICKNESS = 0.004;
const BOARD_MARGIN = 0.006;
const FILES = "abcdefgh";

export function webglAvailable() {
  try {
    return !!document.createElement("canvas").getContext("webgl2");
  } catch {
    return false;
  }
}

// --- pieces -------------------------------------------------------------------------
// Shapes and sizes come from the piece set description (chessbot_description/pieces/pieces.json):
// radii as fractions of the base radius, heights as fractions of each piece's height.

function pieceGeometries(type, set) {
  const radius = set.base_diameter_m / 2;
  const height = set.height_m[type];
  const lathe = new THREE.LatheGeometry(set.profile[type].map(([r, h]) => new THREE.Vector2(r * radius, h * height)), 40);
  lathe.rotateX(Math.PI / 2); // lathe axis y -> z
  const parts = [lathe];
  if (type === "k") {
    const cross = set.king_cross;
    const [uprightWidth, uprightHeight] = cross.upright;
    const [barWidth, barHeight] = cross.bar;
    const upright = new THREE.BoxGeometry(uprightWidth * radius, uprightWidth * radius, uprightHeight * height);
    upright.translate(0, 0, cross.centre * height);
    const bar = new THREE.BoxGeometry(barWidth * radius, uprightWidth * radius, barHeight * height);
    bar.translate(0, 0, cross.bar_centre * height);
    parts.push(upright, bar);
  } else if (type === "n") {
    const head = set.knight_head;
    const shape = new THREE.Shape(head.outline.map(([x, y]) => new THREE.Vector2(x * radius, y * height)));
    const depth = head.thickness * radius;
    const geometry = new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: false });
    geometry.translate(0, 0, -depth / 2);
    geometry.rotateX(Math.PI / 2); // outline plane xy -> xz
    parts.push(geometry);
  }
  return parts;
}

// --- scene --------------------------------------------------------------------------

export class BoardScene {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    container.append(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(COLORS.background);
    this.world = new THREE.Group(); // robot root frame
    this.scene.add(this.world);

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0xd8d2ca, 1.8));
    this.sun = new THREE.DirectionalLight(0xffffff, 2.2);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    Object.assign(this.sun.shadow.camera, { left: -0.4, right: 0.4, top: 0.4, bottom: -0.4, near: 0.1, far: 3 });
    this.sun.shadow.bias = -0.0004;
    this.sun.shadow.normalBias = 0.002;
    this.scene.add(this.sun, this.sun.target);

    this.materials = {
      white: new THREE.MeshStandardMaterial({ color: COLORS.whitePiece, roughness: 0.5 }),
      black: new THREE.MeshStandardMaterial({ color: COLORS.blackPiece, roughness: 0.45 }),
      printed: new THREE.MeshStandardMaterial({ color: COLORS.printedPart, roughness: 0.7 }),
      motor: new THREE.MeshStandardMaterial({ color: COLORS.motor, roughness: 0.5 }),
    };

    this.perspective = new THREE.PerspectiveCamera(35, 1, 0.01, 10);
    this.perspective.up.set(0, 0, 1);
    this.orthographic = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.01, 10);
    this.controls = new OrbitControls(this.perspective, this.renderer.domElement);
    this.controls.maxPolarAngle = THREE.MathUtils.degToRad(82);
    this.controls.minDistance = 0.15;
    this.controls.maxDistance = 1.5;
    this.controls.addEventListener("change", () => this.invalidate());

    this.robot = null;
    this.pieceSet = null; // piece set description, loaded by the app
    this.board = null; // { group, pieces, s, frameId, key }
    this.beliefKey = "";
    this.view = "player";
    this.active = false;
    this.dirty = true;

    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    const loop = () => {
      if (this.active && this.dirty) {
        this.dirty = false;
        this.renderer.render(this.scene, this.view === "top" ? this.orthographic : this.perspective);
      }
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }

  invalidate() { this.dirty = true; }

  setActive(active) {
    this.active = active;
    if (active) this.resize();
  }

  resize() {
    const { clientWidth: w, clientHeight: h } = this.container;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.perspective.aspect = w / h;
    this.perspective.updateProjectionMatrix();
    this.frameTopDown();
    this.invalidate();
  }

  async setRobot(urdfText, resolveUri) {
    const robot = await loadUrdf(urdfText, {
      resolveUri,
      materialFor: (filename) => (/sts3215|motor(?!_holder)/i.test(filename) ? this.materials.motor : this.materials.printed),
    });
    if (this.robot) this.world.remove(this.robot.root);
    this.robot = robot;
    this.world.add(robot.root);
    if (this.board) this.attachBoard();
    this.applyView();
  }

  setJointState(names, positions) {
    if (!this.robot) return;
    this.robot.setJointPositions(names, positions);
    this.invalidate();
  }

  // board: the calibration profile's board section.
  setCalibration(board) {
    const key = JSON.stringify(board);
    if (this.board?.key === key) return;
    if (this.board) this.board.group.removeFromParent();
    const s = board.square_size_m;
    const group = new THREE.Group();
    group.position.set(...board.origin_xyz);
    group.rotation.z = board.yaw_rad;

    const size = 8 * s;
    const slab = new THREE.Mesh(
      new THREE.BoxGeometry(size + 2 * BOARD_MARGIN, size + 2 * BOARD_MARGIN, BOARD_THICKNESS),
      new THREE.MeshStandardMaterial({ color: COLORS.boardEdge, roughness: 0.8 }),
    );
    slab.position.set(size / 2, size / 2, -BOARD_THICKNESS / 2 - 0.0002);
    slab.receiveShadow = true;
    const squares = new THREE.Mesh(
      new THREE.PlaneGeometry(size, size),
      new THREE.MeshStandardMaterial({ map: this.boardTexture(), roughness: 0.85 }),
    );
    squares.position.set(size / 2, size / 2, 0);
    squares.receiveShadow = true;
    const table = new THREE.Mesh(new THREE.PlaneGeometry(3, 3),
      new THREE.MeshStandardMaterial({ color: COLORS.table, roughness: 0.95 }));
    table.position.set(size / 2, size / 2, -BOARD_THICKNESS - 0.0004);
    table.receiveShadow = true;

    const pieces = new THREE.Group();
    group.add(table, slab, squares, pieces);
    this.board = { group, pieces, s, frameId: board.frame_id, key };
    this.beliefKey = "";
    this.attachBoard();
    if (this.belief) this.setBelief(this.belief.fen, this.belief.graveyard);
    this.applyView();
  }

  attachBoard() {
    const parent = this.robot?.links.get(this.board.frameId) || this.world;
    parent.add(this.board.group);
    this.invalidate();
  }

  boardTexture() {
    const px = 1024;
    const cell = px / 8;
    const canvas = document.createElement("canvas");
    canvas.width = canvas.height = px;
    const ctx = canvas.getContext("2d");
    ctx.font = `600 ${Math.round(cell * 0.2)}px Inter, system-ui, sans-serif`;
    for (let rank = 0; rank < 8; rank++) {
      for (let file = 0; file < 8; file++) {
        const dark = (file + rank) % 2 === 0; // a1 is dark
        const x = file * cell;
        const y = (7 - rank) * cell; // canvas top is rank 8
        ctx.fillStyle = dark ? COLORS.darkSquare : COLORS.lightSquare;
        ctx.fillRect(x, y, cell, cell);
        ctx.fillStyle = dark ? COLORS.labelOnDark : COLORS.labelOnLight;
        if (file === 0) {
          ctx.textAlign = "left";
          ctx.textBaseline = "top";
          ctx.fillText(String(rank + 1), x + cell * 0.07, y + cell * 0.06);
        }
        if (rank === 0) {
          ctx.textAlign = "right";
          ctx.textBaseline = "bottom";
          ctx.fillText(FILES[file], x + cell * 0.93, y + cell * 0.95);
        }
      }
    }
    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    texture.anisotropy = this.renderer.capabilities.getMaxAnisotropy();
    return texture;
  }

  // fen: the logical position; graveyard: [{piece, cell: [x, y]}] in squares.
  setBelief(fen, graveyard = []) {
    this.belief = { fen, graveyard };
    if (!this.board || !this.pieceSet) return;
    const key = JSON.stringify(this.belief);
    if (key === this.beliefKey) return;
    this.beliefKey = key;
    const { pieces, s } = this.board;
    pieces.clear();
    const place = (letter, x, y) => pieces.add(this.pieceMesh(letter, s, x * s, y * s));
    (fen || "8/8/8/8/8/8/8/8").split(" ")[0].split("/").forEach((row, i) => {
      let file = 0;
      for (const ch of row) {
        if (/\d/.test(ch)) file += Number(ch);
        else place(ch, file++ + 0.5, 7 - i + 0.5);
      }
    });
    for (const { piece, cell } of graveyard) place(piece, cell[0], cell[1]);
    this.invalidate();
  }

  setPieceSet(set) {
    this.pieceSet = set;
    this.geometryCache = new Map();
    this.beliefKey = "";
    if (this.belief) this.setBelief(this.belief.fen, this.belief.graveyard);
  }

  pieceMesh(letter, s, x, y) {
    const type = letter.toLowerCase();
    const white = letter !== type;
    this.geometryCache ??= new Map();
    if (!this.geometryCache.has(type)) this.geometryCache.set(type, pieceGeometries(type, this.pieceSet));
    const group = new THREE.Group();
    for (const geometry of this.geometryCache.get(cacheKey)) {
      const mesh = new THREE.Mesh(geometry, white ? this.materials.white : this.materials.black);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      group.add(mesh);
    }
    group.position.set(x, y, 0);
    // Knights face the opponent, turned a little so their profile shows from the player's seat.
    group.rotation.z = (white ? Math.PI / 2 : -Math.PI / 2) + (x < 4 * s ? 0.5 : -0.5);
    return group;
  }

  setView(view) {
    this.view = view;
    this.applyView();
  }

  // Where things are in the world: board centre, and the unit direction from
  // the robot across the board towards the human's seat.
  layout() {
    if (!this.board) return null;
    this.scene.updateMatrixWorld(true);
    const s = this.board.s;
    const centre = this.board.group.localToWorld(new THREE.Vector3(4 * s, 4 * s, 0));
    const base = new THREE.Vector3();
    (this.robot?.links.get("base_link") || this.world).getWorldPosition(base);
    const towardsHuman = centre.clone().sub(base).setZ(0);
    if (towardsHuman.lengthSq() < 1e-6) {
      // No robot: assume it sits at the rank-8 edge.
      towardsHuman.set(0, -1, 0).applyQuaternion(this.board.group.getWorldQuaternion(new THREE.Quaternion()));
    }
    return { centre, towardsHuman: towardsHuman.normalize(), s };
  }

  applyView() {
    const layout = this.layout();
    if (!layout) return;
    const { centre, towardsHuman, s } = layout;
    const lightOffset = towardsHuman.clone().multiplyScalar(0.25).add(new THREE.Vector3(0.2, 0, 0.8));
    this.sun.position.copy(centre).add(lightOffset);
    this.sun.target.position.copy(centre);

    const topDown = this.view === "top";
    for (const material of [this.materials.printed, this.materials.motor]) {
      material.transparent = topDown;
      material.opacity = topDown ? 0.3 : 1;
      material.depthWrite = !topDown;
      material.needsUpdate = true;
    }
    // Shadows help depth in 3D but only add clutter looking straight down.
    this.sun.castShadow = !topDown;
    for (const material of Object.values(this.materials)) material.needsUpdate = true;
    this.controls.enabled = !topDown;
    if (topDown) {
      this.frameTopDown();
    } else {
      // From the human's seat, high enough that the arm stands behind the board
      // rather than in front of the robot's pieces.
      const target = centre.clone().addScaledVector(towardsHuman, -3 * s).setZ(centre.z + 0.06);
      this.perspective.position.copy(centre).addScaledVector(towardsHuman, 0.5).setZ(centre.z + 0.58);
      this.controls.target.copy(target);
      this.controls.update();
    }
    this.invalidate();
  }

  // Looks straight down with the human's side at the bottom of the screen,
  // wide enough for the graveyards, tall enough to show the robot's base.
  frameTopDown() {
    const layout = this.layout();
    if (!layout) return;
    const { centre, towardsHuman, s } = layout;
    const aspect = this.perspective.aspect || 1;
    const halfHeight = Math.max(6.6 * s, (6.8 * s) / aspect);
    const cam = this.orthographic;
    Object.assign(cam, { left: -halfHeight * aspect, right: halfHeight * aspect, top: halfHeight, bottom: -halfHeight });
    const focus = centre.clone().addScaledVector(towardsHuman, -1.2 * s);
    cam.position.copy(focus).setZ(centre.z + 2);
    cam.up.copy(towardsHuman).negate();
    cam.lookAt(focus);
    cam.updateProjectionMatrix();
  }
}
