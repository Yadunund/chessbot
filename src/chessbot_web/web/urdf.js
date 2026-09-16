// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0
//
// Minimal URDF loader for three.js: links with visual geometry (mesh, box,
// cylinder, sphere), and fixed, revolute, continuous and prismatic joints,
// including mimic joints. Collision, inertia and plugin tags are ignored.

import * as THREE from "three";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

const stlLoader = new STLLoader();
const geometryCache = new Map();

function numbers(text, fallback) {
  if (!text) return fallback;
  return text.trim().split(/\s+/).map(Number);
}

// URDF rpy is fixed-axis roll, pitch, yaw: R = Rz(yaw) Ry(pitch) Rx(roll).
function applyOrigin(object, originEl) {
  if (!originEl) return;
  const [x, y, z] = numbers(originEl.getAttribute("xyz"), [0, 0, 0]);
  const [roll, pitch, yaw] = numbers(originEl.getAttribute("rpy"), [0, 0, 0]);
  object.position.set(x, y, z);
  object.rotation.set(roll, pitch, yaw, "ZYX");
}

function child(el, tag) {
  return [...el.children].find((c) => c.tagName === tag) || null;
}

function loadMesh(url) {
  if (!geometryCache.has(url)) {
    geometryCache.set(url, stlLoader.loadAsync(url));
  }
  return geometryCache.get(url);
}

async function visualObject(visualEl, options) {
  const geometryEl = child(visualEl, "geometry");
  if (!geometryEl) return null;
  const shape = geometryEl.children[0];
  if (!shape) return null;
  let object;
  switch (shape.tagName) {
    case "mesh": {
      const filename = shape.getAttribute("filename");
      const url = options.resolveUri(filename);
      if (!url || !/\.stl$/i.test(filename)) {
        console.warn(`urdf: skipping unsupported mesh ${filename}`);
        return null;
      }
      const geometry = await loadMesh(url);
      object = new THREE.Mesh(geometry, options.materialFor(filename));
      const [sx, sy, sz] = numbers(shape.getAttribute("scale"), [1, 1, 1]);
      object.scale.set(sx, sy, sz);
      break;
    }
    case "box": {
      const [x, y, z] = numbers(shape.getAttribute("size"), [0, 0, 0]);
      object = new THREE.Mesh(new THREE.BoxGeometry(x, y, z), options.materialFor(""));
      break;
    }
    case "cylinder": {
      const radius = Number(shape.getAttribute("radius"));
      const length = Number(shape.getAttribute("length"));
      const geometry = new THREE.CylinderGeometry(radius, radius, length, 24);
      geometry.rotateX(Math.PI / 2); // URDF cylinders run along z, three.js along y
      object = new THREE.Mesh(geometry, options.materialFor(""));
      break;
    }
    case "sphere":
      object = new THREE.Mesh(new THREE.SphereGeometry(Number(shape.getAttribute("radius")), 24, 16),
        options.materialFor(""));
      break;
    default:
      return null;
  }
  object.castShadow = true;
  object.receiveShadow = true;
  const wrapper = new THREE.Group();
  applyOrigin(wrapper, child(visualEl, "origin"));
  wrapper.add(object);
  return wrapper;
}

// Returns { root, links, setJointPositions(names, positions) }.
// options.resolveUri(uri) -> URL or null; options.materialFor(meshFilename) -> Material.
export async function loadUrdf(text, options) {
  const doc = new DOMParser().parseFromString(text, "application/xml");
  const robot = doc.documentElement;
  if (robot.tagName !== "robot") throw new Error("not a URDF document");

  const links = new Map();
  const pending = [];
  for (const linkEl of [...robot.children].filter((e) => e.tagName === "link")) {
    const group = new THREE.Group();
    group.name = linkEl.getAttribute("name");
    links.set(group.name, group);
    for (const visualEl of [...linkEl.children].filter((e) => e.tagName === "visual")) {
      pending.push(visualObject(visualEl, options).then((object) => object && group.add(object)));
    }
  }

  const joints = new Map();
  const childLinks = new Set();
  for (const jointEl of [...robot.children].filter((e) => e.tagName === "joint")) {
    const parent = links.get(child(jointEl, "parent")?.getAttribute("link"));
    const childLink = links.get(child(jointEl, "child")?.getAttribute("link"));
    if (!parent || !childLink) continue;
    const origin = new THREE.Group();
    applyOrigin(origin, child(jointEl, "origin"));
    const motion = new THREE.Group();
    parent.add(origin);
    origin.add(motion);
    motion.add(childLink);
    childLinks.add(childLink.name);
    const mimicEl = child(jointEl, "mimic");
    joints.set(jointEl.getAttribute("name"), {
      type: jointEl.getAttribute("type"),
      axis: new THREE.Vector3(...numbers(child(jointEl, "axis")?.getAttribute("xyz"), [1, 0, 0])).normalize(),
      motion,
      mimic: mimicEl && {
        joint: mimicEl.getAttribute("joint"),
        multiplier: Number(mimicEl.getAttribute("multiplier") ?? 1),
        offset: Number(mimicEl.getAttribute("offset") ?? 0),
      },
    });
  }

  const root = new THREE.Group();
  for (const link of links.values()) {
    if (!childLinks.has(link.name)) root.add(link);
  }
  await Promise.all(pending);

  function apply(joint, value) {
    if (joint.type === "revolute" || joint.type === "continuous") {
      joint.motion.quaternion.setFromAxisAngle(joint.axis, value);
    } else if (joint.type === "prismatic") {
      joint.motion.position.copy(joint.axis).multiplyScalar(value);
    }
  }

  function setJointPositions(names, positions) {
    const values = new Map();
    names.forEach((name, i) => {
      const joint = joints.get(name);
      if (joint && Number.isFinite(positions[i])) {
        apply(joint, positions[i]);
        values.set(name, positions[i]);
      }
    });
    for (const joint of joints.values()) {
      if (joint.mimic && values.has(joint.mimic.joint)) {
        apply(joint, values.get(joint.mimic.joint) * joint.mimic.multiplier + joint.mimic.offset);
      }
    }
  }

  return { root, links, setJointPositions };
}
