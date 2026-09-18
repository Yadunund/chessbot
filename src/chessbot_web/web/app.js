// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0
//
// Chessbot UI.
// - Commands and the initial snapshot: the brain's REST API (same origin).
// - Live updates: ROS topics straight from the Zenoh router's REST plugin (SSE),
//   decoded from CDR here. The snapshot is re-fetched periodically as a fallback.

import { base64ToBytes, decodeGameState, decodeJointState, decodeThought } from "./cdr.js";

const host = window.location.hostname;
const params = new URLSearchParams(window.location.search);
const ROUTER = params.get("router") || `http://${host}:8080`;
const RERUN_GRPC = params.get("rerun") || `rerun+http://${host}:9876/proxy`;
const VIEWER = params.get("viewer") || `http://${host}:9090`;

const PHASES = ["idle", "calibrating", "setup", "human_turn", "reading_board", "thinking", "moving",
  "verifying", "needs_help", "game_over"];
const PHASE_TEXT = {
  idle: "Idle", calibrating: "Calibrating", setup: "Setting up", human_turn: "Your turn",
  reading_board: "Reading the board", thinking: "Thinking", moving: "Moving",
  verifying: "Checking the board", needs_help: "Needs help", game_over: "Game over",
};
// Phases in which the robot is working and commands would be rejected.
const WORKING = new Set(["calibrating", "setup", "reading_board", "thinking", "moving", "verifying"]);
const KINDS = ["perceive", "decide", "plan", "act", "verify", "recover", "explain"];
const GLYPH = { K: "♚", Q: "♛", R: "♜", B: "♝", N: "♞", P: "♟" };

const CALIBRATION_KEY = "chessbot/kv/calibration";

const $ = (id) => document.getElementById(id);
let state = null;
let scene = null; // BoardScene once WebGL and three.js have loaded
let view = "player";
let jointSource = null;

// --- rendering -----------------------------------------------------------------------

const FILES = "abcdefgh";

// Squares are drawn from the human's side: white at the bottom when the robot
// plays black, flipped otherwise.
function renderBoard(fen, flipped, lastMove) {
  const placement = (fen || "8/8/8/8/8/8/8/8").split(" ")[0].split("/");
  const grid = {};
  placement.forEach((row, i) => {
    let file = 0;
    for (const ch of row) {
      if (/\d/.test(ch)) file += Number(ch);
      else grid[`${FILES[file++]}${8 - i}`] = ch;
    }
  });
  const highlight = lastMove ? [lastMove.slice(0, 2), lastMove.slice(2, 4)] : [];
  const board = $("board");
  board.replaceChildren();
  for (let r = 0; r < 8; r++) {
    for (let f = 0; f < 8; f++) {
      const rank = flipped ? r + 1 : 8 - r;
      const file = flipped ? 7 - f : f;
      const name = `${FILES[file]}${rank}`;
      const el = document.createElement("div");
      el.className = `sq ${(file + rank) % 2 === 1 ? "d" : "l"}${highlight.includes(name) ? " last" : ""}`;
      if (f === 0) el.append(coord("rank", rank));
      if (r === 7) el.append(coord("file", FILES[file]));
      const piece = grid[name];
      if (piece) {
        const span = document.createElement("span");
        span.className = `piece ${piece === piece.toUpperCase() ? "w" : "b"}`;
        span.textContent = GLYPH[piece.toUpperCase()];
        el.append(span);
      }
      board.append(el);
    }
  }
}

function coord(kind, text) {
  const span = document.createElement("span");
  span.className = `coord ${kind}`;
  span.textContent = text;
  return span;
}

function renderState() {
  if (!state) return;
  const phase = $("phase");
  phase.textContent = PHASE_TEXT[state.phase] || state.phase;
  phase.className = `tag ${state.phase}`;
  const humanSide = state.robot_side === "white" ? "black" : "white";
  renderBoard(state.fen, humanSide === "black", state.moves[state.moves.length - 1]);
  scene?.setBelief(state.fen, state.graveyard || []);
  renderStrips(humanSide);
  $("moves").textContent = state.moves.length ? formatMoves(state.moves) : "No moves yet";

  // Derived from the phase, which arrives live, rather than the polled busy flag.
  const busy = WORKING.has(state.phase);
  const yourTurn = state.phase === "human_turn" && !busy;
  $("clock").disabled = !yourTurn;
  $("move").disabled = !yourTurn;
  $("new-game").disabled = busy;
  $("park").disabled = busy;
  for (const button of document.querySelectorAll("#jog button, #setup-camera")) button.disabled = busy;
  $("resume").disabled = state.phase !== "needs_help" || busy;
  $("clock-label").textContent = state.phase === "human_turn" ? "Your clock" : "Robot's clock";
  renderClock(humanSide);
  if (state.capabilities) renderCapabilities(state.capabilities);
  if (state.last_error) $("error").textContent = state.last_error;
}

function formatMoves(moves) {
  const pairs = [];
  for (let i = 0; i < moves.length; i += 2) {
    pairs.push(`${i / 2 + 1}. ${moves[i]}${moves[i + 1] ? " " + moves[i + 1] : ""}`);
  }
  return pairs.join("   ");
}

function clockText(side) {
  const clock = state.clock;
  let ms = side === "white" ? clock.white_ms : clock.black_ms;
  if (clock.running === side) ms -= Date.now() - clock.last_switch_unix * 1000;
  ms = Math.max(0, ms);
  const m = Math.floor(ms / 60000);
  const s = Math.floor((ms % 60000) / 1000);
  return `${m}:${String(s).padStart(2, "0")}`;
}

function renderClock(humanSide) {
  const shown = state.phase === "human_turn" ? humanSide : state.robot_side;
  $("clock-time").textContent = clockText(shown);
  $("robot-time").textContent = clockText(state.robot_side);
  $("human-time").textContent = clockText(humanSide);
}

// Who plays which colour, whose turn it is, and what the robot is doing.
function renderStrips(humanSide) {
  const title = (side) => side[0].toUpperCase() + side.slice(1);
  $("robot-name").textContent = state.robot_name || "Robot";
  $("robot-side").textContent = `· ${title(state.robot_side)}`;
  $("human-side").textContent = `· ${title(humanSide)}`;
  $("robot-chip").className = `chip ${state.robot_side}`;
  $("human-chip").className = `chip ${humanSide}`;
  const robotActive = WORKING.has(state.phase) || state.clock.running === state.robot_side;
  const humanActive = state.phase === "human_turn";
  $("robot-strip").classList.toggle("active", robotActive && !humanActive);
  $("human-strip").classList.toggle("active", humanActive);
  $("robot-status").textContent = WORKING.has(state.phase) || state.phase === "needs_help" ? PHASE_TEXT[state.phase] : "";
  $("human-status").textContent = humanActive ? "Your move" : "";
}

function renderCapabilities(caps) {
  const el = $("caps");
  el.innerHTML = "";
  for (const [name, ok] of Object.entries(caps)) {
    const dot = document.createElement("span");
    dot.className = `dot ${ok ? "ok" : ""}`;
    dot.title = `${name}: ${ok ? "available" : "unavailable"}`;
    el.append(dot);
  }
}

function renderThoughts(thoughts) {
  const list = $("thoughts");
  list.innerHTML = "";
  for (const t of thoughts) addThought(t, false);
}

function addThought(t, scroll = true) {
  const li = document.createElement("li");
  const kind = document.createElement("span");
  kind.className = "kind";
  kind.textContent = t.kind;
  const text = document.createElement("span");
  text.textContent = t.text;
  if (t.model_generated) text.className = "model";
  li.append(kind, text);
  $("thoughts").append(li);
  if (scroll) li.scrollIntoView({ block: "nearest" });
}

// --- data ----------------------------------------------------------------------------

async function refresh() {
  try {
    const res = await fetch("/api/state");
    state = await res.json();
    renderState();
    renderThoughts(state.thoughts);
  } catch (err) {
    $("phase").textContent = "brain unreachable";
  }
}

function subscribe(topic, onBytes) {
  const source = new EventSource(`${ROUTER}/0/${topic}/**`);
  source.addEventListener("PUT", (event) => {
    try {
      const sample = JSON.parse(event.data);
      onBytes(base64ToBytes(sample.value));
    } catch (err) {
      console.warn(`could not decode ${topic}`, err);
    }
  });
  return source;
}

function live() {
  subscribe("chessbot/game_state", (bytes) => {
    const msg = decodeGameState(bytes);
    if (!state) return;
    // The graveyard is only in the REST snapshot; fetch it when a move lands.
    if (msg.moves.length !== state.moves.length) refresh();
    state.phase = PHASES[msg.phase] || state.phase;
    state.fen = msg.fen;
    state.moves = msg.moves;
    state.robot_side = msg.robot_side === 0 ? "white" : "black";
    state.last_error = "";
    state.clock = {
      white_ms: msg.white_ms,
      black_ms: msg.black_ms,
      running: [null, "white", "black"][msg.clock_running],
      last_switch_unix: msg.clock_last_switch.sec + msg.clock_last_switch.nanosec / 1e9,
    };
    renderState();
  });
  subscribe("chessbot/thoughts", (bytes) => {
    const msg = decodeThought(bytes);
    addThought({ kind: KINDS[msg.kind] || "?", text: msg.text, model_generated: msg.model_generated });
  });
}

async function post(path, body = {}) {
  const res = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!res.ok) {
    const detail = await res.json().catch(() => ({}));
    $("error").textContent = detail.detail || `${path} failed (${res.status})`;
  } else {
    $("error").textContent = "";
  }
}

// --- controls ------------------------------------------------------------------------

$("clock").addEventListener("click", () => {
  const move = $("move").value.trim();
  post("/api/press_clock", move ? { move } : {});
  $("move").value = "";
});
$("move").addEventListener("keydown", (event) => {
  if (event.key === "Enter" && !$("clock").disabled) $("clock").click();
});
document.addEventListener("keydown", (event) => {
  if (event.code === "Space" && event.target === document.body && !$("clock").disabled) {
    event.preventDefault();
    $("clock").click();
  }
});
$("new-game").addEventListener("click", () => {
  const human = document.querySelector('input[name="human-side"]:checked').value;
  post("/api/new_game", {
    robot_side: human === "white" ? "black" : "white",
    engine_elo: Number($("strength").value),
  });
});
$("park").addEventListener("click", () => post("/api/park"));
$("resume").addEventListener("click", () => post("/api/resume"));
$("demo").addEventListener("click", () =>
  post("/api/demo_transfer", { src: $("demo-src").value.trim(), dst: $("demo-dst").value.trim() }));
$("debug-view").addEventListener("click", () =>
  window.open(`${VIEWER}/?url=${encodeURIComponent(RERUN_GRPC)}`, "_blank", "noopener"));

// --- driving the arm by hand -----------------------------------------------------------
//
// For bringing a real rig up: before anything is calibrated the arm still has to be moved
// around, and the only safe way to do that is in small steps you can watch.

async function refreshToolPose() {
  try {
    const res = await fetch("/api/dev/tool_pose");
    if (!res.ok) { $("tool-pose").textContent = "tool —"; return; }
    const pose = await res.json();
    const [x, y, z] = pose.xyz;
    $("tool-pose").textContent = `tool ${(x * 100).toFixed(1)}, ${(y * 100).toFixed(1)}, ${(z * 100).toFixed(1)} cm`;
    ARM_JOINTS.forEach(([joint, label], i) => {
      const cell = $(`jv-${joint}`);
      if (cell && pose.joints) cell.textContent = `${label} ${((pose.joints[i] * 180) / Math.PI).toFixed(0)}°`;
    });
  } catch {
    $("tool-pose").textContent = "tool —";
  }
}

for (const button of document.querySelectorAll("[data-jog]")) {
  button.addEventListener("click", async () => {
    const step = Number($("jog-step").value) * Number(button.dataset.sign);
    await post("/api/dev/jog", { axis: button.dataset.jog, delta_m: step });
    setTimeout(refreshToolPose, 1200);
  });
}
// Per joint, for poses no tool target describes - posing the arm where it should wait, or
// backing one joint off a limit.
const ARM_JOINTS = [
  ["shoulder_pan_joint", "pan"],
  ["shoulder_lift_joint", "lift"],
  ["elbow_flex_joint", "elbow"],
  ["wrist_flex_joint", "wrist"],
  ["wrist_roll_joint", "roll"],
];
$("jog-joints").innerHTML = ARM_JOINTS.map(([joint, label]) =>
  `<span class="name" id="jv-${joint}">${label}</span>` +
  `<button data-joint="${joint}" data-sign="-1" type="button">−</button>` +
  `<button data-joint="${joint}" data-sign="1" type="button">+</button>`).join("");
for (const button of document.querySelectorAll("[data-joint]")) {
  button.addEventListener("click", async () => {
    const step = Number($("joint-step").value) * Number(button.dataset.sign);
    await post("/api/dev/jog_joint", { joint: button.dataset.joint, delta_rad: step });
    setTimeout(refreshToolPose, 1200);
  });
}

$("jaws-open").addEventListener("click", () => post("/api/dev/gripper", { open: true }));
$("jaws-close").addEventListener("click", () => post("/api/dev/gripper", { open: false }));
setInterval(refreshToolPose, 2000);
refreshToolPose();

// --- setting a real rig up -------------------------------------------------------------
//
// The board is wherever it was put and the camera is wherever it was mounted, so the parts
// of calibration that need a person (which corners are the board's, where the arm should
// wait) are asked for here rather than guessed.

const corners = [];
const CORNER_NAMES = ["a1", "h1", "h8", "a8"];

function drawCorners() {
  const marks = $("setup-marks");
  marks.innerHTML = corners
    .map(([x, y], i) => `<circle cx="${x}" cy="${y}" r="1.6" class="mark" />` +
      `<text x="${x + 2.4}" y="${y + 1}" class="mark-label">${CORNER_NAMES[i]}</text>`)
    .join("");
  $("board-result").textContent = corners.length < 4
    ? `Click ${CORNER_NAMES[corners.length]}.`
    : "Working out where the board is…";
}

function newPhoto() {
  corners.length = 0;
  drawCorners();
  $("setup-frame").src = `/api/camera/frame.png?scale=2&t=${Date.now()}`;
}

$("setup-frame").addEventListener("click", async (event) => {
  if (corners.length >= 4) return;
  const box = event.target.getBoundingClientRect();
  // Percentages, so the marks follow the image however it is scaled; the server is told
  // pixels in the full size frame.
  corners.push([((event.clientX - box.left) / box.width) * 100, ((event.clientY - box.top) / box.height) * 100]);
  drawCorners();
  if (corners.length < 4) return;
  const natural = { w: $("setup-frame").naturalWidth, h: $("setup-frame").naturalHeight };
  const scale = Number(new URL($("setup-frame").src, location.origin).searchParams.get("scale") || 1);
  const pixels = corners.map(([x, y]) => [(x / 100) * natural.w * scale, (y / 100) * natural.h * scale]);
  const height = Number($("board-height").value || 0) / 1000;
  const res = await fetch("/api/calibration/board", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ corners: pixels, surface_height_m: height }),
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) { $("board-result").textContent = data.detail || "Could not use those corners."; return; }
  const board = data.board || {};
  const reach = data.reach || {};
  const outOfReach = (reach.unreachable || []).length;
  $("board-result").textContent =
    `${board.square_size_mm} mm squares, turned ${board.yaw_deg}°, corners off by ${board.squareness_mm} mm. ` +
    `Measure a square: if it is not ${board.square_size_mm} mm, the height above is wrong. ` +
    (outOfReach ? `${outOfReach} squares out of reach — move the board closer.` : "Every square is reachable.");
});

$("setup-refresh").addEventListener("click", newPhoto);
$("setup-clear").addEventListener("click", newPhoto);
$("setup").addEventListener("click", () => { $("setup-dialog").showModal(); newPhoto(); });
$("setup-camera").addEventListener("click", async () => {
  $("camera-result").textContent = "Moving the arm so the camera can see the gripper…";
  await post("/api/calibration/camera");
});
$("setup-park").addEventListener("click", async () => {
  const res = await fetch("/api/calibration/park_here", { method: "POST" });
  const data = await res.json().catch(() => ({}));
  $("park-result").textContent = res.ok && data.park
    ? `Saved: ${data.park.joints.map((v) => v.toFixed(2)).join(", ")}`
    : data.detail || "Could not read the arm's pose.";
});
$("setup-save").addEventListener("click", async () => {
  const res = await fetch("/api/calibration/save", { method: "POST" });
  const data = await res.json().catch(() => ({}));
  $("save-result").textContent = res.ok ? `Saved to ${data.profile_path}` : data.detail || "Could not save.";
  if (res.ok) loadCalibration();
});

async function refreshSetup() {
  if (!$("setup-dialog").open) return;
  const res = await fetch("/api/calibration");
  if (!res.ok) return;
  const { draft } = await res.json();
  if (draft.camera) {
    const [x, y, z] = draft.camera.position_xyz;
    $("camera-result").textContent =
      `Camera at ${(x * 100).toFixed(0)}, ${(y * 100).toFixed(0)}, ${(z * 100).toFixed(0)} cm ` +
      `(fit within ${draft.camera.error_mean_px} px).`;
  }
  if (draft.park) $("park-result").textContent = `Saved: ${draft.park.joints.map((v) => v.toFixed(2)).join(", ")}`;
}
setInterval(refreshSetup, 2000);

// --- board views -----------------------------------------------------------------------

async function loadCalibration() {
  try {
    const res = await fetch(`${ROUTER}/${CALIBRATION_KEY}`);
    const [sample] = await res.json();
    if (!sample) return;
    const profile = typeof sample.value === "string" ? JSON.parse(sample.value) : sample.value;
    scene?.setCalibration(profile.board);
  } catch (err) {
    console.warn("could not read calibration", err);
  }
}

async function loadPieceSet() {
  try {
    const res = await fetch("/packages/chessbot_description/pieces/pieces.json");
    if (res.ok) scene.setPieceSet(await res.json());
  } catch (err) {
    console.warn("could not load the piece set", err);
  }
}

async function loadRobot() {
  for (;;) {
    try {
      const res = await fetch("/api/robot_description");
      if (res.ok) {
        const resolve = (uri) => (uri.startsWith("package://") ? `/packages/${uri.slice("package://".length)}` : null);
        await scene.setRobot(await res.text(), resolve);
        $("scene-note").textContent = "";
        return;
      }
    } catch (err) {
      console.warn("could not load the robot", err);
    }
    $("scene-note").textContent = "Waiting for the robot description…";
    await new Promise((resolve) => setTimeout(resolve, 3000));
  }
}

function setView(next) {
  view = next;
  try { localStorage.setItem("chessbot.view", view); } catch { /* storage unavailable */ }
  const threeD = view !== "2d" && scene;
  $("scene").hidden = !threeD;
  $("board").hidden = !!threeD;
  scene?.setActive(!!threeD);
  if (threeD) {
    scene.setView(view);
    jointSource ??= subscribe("joint_states", (bytes) => {
      const msg = decodeJointState(bytes);
      scene.setJointState(msg.name, msg.position);
    });
  } else if (jointSource) {
    jointSource.close(); // joint states are only needed while the robot is drawn
    jointSource = null;
  }
  document.querySelector(`input[name="view"][value="${view}"]`).checked = true;
}

async function initViews() {
  let saved = "player";
  try { saved = localStorage.getItem("chessbot.view") || saved; } catch { /* storage unavailable */ }
  try {
    const { BoardScene, webglAvailable } = await import("./scene3d.js");
    if (!webglAvailable()) throw new Error("WebGL is not available");
    scene = new BoardScene($("scene"));
    window.chessbotScene = scene; // for the browser console and UI tests
  } catch (err) {
    console.warn("3D view unavailable, showing the 2D board", err);
    document.querySelectorAll('input[name="view"]:not([value="2d"])').forEach((input) => { input.disabled = true; });
    saved = "2d";
  }
  document.querySelectorAll('input[name="view"]').forEach((input) =>
    input.addEventListener("change", () => setView(input.value)));
  setView(saved);
  if (!scene) return;
  if (state) scene.setBelief(state.fen, state.graveyard || []);
  loadPieceSet();
  loadCalibration();
  loadRobot();
  setInterval(loadCalibration, 10000);
}

refresh();
live();
initViews();
setInterval(refresh, 5000);
setInterval(() => state && renderState(), 1000);
