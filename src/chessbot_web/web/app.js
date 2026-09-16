// Copyright 2026 Yadunund Vijay
// SPDX-License-Identifier: Apache-2.0
//
// Chessbot UI.
// - Commands and the initial snapshot: the brain's REST API (same origin).
// - Live updates: ROS topics straight from the Zenoh router's REST plugin (SSE),
//   decoded from CDR here. The snapshot is re-fetched periodically as a fallback.

import { base64ToBytes, decodeGameState, decodeThought } from "./cdr.js";

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

const $ = (id) => document.getElementById(id);
let state = null;

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
  $("moves").textContent = state.moves.length ? formatMoves(state.moves) : "No moves yet";

  // Derived from the phase, which arrives live, rather than the polled busy flag.
  const busy = WORKING.has(state.phase);
  const yourTurn = state.phase === "human_turn" && !busy;
  $("clock").disabled = !yourTurn;
  $("move").disabled = !yourTurn;
  $("new-game").disabled = busy;
  $("calibrate").disabled = busy;
  $("park").disabled = busy;
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

function renderClock(humanSide) {
  const clock = state.clock;
  const shown = state.phase === "human_turn" ? humanSide : state.robot_side;
  let ms = shown === "white" ? clock.white_ms : clock.black_ms;
  if (clock.running === shown) ms -= Date.now() - clock.last_switch_unix * 1000;
  ms = Math.max(0, ms);
  const m = Math.floor(ms / 60000);
  const s = Math.floor((ms % 60000) / 1000);
  $("clock-time").textContent = `${m}:${String(s).padStart(2, "0")}`;
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
$("calibrate").addEventListener("click", () => post("/api/calibrate"));
$("park").addEventListener("click", () => post("/api/park"));
$("resume").addEventListener("click", () => post("/api/resume"));
$("demo").addEventListener("click", () =>
  post("/api/demo_transfer", { src: $("demo-src").value.trim(), dst: $("demo-dst").value.trim() }));
$("debug-view").addEventListener("click", () =>
  window.open(`${VIEWER}/?url=${encodeURIComponent(RERUN_GRPC)}`, "_blank", "noopener"));

refresh();
live();
setInterval(refresh, 5000);
setInterval(() => state && renderState(), 1000);
