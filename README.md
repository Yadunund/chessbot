# chessbot

Play chess against a real robot arm on a physical board.

A robot arm (starting with the [SO-101](https://github.com/TheRobotStudio/SO-ARM100)) watches the board with an overhead and a wrist camera, plans its reply, and moves the pieces itself. The whole stack runs on a small computer next to the robot. Heavier inference can optionally run on another machine on the network, reached over `rmw_zenoh`. You control it from a web page on the same network.

> **Status:** early design. So far the repo contains the workspace and a reachability analysis tool.

## Quick start

Requires [pixi](https://pixi.sh). Supported platforms: `linux-aarch64`, `linux-64`, `osx-arm64`.

```bash
pixi run reach-check   # imports the SO-101 description, builds it, and checks board reachability
pixi run stockfish     # builds the Stockfish engine from pinned upstream source for this CPU
```

`reach-check` reports, for each square, the gripper tilt from vertical needed to pick and carry a piece. It also finds the board placement that minimises the worst-case tilt. Defaults are a 21 cm board; see `python tools/reach_check.py --help`.

## Layout

| Path | Contents |
|---|---|
| `pixi.toml` | the single workspace for all environments |
| `chessbot.repos` | pinned external source dependencies (vcstool) |
| `tools/` | standalone analysis and calibration tools |
| `generated/`, `external/` | build outputs and imported sources (gitignored) |
| `docs/` | user and contributor documentation |

## License

Apache-2.0. All dependencies must be Apache-2.0 compatible; see [docs/licensing.md](docs/licensing.md).
