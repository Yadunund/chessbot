# Licensing policy

chessbot is Apache-2.0. **Every dependency we import, link, vendor, or ship must be Apache-2.0 compatible.** In practice that means Apache-2.0, MIT, BSD, ISC, zlib, or a dual license that includes one of these.

Not allowed: GPL, LGPL, AGPL, and non-commercial licenses (e.g. CC-BY-NC), whether for code or for model weights.

The web frontend is written in-house. Its only third-party JavaScript is three.js (MIT), vendored unmodified under `src/chessbot_web/web/vendor/three/` for the 3D view.

## Before adding a dependency

1. Look up its SPDX license, e.g. `https://api.github.com/repos/<owner>/<repo>` → `license.spdx_id`.
2. For model checkpoints, check the license on the *weights*, which can differ from the code. For example, Depth Anything V2 Small is Apache-2.0, but Base/Large/Giant are CC-BY-NC.
3. If a QP-solver-backed library (Pink, qpsolvers) is used, pin a permissive backend such as OSQP, ProxSuite, or DAQP. Don't use quadprog or CVXOPT, which are GPL.

## Known exclusions and their replacements

| Excluded | License | Instead |
|---|---|---|
| python-chess | GPL-3.0 | in-house chess rules library |
| chessground | GPL-3.0 | in-house SVG board |
| QuIK | AGPL-3.0 | roboplan / Pink / IKPy / EAIK |
| KDL (orocos_kinematics_dynamics) | LGPL-2.1 | Pinocchio |
| Depth Anything V2 Base/Large/Giant weights | CC-BY-NC-4.0 | Depth Anything V2 Small |

## Executables we run but do not link

| Program | License | How it is used |
|---|---|---|
| Stockfish | GPL-3.0 | Built from pinned, unmodified upstream source by `pixi run stockfish` into a separate executable. Driven only over UCI stdin/stdout from our own client. Never linked, vendored, or modified. |

Running a separate program over pipes is generally treated as aggregation rather than a combined work, so our code stays Apache-2.0.

If we ever distribute a pre-built image that includes the Stockfish binary, that image must also ship Stockfish's GPL-3.0 license text and offer its corresponding source.
