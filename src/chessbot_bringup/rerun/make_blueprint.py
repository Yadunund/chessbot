# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0
"""Writes the chessbot Rerun layout to config/chessbot.rbl.

The Rerun bridge sends this file to every viewer as its layout: the robot in 3D
takes most of the screen, with cameras, joint plots and the robot's reasoning in
a column beside it. Entity paths match config/rerun_bridge.yaml.

Run `pixi run rerun-blueprint` after changing it, and commit the .rbl.
"""

from pathlib import Path

import rerun.blueprint as rrb

APPLICATION_ID = "chessbot"  # must match the bridge's application_id

blueprint = rrb.Blueprint(
    rrb.Horizontal(
        rrb.Spatial3DView(
            name="Scene",
            origin="/",
            contents=["robot/model/**", "tf/**"],
            # From the human's seat, opposite the robot, looking at the board
            # (robot base frame: the board centre is about 0.24 m ahead of the base).
            eye_controls=rrb.EyeControls3D(position=[0.85, 0.0, 0.5], look_target=[0.2, 0.0, 0.05], eye_up=[0, 0, 1]),
        ),
        rrb.Vertical(
            rrb.Horizontal(
                rrb.Spatial2DView(name="Overhead camera", origin="cameras/overhead"),
                rrb.Spatial2DView(name="Wrist camera", origin="cameras/wrist"),
            ),
            rrb.TimeSeriesView(name="Joints", origin="robot/joints"),
            rrb.Tabs(
                rrb.TextLogView(name="Thoughts", origin="reasoning/thoughts"),
                rrb.TextDocumentView(name="Game state", origin="game/state"),
            ),
            row_shares=[2, 2, 3],
        ),
        column_shares=[7, 3],
    ),
    rrb.BlueprintPanel(state="collapsed"),
    rrb.SelectionPanel(state="collapsed"),
    rrb.TimePanel(state="collapsed"),
    collapse_panels=False,
)

if __name__ == "__main__":
    out = Path(__file__).resolve().parent.parent / "config" / "chessbot.rbl"
    blueprint.save(APPLICATION_ID, out)
    print(f"wrote {out}")
