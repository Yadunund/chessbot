# Copyright 2026 Yadunund Vijay
# SPDX-License-Identifier: Apache-2.0

import json

from chessbot_calibration import profile as prof


def test_yaml_round_trip(tmp_path):
    path = str(tmp_path / "cal.yaml")
    original = prof.CalibrationProfile(source="measured")
    prof.save(path, original)
    loaded = prof.load(path)
    assert loaded is not None
    assert loaded.board == original.board
    assert loaded.source == "measured"


def test_missing_file_is_none(tmp_path):
    assert prof.load(str(tmp_path / "nope.yaml")) is None


def test_stale_schema_is_rejected(tmp_path):
    path = tmp_path / "old.yaml"
    path.write_text("version: 0\n")
    assert prof.load(str(path)) is None


def test_json_carries_version():
    data = json.loads(prof.CalibrationProfile().to_json())
    assert data["version"] == prof.SCHEMA_VERSION
    assert len(data["board"]["origin_xyz"]) == 3
