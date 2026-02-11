#!/usr/bin/env python3
"""Shared helpers for thesis batch evaluation scripts.

These helpers keep the public scripts deterministic and independent from
machine-specific absolute paths.
"""

from __future__ import annotations

import csv
import re
import subprocess
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

CONTACT_PATTERN = re.compile(
    r"Contact \d+:\n\s+F_Ac_W: \[([\-0-9.eE+]+), ([\-0-9.eE+]+), ([\-0-9.eE+]+)\]\n"
    r"\s+p_WC: \[([\-0-9.eE+]+), ([\-0-9.eE+]+), ([\-0-9.eE+]+)\]"
)
OBJECT_COM_PATTERN = re.compile(
    r"object_com: \[([\-0-9.eE+]+), ([\-0-9.eE+]+), ([\-0-9.eE+]+)\]"
)


def csv_float(row: Dict[str, str], key: str) -> float:
    return float(row[key])


def csv_int(row: Dict[str, str], key: str) -> int:
    return int(row[key])


def parse_grasp_rows(
    input_csv: Path,
    ids: Optional[Iterable[int]] = None,
    only_matrix_name: Optional[str] = "X_Drake_Grasp",
) -> List[Dict[str, object]]:
    """Parse grasp rows from a semicolon-separated CSV.

    Supports both formats used in this thesis:
    - grasp proposal CSV (`graspID`, `matrix_name`, ...)
    - post-processed CSV (`id`, position/orientation fields, ...)
    """
    wanted_ids = set(ids) if ids else None
    rows: List[Dict[str, object]] = []

    with input_csv.open("r", newline="") as f:
        reader = csv.DictReader(f, delimiter=";")
        for raw in reader:
            # Optional filter for X_Drake_Grasp rows in the proposal CSV format.
            if only_matrix_name and "matrix_name" in raw and raw["matrix_name"] != only_matrix_name:
                continue

            grasp_id_key = "graspID" if "graspID" in raw else "id"
            grasp_id = csv_int(raw, grasp_id_key)
            if wanted_ids and grasp_id not in wanted_ids:
                continue

            row = {
                "grasp_id": grasp_id,
                "position": [csv_float(raw, "position_x"), csv_float(raw, "position_y"), csv_float(raw, "position_z")],
                "orientation": [
                    csv_float(raw, "orientation_x"),
                    csv_float(raw, "orientation_y"),
                    csv_float(raw, "orientation_z"),
                    csv_float(raw, "orientation_w"),
                ],
                "gripper_opening": csv_float(raw, "gripper_opening"),
                "score": float(raw.get("score", "nan")),
            }
            rows.append(row)

    return rows


def min_z_from_obj(mesh_path: Path) -> float:
    """Return minimum z-coordinate from an OBJ file without Open3D dependency."""
    min_z: Optional[float] = None
    with mesh_path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if not line.startswith("v "):
                continue
            parts = line.strip().split()
            if len(parts) < 4:
                continue
            z = float(parts[3])
            min_z = z if min_z is None else min(min_z, z)

    if min_z is None:
        raise ValueError(f"No vertex lines found in mesh file: {mesh_path}")
    return min_z


def run_drake_command(command: Sequence[str], cwd: Path, verbose: bool = True) -> str:
    """Run Drake process and return captured stdout+stderr."""
    process = subprocess.Popen(
        list(command),
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        cwd=str(cwd),
    )

    out_lines: List[str] = []
    sent_enter = False

    assert process.stdout is not None
    for line in iter(process.stdout.readline, ""):
        if not line:
            break
        out_lines.append(line)
        if verbose:
            print(line, end="")

        # Both binaries block on stdin after printing one of these prompts.
        if ("[Press Enter to finish]." in line or "[Press Ctrl-C to finish]." in line) and not sent_enter:
            if process.stdin is not None:
                process.stdin.write("\n")
                process.stdin.flush()
                sent_enter = True

    remaining_out, remaining_err = process.communicate()
    out_lines.append(remaining_out)
    if remaining_err:
        out_lines.append("\n[stderr]\n")
        out_lines.append(remaining_err)
        if verbose:
            print(remaining_err, end="")

    return "".join(out_lines)


def parse_drake_contacts(output: str) -> Tuple[List[Dict[str, np.ndarray]], Optional[np.ndarray], int]:
    contacts: List[Dict[str, np.ndarray]] = []
    for match in CONTACT_PATTERN.findall(output):
        force = np.array([float(match[0]), float(match[1]), float(match[2])], dtype=float)
        pos = np.array([float(match[3]), float(match[4]), float(match[5])], dtype=float)
        contacts.append({"F_c_W": force, "p_WC_W": pos})

    com_match = OBJECT_COM_PATTERN.findall(output)
    p_wo_w: Optional[np.ndarray] = None
    if com_match:
        p_wo_w = np.array([float(com_match[0][0]), float(com_match[0][1]), float(com_match[0][2])], dtype=float)

    num_contacts = len(contacts)

    # Match legacy behavior: if there are exactly 3 contacts, drop the one most aligned
    # with +Z (typically table support contact) before quality evaluation.
    if len(contacts) == 3:
        up = np.array([0.0, 0.0, 1.0], dtype=float)
        cos_vals: List[float] = []
        for c in contacts:
            force = c["F_c_W"]
            norm = np.linalg.norm(force)
            if norm < 1e-12:
                cos_vals.append(-1.0)
                continue
            cos_vals.append(float(np.dot(force / norm, up)))
        remove_idx = int(np.argmax(cos_vals))
        contacts.pop(remove_idx)
        num_contacts = len(contacts)

    return contacts, p_wo_w, num_contacts


def default_binary_path(target_name: str) -> Path:
    """Resolve bazel-bin path relative to drake workspace root."""
    # .../drake/examples/simple_gripper/thesis_eval/_drake_batch_common.py
    workspace_root = Path(__file__).resolve().parents[3]
    return workspace_root / "bazel-bin" / "examples" / "simple_gripper" / target_name
