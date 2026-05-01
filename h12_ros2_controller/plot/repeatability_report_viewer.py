#!/usr/bin/env python3
"""Repeatability report + interactive viewer for sweep_tilt runs."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import csv
from collections import defaultdict

import numpy as np
from scipy.spatial.transform import Rotation

try:
    import dash
    from dash import dcc, html
    from dash.dependencies import Input, Output
    import plotly.graph_objects as go

    _DASH_AVAILABLE = True
except Exception:  # pragma: no cover - dash is optional for report-only mode
    _DASH_AVAILABLE = False


DEFAULT_REAL_SUMMARY = (
    "runs/20260430_161543_sweep_tilt_real/summary.csv"
)
DEFAULT_SIM_SUMMARY = (
    "runs/20260430_154427_sweep_tilt_sim/summary.csv"
)
DEFAULT_OUTPUT_DIR = "runs/analysis_sweep_tilt_repeatability"


@dataclass(frozen=True)
class TargetPose:
    x: float
    y: float
    z: float
    r: float
    p: float
    yw: float


def _parse_target_from_case(case: str) -> TargetPose:
    # Expected format: left_pick_x30_y20_z06
    parts = case.split("_")
    vals = {p[0]: p[1:] for p in parts if len(p) > 1 and p[0] in {"x", "y", "z"}}
    def _to_m(val: str) -> float:
        return int(val) / 100.0

    x = _to_m(vals.get("x", "0"))
    y = _to_m(vals.get("y", "0"))
    z = _to_m(vals.get("z", "0"))
    if case.startswith("right_"):
        y = -abs(y)
    # This sweep uses a fixed wrist-down orientation.
    return TargetPose(x=x, y=y, z=z, r=0.0, p=math.pi / 2.0, yw=0.0)


def _arm_from_case(case: str) -> str:
    if case.startswith("left_"):
        return "left"
    if case.startswith("right_"):
        return "right"
    return "unknown"


def _quat_from_rpy(rpy: Iterable[float]) -> np.ndarray:
    return Rotation.from_euler("xyz", list(rpy), degrees=False).as_quat()


def _quat_geodesic(q1: np.ndarray, q2: np.ndarray) -> float:
    # q = [x, y, z, w]; geodesic distance with sign-invariant dot.
    dot = float(np.dot(q1, q2))
    dot = max(-1.0, min(1.0, abs(dot)))
    return 2.0 * math.acos(dot)


def load_summary(summary_path: Path, dataset: str) -> List[Dict[str, float]]:
    records: List[Dict[str, float]] = []
    with summary_path.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            record: Dict[str, float] = {}
            record["dataset"] = dataset
            record["case"] = row["case"]
            record["frame"] = row["frame"]
            record["trial_index"] = int(row["trial_index"])
            record["arm"] = _arm_from_case(record["case"])
            for key in [
                "ee_ss__x",
                "ee_ss__y",
                "ee_ss__z",
                "ee_ss__r",
                "ee_ss__p",
                "ee_ss__yw",
            ]:
                record[key] = float(row[key])

            target = _parse_target_from_case(record["case"])
            record["target_x"] = target.x
            record["target_y"] = target.y
            record["target_z"] = target.z
            record["target_r"] = target.r
            record["target_p"] = target.p
            record["target_yw"] = target.yw

            record["err_x"] = record["ee_ss__x"] - record["target_x"]
            record["err_y"] = record["ee_ss__y"] - record["target_y"]
            record["err_z"] = record["ee_ss__z"] - record["target_z"]
            record["err_l2"] = math.sqrt(
                record["err_x"] ** 2 + record["err_y"] ** 2 + record["err_z"] ** 2
            )

            q_target = _quat_from_rpy(
                [record["target_r"], record["target_p"], record["target_yw"]]
            )
            q_measured = _quat_from_rpy(
                [record["ee_ss__r"], record["ee_ss__p"], record["ee_ss__yw"]]
            )
            record["err_ang"] = _quat_geodesic(q_measured, q_target)

            records.append(record)
    return records


def _mean_abs(values: Iterable[float]) -> float:
    values = list(values)
    if not values:
        return float("nan")
    return float(np.mean(np.abs(values)))


def _mean(values: Iterable[float]) -> float:
    values = list(values)
    if not values:
        return float("nan")
    return float(np.mean(values))


def build_reports(records: List[Dict[str, float]]) -> Dict[str, List[Dict[str, float]]]:
    def _aggregate(keys: Tuple[str, ...]) -> List[Dict[str, float]]:
        buckets: Dict[Tuple[str, ...], List[Dict[str, float]]] = defaultdict(list)
        for record in records:
            group_key = tuple(record[k] for k in keys)
            buckets[group_key].append(record)

        output: List[Dict[str, float]] = []
        for group_key, items in sorted(buckets.items()):
            out_row: Dict[str, float] = {k: v for k, v in zip(keys, group_key)}
            out_row["n_trials"] = len(items)
            out_row["mean_l2_m"] = _mean([r["err_l2"] for r in items])
            out_row["mean_abs_x_m"] = _mean_abs([r["err_x"] for r in items])
            out_row["mean_abs_y_m"] = _mean_abs([r["err_y"] for r in items])
            out_row["mean_abs_z_m"] = _mean_abs([r["err_z"] for r in items])
            out_row["mean_ang_rad"] = _mean([r["err_ang"] for r in items])
            output.append(out_row)
        return output

    return {
        "cases": _aggregate(("dataset", "case", "arm", "frame")),
        "arms": _aggregate(("dataset", "arm")),
        "overall": _aggregate(("dataset",)),
    }


def save_reports(reports: Dict[str, List[Dict[str, float]]], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    for name, report in reports.items():
        if not report:
            continue
        fieldnames = list(report[0].keys())
        with (output_dir / f"report_{name}.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(report)


def _filter_by_axis(
    records: List[Dict[str, float]], fixed_axis: str, fixed_value: float, tol: float
) -> List[Dict[str, float]]:
    axis_map = {"x": "target_x", "y": "target_y", "z": "target_z"}
    col = axis_map[fixed_axis]
    return [r for r in records if abs(r[col] - fixed_value) <= tol]


def _scatter_2d(records: List[Dict[str, float]], fixed_axis: str) -> go.Figure:
    axes = ["x", "y", "z"]
    axes.remove(fixed_axis)
    a1, a2 = axes
    fig = go.Figure()

    for dataset, color in [("real", "#1f77b4"), ("sim", "#ff7f0e")]:
        subset = [r for r in records if r["dataset"] == dataset]
        if not subset:
            continue
        fig.add_trace(
            go.Scatter(
                x=[r[f"ee_ss__{a1}"] for r in subset],
                y=[r[f"ee_ss__{a2}"] for r in subset],
                mode="markers",
                name=f"{dataset} reached",
                marker=dict(color=color, size=6, opacity=0.7),
                hovertext=[r["case"] for r in subset],
            )
        )

    seen = set()
    targets = []
    for record in records:
        if record["case"] in seen:
            continue
        seen.add(record["case"])
        targets.append(record)
    fig.add_trace(
        go.Scatter(
            x=[r[f"target_{a1}"] for r in targets],
            y=[r[f"target_{a2}"] for r in targets],
            mode="markers",
            name="target",
            marker=dict(color="#111111", size=10, symbol="x"),
            hovertext=[r["case"] for r in targets],
        )
    )

    fig.update_layout(
        xaxis_title=f"{a1} (m)",
        yaxis_title=f"{a2} (m)",
        height=600,
        legend=dict(orientation="h"),
        margin=dict(l=40, r=20, t=40, b=40),
    )
    return fig


def _scatter_3d(records: List[Dict[str, float]]) -> go.Figure:
    fig = go.Figure()
    for dataset, color in [("real", "#1f77b4"), ("sim", "#ff7f0e")]:
        subset = [r for r in records if r["dataset"] == dataset]
        if not subset:
            continue
        fig.add_trace(
            go.Scatter3d(
                x=[r["ee_ss__x"] for r in subset],
                y=[r["ee_ss__y"] for r in subset],
                z=[r["ee_ss__z"] for r in subset],
                mode="markers",
                name=f"{dataset} reached",
                marker=dict(size=4, color=color, opacity=0.7),
                hovertext=[r["case"] for r in subset],
            )
        )

    seen = set()
    targets = []
    for record in records:
        if record["case"] in seen:
            continue
        seen.add(record["case"])
        targets.append(record)
    fig.add_trace(
        go.Scatter3d(
            x=[r["target_x"] for r in targets],
            y=[r["target_y"] for r in targets],
            z=[r["target_z"] for r in targets],
            mode="markers",
            name="target",
            marker=dict(size=6, color="#111111", symbol="x"),
            hovertext=[r["case"] for r in targets],
        )
    )

    fig.update_layout(
        scene=dict(
            xaxis_title="x (m)",
            yaxis_title="y (m)",
            zaxis_title="z (m)",
        ),
        height=700,
        margin=dict(l=0, r=0, t=40, b=0),
    )
    return fig


def launch_viewer(records: List[Dict[str, float]], title: str) -> None:
    if not _DASH_AVAILABLE:
        raise RuntimeError("Dash is not installed. Run with --no-viewer or install dash.")

    app = dash.Dash(__name__)

    arms = sorted({r["arm"] for r in records})
    frames = sorted({r["frame"] for r in records})
    datasets = ["real", "sim", "both"]

    target_values = {
        "x": sorted({r["target_x"] for r in records}),
        "y": sorted({r["target_y"] for r in records}),
        "z": sorted({r["target_z"] for r in records}),
    }

    app.layout = html.Div(
        [
            html.H2(title),
            html.Div(
                [
                    html.Div(
                        [
                            html.Label("Dataset"),
                            dcc.Dropdown(
                                id="dataset", options=[{"label": d, "value": d} for d in datasets],
                                value="real",
                            ),
                        ],
                        style={"width": "18%", "display": "inline-block", "padding": "0 10px"},
                    ),
                    html.Div(
                        [
                            html.Label("Arm"),
                            dcc.Dropdown(
                                id="arm", options=[{"label": "all", "value": "all"}] +
                                [{"label": a, "value": a} for a in arms],
                                value="all",
                            ),
                        ],
                        style={"width": "18%", "display": "inline-block", "padding": "0 10px"},
                    ),
                    html.Div(
                        [
                            html.Label("Frame"),
                            dcc.Dropdown(
                                id="frame", options=[{"label": "all", "value": "all"}] +
                                [{"label": f, "value": f} for f in frames],
                                value="all",
                            ),
                        ],
                        style={"width": "22%", "display": "inline-block", "padding": "0 10px"},
                    ),
                    html.Div(
                        [
                            html.Label("Fixed Axis"),
                            dcc.Dropdown(
                                id="fixed_axis", options=[{"label": a, "value": a} for a in ["x", "y", "z"]],
                                value="z",
                            ),
                        ],
                        style={"width": "18%", "display": "inline-block", "padding": "0 10px"},
                    ),
                    html.Div(
                        [
                            html.Label("Fixed Value"),
                            dcc.Dropdown(id="fixed_value"),
                        ],
                        style={"width": "18%", "display": "inline-block", "padding": "0 10px"},
                    ),
                ]
            ),
            html.Div(
                [
                    html.Label("Fixed Axis Tolerance (m)"),
                    dcc.Slider(
                        id="fixed_tol",
                        min=0.0,
                        max=0.02,
                        step=0.001,
                        value=0.001,
                        marks={0.0: "0", 0.01: "0.01", 0.02: "0.02"},
                    ),
                ],
                style={"padding": "10px 10px 20px 10px"},
            ),
            dcc.Tabs(
                [
                    dcc.Tab(label="2D Slice", children=[dcc.Graph(id="plot2d")]),
                    dcc.Tab(label="3D View", children=[dcc.Graph(id="plot3d")]),
                ]
            ),
        ]
    )

    @app.callback(
        Output("fixed_value", "options"),
        Output("fixed_value", "value"),
        Input("fixed_axis", "value"),
    )
    def _update_fixed_values(axis: str):
        values = target_values[axis]
        options = [{"label": f"{v:.2f}", "value": v} for v in values]
        return options, values[0]

    @app.callback(
        Output("plot2d", "figure"),
        Output("plot3d", "figure"),
        Input("dataset", "value"),
        Input("arm", "value"),
        Input("frame", "value"),
        Input("fixed_axis", "value"),
        Input("fixed_value", "value"),
        Input("fixed_tol", "value"),
    )
    def _update_plots(dataset: str, arm: str, frame: str, fixed_axis: str, fixed_value: float, fixed_tol: float):
        subset = records
        if dataset != "both":
            subset = [r for r in subset if r["dataset"] == dataset]
        if arm != "all":
            subset = [r for r in subset if r["arm"] == arm]
        if frame != "all":
            subset = [r for r in subset if r["frame"] == frame]
        subset = _filter_by_axis(subset, fixed_axis, fixed_value, fixed_tol)
        return _scatter_2d(subset, fixed_axis), _scatter_3d(subset)

    app.run(debug=False)


def main() -> None:
    parser = argparse.ArgumentParser(description="Repeatability report and viewer.")
    parser.add_argument("--real-summary", type=Path, default=Path(DEFAULT_REAL_SUMMARY))
    parser.add_argument("--sim-summary", type=Path, default=Path(DEFAULT_SIM_SUMMARY))
    parser.add_argument("--output-dir", type=Path, default=Path(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--no-viewer", action="store_true")
    args = parser.parse_args()

    all_records: List[Dict[str, float]] = []
    if args.real_summary.exists():
        all_records.extend(load_summary(args.real_summary, "real"))
    if args.sim_summary.exists():
        all_records.extend(load_summary(args.sim_summary, "sim"))

    if not all_records:
        raise FileNotFoundError("No summary.csv files found.")

    reports = build_reports(all_records)
    save_reports(reports, args.output_dir)

    report_path = args.output_dir / "report_overall.csv"
    print(f"Wrote reports to: {args.output_dir}")
    print(f"Overall report: {report_path}")

    if not args.no_viewer:
        launch_viewer(all_records, "Sweep Tilt Repeatability Viewer")


if __name__ == "__main__":
    main()
