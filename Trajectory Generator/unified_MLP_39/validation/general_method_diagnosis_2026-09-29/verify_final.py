"""Read-only verification of the finalized diagnostic inputs and artifacts."""
from __future__ import annotations
import ast
import hashlib
import json
from pathlib import Path
import re
from urllib.parse import unquote

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
RUNS = HERE.parents[1] / "runs/rollout/general_method_diagnosis_2026-09-29"
PLOTS = REPO / "plot/09_29_2026_general_method_diagnosis"


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    checked = {}
    manifests = {}
    for path in sorted(HERE.rglob("*input_hashes.json")):
        entries = read(path)
        for name, expected in entries.items():
            source = REPO / name
            actual = checked.setdefault(name, sha(source))
            assert actual == expected, (str(path), name, actual, expected)
        manifests[path.relative_to(HERE).as_posix()] = len(entries)

    cases = {}
    figure_count = 0
    for side in ("LEFT", "RIGHT"):
        for mode in ("baseline", "sequence", "impulse_knee", "impulse_ankle"):
            name = f"{side}_{mode}"
            folder = RUNS / name
            receipt = read(folder / "diagnostic_receipt.json")
            summary = read(folder / "rollout_summary.json")
            protocol = read(folder / "diagnostic_protocol.json")
            assert receipt["steps"] == summary["steps"]
            assert receipt["checkpoint_unchanged"]
            assert sha(REPO / protocol["checkpoint"] / "module_state.pkl") == protocol["checkpoint_sha256"]
            if mode == "baseline":
                assert receipt["comparison_to_archive_prefix_steps"] == 500
                assert all(x["exact"] and x["max_abs"] == 0 for x in receipt["archive_replay"].values())
                assert read(folder / "adopted_watchdog_summary.json")["ok"]
            else:
                watchdog = read(folder / "watchdog_summary.json")
                assert watchdog["returncode"] == 0 and watchdog["timeout_reason"] is None
            if mode.startswith("impulse"):
                assert all(x["exact"] and x["max_abs"] == 0 for x in receipt["archive_replay"].values())
            plot_receipt = read(PLOTS / name / "plot_receipt.json")
            assert len(plot_receipt["figures"]) == 7
            for figure, expected in plot_receipt["figures"].items():
                assert sha(REPO / figure) == expected, figure
            figure_count += 7
            cases[name] = {"steps": summary["steps"], "end_reason": summary["end_reason"],
                           "steps_requested": protocol["steps_requested"],
                           "checkpoint_unchanged": True, "seven_figures_verified": True}
    dynamic = read(HERE / "dynamic/analysis_verification.json")
    assert dynamic["input_hashes_unchanged"] and not dynamic["pending_cases"], dynamic
    assert len(dynamic["analyzed_cases"]) == 4
    assert sha(HERE / "dynamic/metrics.json") == dynamic["metrics_sha256"]
    assert sha(HERE / "dynamic/first_response_explanation.json") == dynamic["first_response_explanation_sha256"]
    syntax = []
    for path in sorted(HERE.rglob("*.py")):
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        syntax.append(path.relative_to(HERE).as_posix())
    documents = list(HERE.rglob("*.md")) + [REPO / "reports/user/2026-09-29_indagine_generale_39d_imitazione_e_forma_coppie.md"]
    for path in documents:
        for target in re.findall(r"\]\(([^)]+)\)", path.read_text(encoding="utf-8")):
            target = unquote(target.strip("<>"))
            if target.startswith(("http:", "https:", "#")):
                continue
            linked = (path.parent / target.split("#")[0]).resolve()
            # The report may link to this verification's own pending output.
            if linked == (HERE / "final_verification.json").resolve():
                continue
            assert linked.exists(), (str(path), target)
    result = {"passed": True, "input_manifests": manifests,
              "unique_input_files_verified": len(checked), "cases": cases,
              "canonical_png_files_verified": figure_count,
              "canonical_cycle_panel_limit": "Short impulse rollouts may have no complete gait cycle; those panels remain explicitly unavailable.",
              "python_syntax_files": syntax, "markdown_links_verified": len(documents),
              "document_sha256": {p.relative_to(REPO).as_posix(): sha(p) for p in documents},
              "scope": "Artifact consistency and recorded checks; no new simulation, training, or generalization claim."}
    (HERE / "final_verification.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
