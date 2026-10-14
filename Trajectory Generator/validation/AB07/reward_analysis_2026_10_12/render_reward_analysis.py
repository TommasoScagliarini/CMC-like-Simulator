"""Read-only analysis of saved telemetry; export explanatory example plots.

Run with Python + NumPy + Matplotlib. No OpenSim or checkpoint loading occurs.
Artifacts are written under the repository's plot/ tree, never into the run.
"""
from __future__ import annotations

import ast
import csv
import hashlib
import json
import os
from pathlib import Path
import tempfile

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "cmc_reward_plot_cache"))
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
AUDIT = Path(__file__).resolve().parent
RUN_NAME = "MLP_V26C_best47_AB07_pylon400_2026_10_07_6p11s"
RUN = ROOT / "Trajectory Generator" / "runs" / "rollout" / RUN_NAME
RECEIPT = ROOT / "Trajectory Generator/validation/AB07/v26c_best47_pylon400_2026_10_07/runtime_receipt.json"
SOURCE = ROOT / "Trajectory Generator/baseline_MLP/reward_function.py"
OUT = ROOT / "plot" / RUN_NAME / "reward_analysis_2026_10_12"
BLUE, TEAL, RED, GOLD, GREY = "#245da8", "#087f8c", "#c24747", "#cf902b", "#5d6875"


def pure_source_functions():
    wanted = {"_score", "_huber_loss", "_huber_interval_loss", "_soft_window_score"}
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in wanted]
    if len(nodes) != len(wanted):
        raise RuntimeError("Missing mathematical function in reward source")
    scope = {}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), "exec"), scope)
    return scope


def style(ax, xlabel, ylabel):
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(alpha=.17)
    ax.spines[["top", "right"]].set_visible(False)
    ax.set_axisbelow(True)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 12,
        "axes.titlesize": 13, "axes.titleweight": "bold",
        "figure.facecolor": "white", "axes.facecolor": "white",
        "savefig.facecolor": "white", "legend.frameon": False,
        "axes.labelcolor": "#27313c", "text.color": "#27313c",
        "pdf.fonttype": 42, "svg.fonttype": "none",
    })
    functions = pure_source_functions()
    huber = np.vectorize(functions["_huber_loss"])
    interval = np.vectorize(functions["_huber_interval_loss"])
    window = np.vectorize(functions["_soft_window_score"])
    receipt = json.loads(RECEIPT.read_text(encoding="utf-8"))
    cfg = receipt["requested_env_config"]["reward"]
    start = receipt["actual_episode_start_s"]
    diagnostic_path = RUN / "runtime_diagnostics.jsonl"
    with diagnostic_path.open(encoding="utf-8") as stream:
        rows = [json.loads(line) for line in stream]
    components = [r["reward_components"] for r in rows]
    t = np.array([r["time"] - start for r in rows])
    reward = np.array([r["recorded_returned_reward"] for r in rows])
    knee = np.array([c["morphology_knee_term"] for c in components])
    ankle = np.array([c["morphology_ankle_term"] for c in components])
    morphology = np.array([c["morphology_term"] for c in components])
    np.testing.assert_allclose(knee + ankle, morphology, rtol=1e-13, atol=1e-13)
    samples = [s for row in rows for s in row["morphology_causal_samples"]]
    normal_force = np.array([r["online_gait"]["sides"]["left"]["normal_force_n"] for r in rows])
    force_bw = np.array([r["online_gait"]["sides"]["left"]["normal_force_bw"] for r in rows])
    positive = force_bw > 1e-12
    body_weight = float(np.median(normal_force[positive] / force_bw[positive]))
    np.testing.assert_allclose(normal_force, body_weight * force_bw, rtol=1e-12, atol=1e-12)

    output_files = []
    with PdfPages(OUT / "reward_exnovo_AB07_grafici.pdf") as pdf:
        def save(fig, name):
            for ext in ("png", "svg"):
                path = OUT / f"{name}.{ext}"
                fig.savefig(path, dpi=170, bbox_inches="tight")
                output_files.append(str(path.relative_to(ROOT)))
            pdf.savefig(fig, bbox_inches="tight")
            plt.close(fig)

        # The top curve is a duration-to-payout map, not a temporal payout trace.
        fig, axes = plt.subplots(2, 1, figsize=(11.5, 7.3), gridspec_kw={"height_ratios": [1.6, 1]})
        fig.subplots_adjust(hspace=.52, top=.86, bottom=.12)
        fig.suptitle("Toe-off support: reward amount and timing", fontsize=18, fontweight="bold")
        d = np.linspace(0, 1.65, 1200)
        w = window(d, cfg["contact_support_to_window_start_s"], cfg["contact_support_to_window_end_s"], cfg["phase_min_stance_duration_s"], cfg["phase_stance_timeout_s"])
        ax = axes[0]
        ax.axvspan(.79, 1.26, color=TEAL, alpha=.09)
        ax.plot(d, .3*w, color=BLUE, lw=3, label="Mean penetration quality = 1")
        ax.plot(d, .15*w, color=TEAL, lw=2, ls="--", label="Mean penetration quality = 0.5")
        for x in [.30, .79, 1.26, 1.45]:
            ax.axvline(x, color=GREY, lw=.8, ls=":")
        ax.set_xticks([0, .30, .545, .79, 1., 1.26, 1.45, 1.65])
        ax.set_ylim(-.005, .35)
        ax.set_xlim(0, 1.65)
        ax.set_title("Toe-off reward versus completed stance duration", loc="left")
        style(ax, "Stance duration d = TO time − HS time [s]", "Contribution 0.30 × T")
        ax.legend(loc="upper left", fontsize=10)
        ax = axes[1]
        ax.axvspan(0, 1., color=BLUE, alpha=.06)
        ax.axvspan(1., 1.6, color=TEAL, alpha=.06)
        ax.plot([0, 1., 1., 1.01, 1.01, 1.6], [0, 0, .3, .3, 0, 0], color=BLUE, lw=2.5)
        ax.scatter([1.], [.3], color=BLUE, zorder=5)
        ax.annotate("Accepted TO: +0.30 in one 10 ms step", xy=(1., .3), xytext=(.12, .23), arrowprops={"arrowstyle": "->", "color": GREY}, fontsize=11)
        ax.text(.47, .07, "STANCE", ha="center", color=BLUE)
        ax.text(1.32, .07, "SWING", ha="center", color=TEAL)
        ax.set_xlim(0, 1.6)
        ax.set_ylim(-.005, .36)
        ax.set_title("Ideal timing example: HS at 0 s, accepted TO at 1 s", loc="left")
        style(ax, "Time since HS [s]", "Awarded reward")
        fig.text(.10, .025, "TO support contribution before base clipping; other reward terms and penalties are excluded.", fontsize=11, color=GREY)
        save(fig, "01_supporto_TO")

        fig, axes = plt.subplots(3, 2, figsize=(13, 11.5))
        fig.subplots_adjust(hspace=.65, wspace=.26, top=.89, bottom=.13)
        fig.suptitle("Regularity: base contribution = 0.25 / (1 + 4 × total loss)", fontsize=18, fontweight="bold")
        fig.text(.5, .925, "Isolated curves: at least one valid cycle; all other losses set to zero", ha="center", color=GREY)
        def reg(loss):
            return .25 / (1 + 4 * loss)
        period = np.linspace(.60, 2.5, 800)
        loss_period = interval(period, 1.33, 1.83, .25) + interval(period, .9, 2.2, .25)
        ax = axes[0, 0]
        ax.axvspan(1.33, 1.83, color=TEAL, alpha=.12)
        ax.plot(period, reg(loss_period), color=BLUE, lw=2.5)
        for boundary in [.9, 2.2]:
            ax.axvline(boundary, color=GOLD, ls=":", lw=1.5)
        ax.set_xticks([.6, .9, 1.33, 1.83, 2.2, 2.5])
        ax.set_title("1 · Duration of the latest HS → HS cycle", loc="left")
        style(ax, "Period T [s]; previous period equals T", "Regularity contribution")
        ax = axes[0, 1]
        delta = np.linspace(-.8, .8, 800)
        ax.plot(delta, reg(.5*huber(delta/.2)), color=BLUE, lw=2.5)
        ax.axvline(0, color=GREY, ls=":")
        ax.scatter([-.2, .2], [.125, .125], color=TEAL, zorder=4)
        ax.set_title("2 · Similarity of consecutive periods", loc="left")
        style(ax, "T − previous T [s]", "Regularity contribution")
        ax = axes[1, 0]
        fraction = np.linspace(.1, 1., 800)
        ax.axvspan(50, 80, color=TEAL, alpha=.12)
        ax.plot(100*fraction, reg(.5*interval(fraction, .5, .8, .15)), color=BLUE, lw=2.5)
        ax.set_title("3 · Fraction of the cycle spent in stance", loc="left")
        style(ax, "Stance duration / cycle duration [%]", "Regularity contribution")
        ax = axes[1, 1]
        elapsed = np.linspace(0, 2.6, 800)
        ax.plot(elapsed, reg(.5*interval(elapsed, 0, 1.45, .2)), color=BLUE, lw=2.5, label="Stance: 1.45 s threshold")
        ax.plot(elapsed, reg(.5*interval(elapsed, 0, .75, .2)), color=TEAL, lw=2.5, label="Swing: 0.75 s threshold")
        ax.set_title("4 · Time elapsed in the current phase", loc="left")
        style(ax, "Time in phase [s]", "Regularity contribution")
        ax.legend(fontsize=10, loc="lower left")
        ax = axes[2, 0]
        z = np.linspace(0, 3., 600)
        ax.plot(z, huber(z), color=RED, lw=2.5, label="Huber loss used by the reward")
        ax.plot(z, .5*z*z, color=GREY, ls="--", label="Quadratic loss for comparison")
        ax.axvline(1, color=GREY, ls=":")
        ax.set_title("5 · From deviation to Huber loss", loc="left")
        style(ax, "Deviation / scale [dimensionless]", "Loss")
        ax.legend(fontsize=10, loc="upper left")
        ax = axes[2, 1]
        loss_total = np.linspace(0, 3., 600)
        ax.plot(loss_total, reg(loss_total), color=BLUE, lw=2.5)
        ax.scatter([0, .25, .5, 1], reg(np.array([0, .25, .5, 1])), color=TEAL, zorder=5)
        ax.annotate("L = 0 → 0.25", xy=(0, .25), xytext=(.55, .225), arrowprops={"arrowstyle":"->", "color":GREY})
        ax.annotate("L = 1 → 0.05", xy=(1, .05), xytext=(1.3, .13), arrowprops={"arrowstyle":"->", "color":GREY})
        ax.set_title("6 · From total loss to reward", loc="left")
        style(ax, "Total loss L_reg", "Regularity contribution")
        for ax in [*axes[0], *axes[1], axes[2, 1]]:
            ax.set_ylim(0, .275)
            ax.set_yticks([0, .05, .125, .20, .25])
        fig.text(.08, .018, "The contribution is zero before the first valid cycle. The separate timeout cost and other reward terms are omitted.\nThese are mathematical responses of individual terms; no new simulation was run.", fontsize=11, color=GREY)
        save(fig, "02_regolarita")

        fig, axes = plt.subplots(1, 2, figsize=(13, 5.4), gridspec_kw={"width_ratios": [1.6, 1]})
        fig.subplots_adjust(wspace=.30, top=.80, bottom=.23)
        fig.suptitle("Morphology cost: illustrative recorded episode", fontsize=18, fontweight="bold")
        ax = axes[0]
        ax.plot(t, np.cumsum(reward + morphology), color=BLUE, lw=2.5, label="Same trajectory with morphology cost removed")
        ax.plot(t, np.cumsum(reward), color=RED, lw=2.5, label="Recorded return")
        ax.fill_between(t, np.cumsum(reward), np.cumsum(reward+morphology), color=GOLD, alpha=.12)
        ax.axhline(0, color=GREY, lw=.8)
        ax.set_xlim(0, 6.11)
        ax.legend(fontsize=10, loc="upper left")
        ax.annotate(f"{(reward+morphology).sum():+.2f}", xy=(t[-1], (reward+morphology).sum()), xytext=(-5, 8), textcoords="offset points", ha="right", color=BLUE, fontweight="bold")
        ax.annotate(f"{reward.sum():+.2f}", xy=(t[-1], reward.sum()), xytext=(-5, -32), textcoords="offset points", ha="right", color=RED, fontweight="bold")
        style(ax, "Time since episode start [s]", "Cumulative reward")
        ax = axes[1]
        bars = ax.bar([0, 1], [knee.sum(), ankle.sum()], color=[BLUE, TEAL], width=.58)
        ax.set_xticks([0, 1], ["Knee", "Ankle"])
        ax.set_ylim(0, 74)
        style(ax, "", "Reward points subtracted")
        for bar, value in zip(bars, [knee.sum(), ankle.sum()]):
            ax.text(bar.get_x()+bar.get_width()/2, value+2, f"{value:.2f}\n{100*value/morphology.sum():.1f}%", ha="center", fontweight="bold")
        ax.set_title(f"Total cost: {morphology.sum():.2f}", loc="left")
        fig.text(.08, .07, f"Recalculated over {len(rows)} saved steps, with the same actions and states.\nThis does not predict training without morphology; time denotes reward emission.", color=GREY, fontsize=11)
        save(fig, "03_impatto_morphology")

        fig, axes = plt.subplots(1, 2, figsize=(13, 5.2))
        fig.subplots_adjust(wspace=.30, top=.80, bottom=.24)
        fig.suptitle("Landing reward and load: illustrative recorded episode", fontsize=18, fontweight="bold")
        f = np.linspace(0, 1.5, 500)
        ax = axes[0]
        ax.plot(f, .25*np.clip(f/.65, 0, 1), color=TEAL, lw=3)
        ax.axvline(.65, color=GREY, ls=":")
        ax.set_xticks([0, .325, .65, 1, 1.5])
        ax.set_ylim(0, .29)
        ax.set_title("Landing: full credit at 0.65 BW", loc="left")
        style(ax, "Normal load / body weight: F_n / BW", "Contribution 0.25 × A")
        ax.text(.72, .18, "More load does not\nincrease this reward", fontsize=11)
        ax = axes[1]
        ax.plot(t, normal_force, color=BLUE, lw=1.8, label="Online left normal force F_n(t)")
        ax.axhline(body_weight, color=RED, ls="--", label=f"Constant BW = {body_weight:.1f} N")
        ax.axhline(.65*body_weight, color=TEAL, ls=":", label=f"0.65 BW = {.65*body_weight:.1f} N")
        ax.set_xlim(0, 6.11)
        ax.set_title("Signals from the illustrative recorded episode", loc="left")
        style(ax, "Time since episode start [s]", "Force [N]")
        ax.legend(fontsize=9, loc="upper right")
        fig.text(.08, .07, "Landing curve applies only during contact within the 0.35–0.85 s swing window.\nBW = total OpenSim model mass × 9.80665 m/s²; online force varies during the episode.", fontsize=11, color=GREY)
        save(fig, "04_carico_e_landing")

    metrics = {
        "scope": "Saved-trajectory arithmetic; no new rollout or training",
        "episode": RUN_NAME,
        "steps": len(rows),
        "return_actual": float(reward.sum()),
        "return_same_trajectory_without_morphology": float((reward+morphology).sum()),
        "morphology_cost_total": float(morphology.sum()),
        "morphology_cost_knee": float(knee.sum()),
        "morphology_cost_ankle": float(ankle.sum()),
        "morphology_ankle_fraction": float(ankle.sum()/morphology.sum()),
        "morphology_penalized_steps": int(np.count_nonzero(morphology > 0)),
        "morphology_evaluated_samples": len(samples),
        "morphology_outside_samples": sum(s["knee_loss"] + s["ankle_loss"] > 0 for s in samples),
        "morphology_mean_per_step": float(morphology.mean()),
        "morphology_max_per_step": float(morphology.max()),
        "body_weight_n": body_weight,
        "body_mass_kg": body_weight / 9.80665,
        "landing_full_credit_force_n": .65 * body_weight,
        "source_files": {
            str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (SOURCE, RECEIPT, diagnostic_path)
        },
        "plots": output_files,
        "combined_pdf": str((OUT / "reward_exnovo_AB07_grafici.pdf").relative_to(ROOT)),
    }
    (AUDIT / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    with (AUDIT / "reward_timeseries.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["time_from_start_s", "reward_actual", "reward_plus_morphology", "morphology_knee", "morphology_ankle", "force_normal_n", "force_normal_bw"])
        writer.writerows(zip(t, reward, reward+morphology, knee, ankle, normal_force, force_bw))
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
