"""Render canonical figures in an explicit repository-root per-rollout folder."""
from __future__ import annotations
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
PACKAGE=HERE.parents[2]
REPO=PACKAGE.parents[1]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--rollout",type=Path,required=True)
    args=parser.parse_args()
    folder=args.rollout.resolve()
    summary=json.loads((folder/"rollout_summary.json").read_text())
    out=REPO/"plot/09_29_2026_general_method_diagnosis"/folder.name
    out.mkdir(parents=True,exist_ok=True)
    spec=importlib.util.spec_from_file_location("diagnostic_canonical_plotter", REPO/"plot/plotter.py")
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    context=module.valid_mlp_rollout_context(folder)
    if context is None:raise ValueError("Invalid recorded rollout context")
    source=summary["diagnostic_intervention"]["source"]
    sequence=summary["diagnostic_intervention"].get("mode")=="sequence"
    if sequence:
        # Append only descriptive metadata after the worker's final receipt.
        # The recorded command-minus-mean diagnostic is deterministic here.
        if not (folder/"diagnostic_receipt.json").exists():
            raise ValueError("Sequence worker must finish before annotated rendering")
        summary["diagnostic_intervention"]["sequence_metadata"]={
            "applied_controller":"offline target-informed endpoint sequence",
            "learned_policy_applied":False,
            "actor_evaluated_for_diagnostics_only":True,
            "future_target_available_offline":True,
            "random_exploration_applied":False,
            "raw_policy_action_semantics":"deterministic substituted sequence command, not learned actor output",
            "exploration_action_noise_semantics":"deterministic sequence command minus frozen actor mean; not sampled noise",
            "exploration_noise_summary_semantics":"summary noise statistics describe the deterministic intervention relative to actor mean, not exploration",
        }
        (folder/"rollout_summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    from matplotlib.text import Text
    original=module.save_figure
    saved=[]
    def save(fig,out_dir,filename):
        if filename.startswith("07_") and source=="LEFT":
            for label in fig.findobj(match=Text):
                text=label.get_text().replace("Shifted Sound Leg Target","Prescribed Left Phase Target").replace("shifted sound leg target","prescribed left phase target").replace("shifted sound leg - policy C2","left phase target - policy C2")
                label.set_text(text)
        if sequence:
            for label in fig.findobj(match=Text):
                text=label.get_text().replace("MLP C2-Filtered Policy","C2-Filtered Offline Sequence")
                text=text.replace("policy C2","sequence C2").replace("policy trajectory","offline sequence trajectory")
                text=text.replace("raw policy action","offline sequence command")
                label.set_text(text)
            # Reserve a separate top line without obscuring the canonical
            # controller metadata or changing the plotted scientific data.
            if fig._suptitle is not None:
                fig._suptitle.set_y(.974)
                fig._suptitle.set_fontsize(12)
            for label in fig.texts:
                if label is not fig._suptitle and label.get_position()[1]>.90:
                    label.set_y(label.get_position()[1]-.022)
            fig.text(.5,.997,
                     "Offline target-informed endpoint sequence — diagnostic, not learned policy",
                     ha="center",va="top",fontsize=11,color="#7a321b",weight="bold")
        original(fig,out_dir,filename);saved.append(out_dir/filename)
    module.save_figure=save
    module.next_output_dir=lambda *_:out
    module.latest_mlp_rollout_context=lambda:context
    sys.argv=[str(REPO/"plot/plotter.py"),"--mlp","--results-dir",str(context.sim_outputs_dir),"--prefix","rollout_episode","--setup",str(summary["setup_xml_path"]),"--gait-side","left","--out-root",str(REPO/"plot")]
    module.main()
    if len(saved)!=7:raise RuntimeError(f"Expected seven figures, got {len(saved)}")
    receipt={"rollout":str(folder.relative_to(REPO)),"source":source,"diagnostic_intervention":summary["diagnostic_intervention"],"figures":{str(p.relative_to(REPO)):hashlib.sha256(p.read_bytes()).hexdigest() for p in saved}}
    (out/"plot_receipt.json").write_text(json.dumps(receipt,indent=2)+"\n")
    print(str(out))


if __name__=="__main__":main()
