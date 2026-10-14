"""Build the English reward and actor--critic observation-space document.

Requires NumPy, Matplotlib and pdflatex on PATH. Paths use pathlib for Windows
and macOS. Saved telemetry is used only for appendix illustrations; no OpenSim,
Ray, checkpoint, rollout or training is loaded or executed.
"""
from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

import observation_tables
import render_corridor_example
import render_reward_analysis as plots

AUDIT, ROOT, OUT = plots.AUDIT, plots.ROOT, plots.OUT


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    compiler = shutil.which("pdflatex")
    if not compiler:
        raise RuntimeError("pdflatex must be available on PATH")

    # Regenerate every embedded figure so that labels match the English text.
    plots.main()
    render_corridor_example.main()
    observation_tables.write_tables(receipt_path=plots.RECEIPT)

    template_path = AUDIT / "reward_exnovo_AB07.tex"
    rendered = template_path.read_text(encoding="utf-8").replace(
        "@@GRAPHICS@@", OUT.as_posix()
    )
    destination = OUT / "reward_exnovo_AB07.pdf"
    table_names = ("actor_features.tex", "critic_features.tex")
    with tempfile.TemporaryDirectory(prefix="cmc_reward_document_") as temp:
        work = Path(temp)
        (work / template_path.name).write_text(rendered, encoding="utf-8")
        for name in table_names:
            shutil.copy2(AUDIT / name, work / name)
        for _ in range(2):
            result = subprocess.run(
                [compiler, "-interaction=nonstopmode", "-halt-on-error",
                 "-no-shell-escape", template_path.name],
                cwd=work, text=True, capture_output=True,
            )
            if result.returncode:
                (AUDIT / "pdf_build_failed.log").write_text(
                    result.stdout + result.stderr, encoding="utf-8"
                )
                raise RuntimeError(result.stdout[-5000:])
        log = (work / "reward_exnovo_AB07.log").read_text(
            encoding="utf-8", errors="replace"
        )
        (AUDIT / "pdf_build.log").write_text(log, encoding="utf-8")
        warnings = [
            line for line in log.splitlines()
            if any(term in line for term in (
                "Overfull", "Underfull", "Warning", "Missing character", "Output written"
            ))
        ]
        shutil.copy2(work / destination.name, destination)

    generated_sources = [
        template_path, Path(__file__),
        AUDIT / "observation_tables.py", AUDIT / "render_reward_analysis.py",
        AUDIT / "render_corridor_example.py",
        *(AUDIT / name for name in table_names),
    ]
    receipt = {
        "pdf": str(destination.relative_to(ROOT)),
        "pdf_sha256": sha256(destination),
        "template_sha256": sha256(template_path),
        "compiler": Path(compiler).name,
        "language": "English",
        "scope": "General reward and observation contract; recorded examples in appendix only",
        "warnings": warnings,
        "actor_feature_count": 35,
        "critic_feature_count": 84,
        "critic_additional_feature_count": 49,
        "source_hashes": {str(p.relative_to(ROOT)): sha256(p) for p in generated_sources},
        "figure_hashes": {
            p.name: sha256(p) for p in sorted(OUT.glob("0[1-5]_*.png"))
        },
        "new_simulation_or_training": False,
    }
    (AUDIT / "pdf_build_receipt.json").write_text(
        json.dumps(receipt, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    main()
