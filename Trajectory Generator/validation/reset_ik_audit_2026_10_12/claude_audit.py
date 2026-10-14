"""Run the explicitly requested independent Claude audit and comparison.

All arguments are passed directly to subprocess; Claude has read-only tools.
No fallback model is permitted. Inputs, responses and invocation are retained.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path

AUDIT = Path(__file__).resolve().parent
ROOT = AUDIT.parents[2]
MODEL = "claude-opus-5-5"

SOURCE_PROMPT = """L'utente richiede un audit indipendente del reset IK del simulatore
OpenSim/Python CMC-like e della policy V26 ex-novo. Lavora in italiano.
Ipotesi da verificare, senza assumerla vera: ad ogni reset q e qdot vengono
inizializzati dall'IK; sarebbe l'unica informazione prescribed usata nella rete
ex-novo e la ripetizione della stessa posa potrebbe limitare l'apprendimento.

Leggi direttamente il codice e gli artefatti storici originali nel repository.
NON leggere risultati, conclusioni o report Codex in reset_ik_audit_2026_10_12,
reports/user del 12 ottobre o altri audit di questa nuova indagine. Non modificare
file, non lanciare simulazioni/training, non usare servizi esterni. Tools solo
Read/Grep/Glob. Questo audit e' una verifica indipendente delle sorgenti e dei
dati storici, non una nuova esecuzione OpenSim.

Fonti principali:
- AGENTS.md
- Trajectory Generator/osim_trj_cmc_like.py
- simulation_runner.py
- Trajectory Generator/baseline_MLP/env_factory.py
- Trajectory Generator/baseline_MLP/asymmetric_rl_module.py
- Trajectory Generator/baseline_MLP/train_ppo_mlp.py
- Trajectory Generator/baseline_MLP/reward_function.py
- Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/training_cfg.resolved.yaml
- nello stesso run: summary.json e train_iterations.jsonl, solo i campi rilevanti.

Restituisci: (1) percorso effettivo q/qdot, SEA, governor e FSM al reset;
(2) starts realmente configurati/usati e se sono uguali per tutti gli episodi
o per ciascun runner; (3) separazione actor35/critic84, segnali prescribed
diretti, indiretti e supporto biologico/GRF/reward;
(4) cosa e' gia dimostrato e cosa richiede un esperimento causale di training;
(5) test controllati minimi e possibili confondenti. Cita file e righe o campi
JSON esatti. Non equiparare assenza di target actor con autonomia dell'intero
plant. Non interpretare un frozen-policy test come effetto causale sulla
qualita dell'apprendimento. Non assumere che zero velocita in una posa IK di
cammino equivalga a una posa statica bilanciata. Esponi dubbi e controevidenze.
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("phase", choices=["source", "comparison"])
    args = parser.parse_args()
    folder = AUDIT / "claude"
    folder.mkdir(parents=True, exist_ok=True)
    if args.phase == "source":
        prompt = SOURCE_PROMPT
    else:
        prompt = (folder / "comparison_prompt.txt").read_text(encoding="utf-8")
    input_path = folder / f"{args.phase}_input.txt"
    input_path.write_text(prompt, encoding="utf-8")
    cli = shutil.which("claude")
    if not cli:
        raise RuntimeError("Claude CLI unavailable")
    allowed = "Read,Grep,Glob" if args.phase == "source" else ""
    command = [cli, "--print", "--model", MODEL, "--effort", "max",
               "--restricted", "--tools", allowed,
               "--allowedTools", allowed, "--disable-slash-commands",
               "--strict-mcp-config", "--mcp-config", '{"mcpServers":{}}',
               "--setting-sources", "", "--no-session-persistence",
               "--output-format", "json"]
    receipt = {"phase": args.phase, "model_requested": MODEL,
               "effort_requested": "max", "fallback_allowed": False,
               "command": command, "cwd": str(ROOT),
               "input_sha256": hashlib.sha256(prompt.encode()).hexdigest(),
               "started_utc": datetime.now(timezone.utc).isoformat(),
               "source_independence": args.phase == "source"}
    receipt_path = folder / f"{args.phase}_receipt.json"
    receipt_path.write_text(json.dumps(receipt, indent=2) + "\n")
    with (folder / f"{args.phase}_response.json").open("w", encoding="utf-8") as out, (folder / f"{args.phase}_stderr.log").open("w", encoding="utf-8") as err:
        completed = subprocess.run(command, input=prompt, text=True, cwd=ROOT,
                                   stdout=out, stderr=err, timeout=1800)
    receipt.update(returncode=completed.returncode,
                   completed_utc=datetime.now(timezone.utc).isoformat())
    try:
        response = json.loads((folder / f"{args.phase}_response.json").read_text())
        receipt["model_usage"] = response.get("modelUsage")
        receipt["is_error"] = response.get("is_error")
        receipt["usage"] = response.get("usage")
        (folder / f"{args.phase}_response.md").write_text(str(response.get("result", response)) + "\n", encoding="utf-8")
    except (ValueError, OSError):
        receipt["response_json_parse_failed"] = True
    receipt_path.write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, ensure_ascii=False))
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
