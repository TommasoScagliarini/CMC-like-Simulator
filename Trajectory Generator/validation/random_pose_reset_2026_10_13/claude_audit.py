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

SOURCE_PROMPT = """Audit indipendente richiesto dall'utente il13ottobre: verificare la capacita
V26C best47 di gestire reset da pose CASUALI DI KNEE/ANKLE PROTESICI. Il resto
biologico rimane inizializzato e guidato dalla IK; non e avvio autonomo da fermo.
Usa italiano e cita file/funzioni/righe. Max1500 parole output. Opus5.5 effortmax.

Questo e audit indipendente di sorgenti, progetto esperimento e possibili
confondenti. Non leggere conclusioni Codex, source_protocol_audit, analysis_plan,
report o risultati nuovi nella cartella random_pose_reset_2026_10_13. Puoi leggere
SOLTANTO il nuovo harness random_pose_reset.py e bootstrap_probe.py, i sorgenti
operativi originali elencati sotto e artifacts del12/10 per capire il confronto
coerente vs legacy. Non eseguire comandi, simulazioni o training, non modificare
file, non usare servizi esterni; tools Read/Grep/Glob soltanto.

Disegno da auditare, non assumere corretto: checkpoint exnovo35/84 best47 frozen,
AB06, tempo13.946870983805102s, biologia/GRFdestra originali, physicalGRFsinistra
online. Jointposes IIDuniformi qknee[-1.20,-0.03]rad, qankle[-.55,+.55]rad;
seed20261013, pool400, first20 ammessi dopo screeningGEOMETRIA prima dirollout,
5s deterministicseed123 ciascuno. Gateinitialjointbounds/finiteFK e contatti
primariLEFT maxpenetration<=27mm (operational28mm resta invariato), normforceLEFT
<=5BW comeguardstandard. AIRammesso senza limite arbitrario clearance; non e
certifica statico equilibrio o selfcollision. Tutti geometricrejections e poi
physicalfailures salvati; nessun failure sostituito. Control coherentnominal
qIKnom ma qdotprosth0, motorq=q/ω0, controllerPI0, prevendpointq, served/governor
q0/vel0/acc0. Randomcases stessa coerenza. FSMprime dalla nuova geometria,
nessunHS sintetico. Loadobs iniziale segue contratto originale0fino primasample.

Esamina se il bootstrap davvero NONreintroduceIKprotesica al firstaction o
segmento: _initialise_episode/kin.clear_segment/get/_action_to_segment/filterC2,
state SEA/primi sensorpoints/FSM. Verifica come preservare eventi e tracce nei
failure fail_fast, censure ultimo segmento nonritornato, forcevsdetectorplane.
Definisci criteri per distinguere semplicecompletion da recupero gait stabile.
Proposta primaria:500steps/5s,>=2cicli,firstHS<=3s,0invalid/resync,
pen1ms<28mm,post0.5actualservedRMSE<=2deg,escursioni cicliche dichiarate;
clipping/forze/reserve/timing separati. Critica selezione bias, campionamento,
coerenza bootstrap e limiti inferenze daN20. Quali risultati permettono davvero
concludere gestisceposecasuali nel dominio? Cosa non sarebbe dimostrato?

Fonti mirate:
AGENTS.md; simulation_runner.py (reset_to_time,_init_muscle_states,_sample_online_grf);
Trajectory Generator/osim_trj_cmc_like.py (ProstheticSegmentKinematics,reset,
_initialise_episode,_action_to_segment,_get_observation,primeFSM);
Trajectory Generator/binary_phase_adapter.py; Trajectory Generator/prosthetic_phase_fsm.py;
online_grf.py; Trajectory Generator/baseline_MLP/rollout_eval.py;
Trajectory Generator/validation/random_pose_reset_2026_10_13/random_pose_reset.py;
Trajectory Generator/validation/random_pose_reset_2026_10_13/bootstrap_probe.py;
Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/training_cfg.resolved.yaml;
solo reset receipt del12/10 se necessario. NON leggere interi traininglogs o
dailyreports: non aiutano questa verifica di randompose e aumentano contesto.
Non vedere la proposta Codex per formulare un tuo verdetto indipendente.
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
