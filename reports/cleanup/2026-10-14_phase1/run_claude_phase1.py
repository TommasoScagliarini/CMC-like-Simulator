"""Focused read-only review of the executed first cleanup, explicitly requested."""
from pathlib import Path
import datetime
import hashlib
import json
import shutil
import subprocess

ROOT = Path('/Users/tommy/Documents/CMC-like-Simulator - Claude')
OUT = Path('/tmp/cmc_cleanup_phase1_2026_10_14')
MODEL = 'claude-opus-5-5'
PROMPT = '''Rispondi in italiano. Sei il revisore indipendente Claude Opus 5.5,
effort max, richiesto dall'utente per questa PRIMA PULIZIA della repository.
Hai gia contribuito all'analisi generale precedente, ora valuta l'esecuzione.
Non modificare file, non eseguire comandi, non avviare training o simulazioni.
Usa soltanto Read, Grep, Glob. Output finale circa 900 parole, concreto.

Contesto: simulatore OpenSim/CMC-like con due SEA. Leggi AGENTS.md per le
invarianti (nessuna modifica C++/semantica SEA autorizzata in questa fase).
L'utente ha dichiarato di avere copiato TUTTA la cartella su un disco esterno
e ha autorizzato la prima pulizia. Non occorre chiedere ancora il backup.
Il commit HEAD resta dba42388c32a441a5543f20881612a52781d7a99;31 cancellazioni
Prosthesis_SNN erano gia presenti PRIMA, estranee a questo intervento.

Evidenze disponibili in /tmp/cmc_cleanup_phase1_2026_10_14 (cartella consentita):
- preflight_summary.json, removal_review.md: classificazione e riferimenti;
- action_manifest.json, deletion_journal.jsonl, cleanup_result.json:
  rimozione puntuale2158 file135728993 B,86 tracciati generati,103 dirs vuote;
- expected_gitignore.json e .gitignore attuale: unica modifica a un file
  mantenuto, ignore ancorati /build/, /build_online_grf/ e5 intermedi LaTeX;
- whole_worktree_after_check.json: SHA256 di151319 file mantenuti prima/dopo,
  44636575150 B,151318 identici, unica variazione .gitignore attesa; nessun
  altro file perso/aggiunto/cambiato, nessun target rigenerato. Non leggere
  il voluminoso whole_worktree_before.json salvo bisogno specifico;
- protected_paths_integrity_baseline.json:86 ancore modello/plugin/profili/
  sorgenti/checkpoint; tre .DS_Store conservati per provenance; verifica
  ancore postpulizia PASS, e tutti coperti dal confronto completo SHA256;
- bundle_drift_diagnostic.json:1357 file in_place conservati identici alla
  baseline pre-cleanup,1356 matching al manifest agosto, uno drift:
  v26c_j1_collect.py atteso55428B, attuale55320B, uguale a HEAD. Verifica
  archivio objects/controls positiva,692objects257138708B,1888source entries;
- bundle_structural_suite.log e bundle_structural_result.json: suite19 test,
  17PASS,1FAIL source closure per3 file aggiunti PRIMA della pulizia e1ERROR
  stesso driftJ1. Le3 aggiunte hanno SHA identici alla baseline;
- checkpoint_reload_result.json, checkpoint_reload.log, reload_best47.py:
  RLModule best47 ricaricato, inferenza finita, actor35/full84/action2,
  inferenze prefix35/full84 uguali; Python3.10/Torch2.10/Ray2.55;
- native_smoke/rollout_summary.json e rollout_policy_trace.json: macOS arm64,
  OpenSim/plugin SEA+GRF caricati, best47,2 segmenti fisici da0.01s,
  max_steps2, oktrue, residuo ID massimo2.64e-13, nessuna uscita STO o plot.
  È uno smoke di caricamento/reset/integrazione breve, non una replica
  scientifica best47 completa5s o qualifica FSMv3. Non è prova Windows.

Controlla con Read/Grep le evidenze e pochi sorgenti/manifests rilevanti se
servono a interpretare i risultati. Nessuna analisi generale ripetuta.
Valuta: 1) eventuali rischi concreti di cancellazione di dipendenze o binari
vincolati;2) qualità del confronto byte prima/dopo e dei test;3) attribuzione
dei problemi V26C già presenti, senza riscrivere vecchi manifest o annullare
modifiche successive legittime;4) formulazione onesta dell'integrità verificata
e limiti;5) se questa prima pulizia è completa o richiede una correzione.
Se non hai evidenza per una conclusione, segnala il limite. Non suggerire
nuove cancellazioni di dati/campagne come condizione per completare questa.
'''

def main():
    prompt_path = OUT / 'claude_phase1_input.txt'
    prompt_path.write_text(PROMPT)
    cli = shutil.which('claude')
    if not cli:
        raise RuntimeError('Claude CLI unavailable; no fallback permitted')
    command = [cli, '--print', '--model', MODEL, '--effort', 'max',
               '--restricted', '--tools', 'Read,Grep,Glob',
               '--allowedTools', 'Read,Grep,Glob', '--disable-slash-commands',
               '--strict-mcp-config', '--mcp-config', '{"mcpServers":{}}',
               '--setting-sources', '', '--no-session-persistence',
               '--output-format', 'json', '--add-dir', str(OUT)]
    receipt = dict(phase='executed_cleanup_review', model_requested=MODEL,
                   effort_requested='max', fallback_allowed=False,
                   command=command, cwd=str(ROOT),
                   input_sha256=hashlib.sha256(PROMPT.encode()).hexdigest(),
                   started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    receipt_path = OUT / 'claude_phase1_receipt.json'
    receipt_path.write_text(json.dumps(receipt, indent=2) + '\n')
    with (OUT / 'claude_phase1_response.json').open('w') as out, (OUT / 'claude_phase1_stderr.log').open('w') as err:
        result = subprocess.run(command, input=PROMPT, text=True, cwd=ROOT,
                                stdout=out, stderr=err, timeout=1800)
    receipt.update(returncode=result.returncode,
                   completed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    try:
        response = json.loads((OUT / 'claude_phase1_response.json').read_text())
        receipt.update(model_usage=response.get('modelUsage'),
                       usage=response.get('usage'), is_error=response.get('is_error'))
        (OUT / 'claude_phase1_response.md').write_text(str(response.get('result', response)) + '\n')
    except (ValueError, OSError):
        receipt['response_json_parse_failed'] = True
    receipt_path.write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, ensure_ascii=False), flush=True)
    return result.returncode

if __name__ == '__main__':
    raise SystemExit(main())
