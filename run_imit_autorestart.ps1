# Auto-restart wrapper for the imitation pre-training (created 2026-06-11 ~08:56).
# Resumes from checkpoint_last after any non-completion exit (e.g. the sporadic
# Ray "access violation" native crash that killed the first run at iter 14), until
# logical_iteration reaches the target or attempts are exhausted. The training's
# own supervisor handles iteration-timeout skips; this wrapper handles hard crashes
# that the supervisor does not auto-restart. Independent of agent notification
# latency.
$ErrorActionPreference = "Continue"
$repo = "c:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude"
Set-Location $repo
$conda = "C:\Users\tomma\anaconda3\Scripts\conda.exe"
$runDir = Join-Path $repo "Trajectory Generator\runs\baseline_mlp_imit_win"
$metaFile = Join-Path $runDir "checkpoint_last_meta.json"
$summaryFile = Join-Path $runDir "summary.json"
$rewardJson = Join-Path $repo "Trajectory Generator\baseline_MLP\reward_imitation.json"
$log = Join-Path $repo "imit_autorestart.log"
$TARGET = 40
$MAXATT = 15
$workers = 12
$lastIter = -1
$stuck = 0
$keep = @(24484, 33136)  # pre-existing TensorBoard pythons (do not kill)

function Log($m) {
  $line = "$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')  $m"
  Write-Host $line -ForegroundColor Cyan
  Add-Content -Path $log -Value $line
}
function CurIter {
  if (Test-Path $metaFile) { try { return [int](Get-Content $metaFile -Raw | ConvertFrom-Json).logical_iteration } catch { return -1 } }
  return -1
}

Log "=== AUTO-RESTART WRAPPER START (target=$TARGET iter) ==="
for ($att = 1; $att -le $MAXATT; $att++) {
  $ci = CurIter
  if ($ci -ge $TARGET) { Log "Target already reached (logical_iteration=$ci)."; break }
  Log "Attempt $att/$MAXATT  (resume from iter $ci, workers=$workers)"
  & $conda run --no-capture-output -n envCMC-rllib python "Trajectory Generator\baseline_MLP\train_ppo_mlp.py" `
    --resume-from "runs\baseline_mlp_imit_win\checkpoint_last" `
    --output-dir "runs\baseline_mlp_imit_win" `
    --setup-xml "models\AB06_SEASEA_Threadmill\AB06_SEASEA_stiff321_500_pi_setup.xml" `
    --reward-mode imitation --reward-json $rewardJson `
    --grf-mode online_sensor --online-grf-applied-side left `
    --num-env-runners $workers --ray-num-cpus ($workers + 1) --iterations $TARGET `
    --train-batch-size 4096 --minibatch-size 512 --num-epochs 10 `
    --episode-duration 2.0 --segment-duration 0.01 --fcnet-hiddens 256 256 --seed 123 `
    --checkpoint-every 1 --max-consecutive-skips 5 `
    --startup-timeout-s 600 --iteration-timeout-s 3600 --sample-timeout-s 3000 `
    --step-wall-timeout-s 60 --checkpoint-timeout-s 240 --cleanup-timeout-s 60
  $ec = $LASTEXITCODE
  $ni = CurIter
  $stop = ""
  if (Test-Path $summaryFile) { try { $stop = (Get-Content $summaryFile -Raw | ConvertFrom-Json).stop_reason } catch {} }
  Log "Attempt $att ended: exit=$ec stop_reason='$stop' logical_iteration=$ni"
  if ($ni -ge $TARGET -or $stop -eq "completed") { Log "COMPLETED at iter $ni."; break }
  if ($ni -le $lastIter) { $stuck++ } else { $stuck = 0 }
  $lastIter = $ni
  if ($stuck -ge 5 -and $workers -gt 6) { $workers = 6; Log "No progress x$stuck -> reduce workers to $workers" }
  elseif ($stuck -ge 3 -and $workers -gt 8) { $workers = 8; Log "No progress x$stuck -> reduce workers to $workers" }
  Log "Cleaning orphan Ray/python before relaunch ..."
  Get-Process python, raylet, gcs_server, plasma_store -ErrorAction SilentlyContinue | Where-Object { $keep -notcontains $_.Id } | Stop-Process -Force -ErrorAction SilentlyContinue
  Start-Sleep -Seconds 15
}
$final = CurIter
Log "=== WRAPPER END. final logical_iteration=$final ==="
