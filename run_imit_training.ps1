# Imitation pre-training launcher (visible window). Created 2026-06-10 22:34.
# The prosthesis imitates the sound leg anti-phase (--reward-mode imitation).
# Checkpoints are saved every iteration to runs\baseline_mlp_imit_win, so a usable
# seed exists even if the run is stopped early.
$ErrorActionPreference = "Continue"
Set-Location "c:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude"
try { Start-Transcript -Path "c:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\imit_training_console.log" -Append | Out-Null } catch {}
Write-Host "=== IMITATION TRAINING START $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss') ===" -ForegroundColor Cyan
& C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-rllib python "Trajectory Generator\baseline_MLP\train_ppo_mlp.py" `
  --setup-xml "models\AB06_SEASEA_Threadmill\AB06_SEASEA_stiff321_500_pi_setup.xml" `
  --output-dir "runs\baseline_mlp_imit_win" `
  --reward-mode imitation `
  --reward-json "c:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude\Trajectory Generator\baseline_MLP\reward_imitation.json" `
  --grf-mode online_sensor --online-grf-applied-side left `
  --num-env-runners 12 --ray-num-cpus 13 --iterations 40 `
  --train-batch-size 4096 --minibatch-size 512 --num-epochs 10 `
  --episode-duration 2.0 --segment-duration 0.01 --fcnet-hiddens 256 256 --seed 123 `
  --checkpoint-every 1 --max-consecutive-skips 5 `
  --startup-timeout-s 600 --iteration-timeout-s 3600 --sample-timeout-s 3000 `
  --step-wall-timeout-s 60 --checkpoint-timeout-s 240 --cleanup-timeout-s 60
Write-Host ""
Write-Host "=== IMITATION TRAINING ENDED $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss') (exit $LASTEXITCODE) ===" -ForegroundColor Cyan
Write-Host "Window kept open for inspection." -ForegroundColor Yellow
try { Stop-Transcript | Out-Null } catch {}
