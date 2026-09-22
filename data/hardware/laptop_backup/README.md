# Laptop backup

Hardware logs from the deployment laptop, copied on 2026-09-22 from its backup
(`Sim_to_real_spacerobot_kinova` on drive D). Only files without an identical copy elsewhere in this repo were
copied. File names and time stamps are unchanged.

Deployment laptop: HP Pavilion x360 14-dy0xxx, Intel Core i5-1135G7 (4 cores, up to 4.2 GHz), 16 GB RAM, Windows,
MATLAB R2025b Update 3. The logs name it `Patricks-PC`. Training and simulation ran on a separate desktop PC.

## Numbering

The laptop numbered its runs independently of `../runs/`.

- `../runs/run_070` to `run_078` are identical to the laptop files with the same names.
- `../runs/run_068_agent_p2p_seed1.mat` and `run_069_agent_p2p_seed1.mat` are the laptop files
  `a_test_run_p2p_0-5_1.mat` and `a_test_run_p2p_0-5_2.mat` from 2026-06-18.
- `runs/run_068_agent_p2p_seed1.mat` and `runs/run_069_agent_p2p_seed1.mat` in this folder are different runs from
  2026-05-27.
- `runs/run_079` to `run_100` here are set-point runs from 2026-06-24. `../runs/run_079` to `run_088` are
  simulation episodes.

## Content

| Folder or file | Content | Date | Script |
|---|---|---|---|
| `deploy_logs_v1/` | 15 first closed-loop tests of the 40 Hz PPO agent `SpaceKinova_PPO_agent_motionprofile.mat`. Format `cfg`/`log`, time vector nominal (25 ms steps), no wall-clock time | 2026-04-23/24 | `deploy_agent_kinova.m` (V1, local copy `_archive/obsolete_scripts/deploy_agent_kinova_V2_Deploy_Scripts.m`) |
| `runs/run_007`, `run_023`–`025` | Open-loop playback at half speed (`kin_slow_training`) | 2026-05-04, 05-12 | `playback_variants.m` |
| `runs/run_008`–`022` | 10 Hz agent `ppo_10hz.mat`, half-circle, 8.5 s | 2026-05-04 | `deploy_agent_kinova_robust_timing.m` (V2.2) |
| `runs/run_026`–`034` | 10 Hz agent `ppo_10hz.mat`, 8.5 s (031: 6.5 s, 032–034: 17 s) | 2026-05-12 | V2.2 |
| `runs/run_035`–`062`, `067`–`069` | Set-point runs (`run_049` with `deploy_agent_kinova_point_fk.m`) | 2026-05-22 to 05-27 | `deploy_agent_kinova_point.m` |
| `runs/run_063`–`066` | Aborted after one step | 2026-05-27 | `Copy_of_deploy_agent_kinova_robust_timing2.m` |
| `runs/run_079`–`100` | Set-point runs | 2026-06-24 | `deploy_agent_kinova_point.m` |
| `p2p_legacy_logs/` | 22 logs in the older `deploy_log` format, same session as `runs/run_079`–`100` | 2026-06-24 | `deploy_agent_kinova_point.m` |
| `misc/` | Older versions of test logs and `dq_cmd` files, suffix = file date | 2026-04-24 to 05-26 | various |
| `../deploy_logs/run_013_agent_train_seed5.mat` | CDR2-4 agent at 10 Hz, 8.5 s, completed | 2026-04-30 | V2.2 |
| `../deploy_logs/run_014_agent_train_seed6.mat` | `ppo_10hz.mat`, 16 s, stopped after 18 steps | 2026-04-30 | V2.3 |

V2.1 and V2.2 build the observation differently from training (order and sign, finding A22 in the paper's review
list). V2.3 (`robust_timing2`, runs 074–078) is corrected.

## Effective command scaling

`meta.speedScale` is not the factor that reached the robot. The deploy scripts applied further fixed factors (for
example `0.35` in V2.3). The effective factor below is `max|dq_cmd| / max|dq_filt|` for J2 and J4, converted from
deg/s to rad/s (evaluated on 2026-09-22). A run counts as completed if the last logged wall-clock time is set.

| Runs | Agent, rate | Path | Effective factor | Result |
|---|---|---|---|---|
| `../deploy_logs/run_008`–`011` | 40 Hz PPO, V2.1 | 8.5 s | 0.21, 0.38, 0.61, 0.81 | all stopped (32–44 steps) |
| `../deploy_logs/run_012`–`014` (`_cdr`) | 40 Hz CDR2-4, V2.1 | 8.5 s | 0.29, 0.43, 0.62 | all stopped (34–67 steps) |
| `runs/run_008`–`030` here and `../deploy_logs/run_012_agent_train_seed10_okay.mat` | `ppo_10hz`, V2.2 | 8.5 s | 0.30–1.00 | 20 runs. Completed in 10 runs, all with 0.50–0.70. Stopped in 10 runs, with 0.30, 0.40–0.55 and 1.00 |
| `../runs/run_074`–`078` | `ppo_10hz`, V2.3 | 17 s | 0.35 | all completed |

The 40 Hz values in Table IV of the iSpaRo submission (RMS 0.1664 m, max 0.4175 m, 34 steps) are not in any of these
logs.
