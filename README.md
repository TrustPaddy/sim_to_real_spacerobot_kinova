# Sim-to-Real Reinforcement Learning for SpaceKinova

MATLAB/Simulink code, trained agents and hardware logs for training reinforcement-learning (RL) policies on a
simulated 7-DOF Kinova Gen3 arm mounted on a free-floating cube base ("SpaceKinova"), and for deploying them on a
table-mounted Kinova Gen3.

The work started as a bachelor thesis at Frankfurt University of Applied Sciences (see [Citation](#citation)). It
covers two tasks, Cartesian half-circle tracking and set-point attainment (point-to-point reaching). It also includes
a comparison of six RL algorithms, Bayesian hyperparameter tuning of PPO, curriculum domain randomization (CDR), and a
hardware study of the control-loop rate that the Kinova high-level API reaches from MATLAB.

![Sim-to-real pipeline](Figures/Schemata/Gesamtdiagramm_SimToRealPipeline.png)

## Repository layout

```
setup_project.m      one-time setup per MATLAB session (path, working folder, Simulink cache)
robot/               SpaceKinova URDF, plain Gen3 URDF, URDF generator
models/              Simulink/Simscape Multibody models
training/            training scripts
evaluation/          simulation KPI scripts
  campaign/          analysis of the 2026 hardware campaign (tables and figures from the logs)
hardware/
  deploy/            closed-loop deployment of a trained agent on the real Gen3
  campaign/          2026 measurement campaign: tracking and set-point deploy V2.4, loop timing,
                     fixed set-point start list, plan (MESSPLAN.md)
  playback/          open-loop playback of logged joint-velocity commands, jog tests
  analysis/          offline analysis of hardware and simulation runs
utils/               sk_path (repository paths), run_logger (numbered run files)
SavedAgents/         trained agents (.mat, variable "agent")
data/
  dq_cmd/            logged simulation commands used as playback input
  hardware/          hardware and simulation run logs, KPI summaries
Figures/             figures from the thesis
ros_kortex/          Gen3 meshes from Kinova's ros_kortex (only the files the models need)
```

## Requirements

- MATLAB R2025b with Simulink, Simscape, Simscape Multibody, Robotics System Toolbox, Reinforcement Learning Toolbox
  and Deep Learning Toolbox. The models are saved in R2025b. Newer releases open them but upgrade the file on save.
- Parallel Computing Toolbox (optional, used by some training scripts).
- For hardware runs only: a Kinova Gen3 7-DOF, the Robotics System Toolbox Support Package for KINOVA Gen3
  Manipulators (`kortexApiMexInterface`), an emergency stop and a clear workspace.

## Getting started

```matlab
cd path/to/this/repository
setup_project                         % adds all folders to the path and stays in the repository folder
calculate_kpi_spacekinova_point       % set-point evaluation in simulation (50 episodes)
calculate_kpi_spacekinova             % tracking evaluation in simulation (50 episodes)
```

Run the scripts by name from the repository folder. The Simulink models load the Kinova meshes through the relative
path `ros_kortex/...`, so the simulating scripts switch to the repository folder themselves. Agent and model are set
at the top of each script.

## Simulation models

In the velocity models (all except `Torque` and `PD_Control`) the RL agent outputs one velocity command per joint. A
post-action chain (saturation, first-order IIR low-pass `0.05/(1 - 0.95 z^-1)`, rate limiter 0.5 rad/s², integrator)
turns it into a joint position that drives the Simscape joints as prescribed motion. Gravity is off unless noted.

| Model | Agent rate / solver step | Used for |
|---|---|---|
| `SpaceKinova_MotionProfile.slx` | 10 Hz / 20 ms | tracking, 10 Hz agent |
| `SpaceKinova_MotionProfile_40Hz.slx` | 40 Hz / 5 ms | tracking, 40 Hz agents |
| `SpaceKinova_MotionProfile_CDR.slx` | 40 Hz / 5 ms | CDR tests. Stored in a fixed test setting: base 16.25 kg, joint damping ×2, fixed actuator delay |
| `SpaceKinova_MotionProfile_point.slx` | 10 Hz / 20 ms | set-point evaluation. Base 650 kg, start pose [0 −90 0 0 0 0 0]° |
| `SpaceKinova_MotionProfile_point_fixed.slx` | 10 Hz / 20 ms | set-point training for the hardware agent. Base signals set to zero, gravity on, observation-noise block |
| `SpaceKinova_Torque.slx` | 40 Hz / 5 ms | algorithm comparison. Torque actions, reward without base terms |
| `SpaceKinova_PD_Control.slx` | 40 Hz / 5 ms | thesis only (PD velocity loop) |

For planar tracking only joints J2, J4 and J6 are active (saturation ±0.9774, ±0.9774 and ±0.1 rad/s). The
set-point models use all seven joints.

## Training

| Script | Model | Result |
|---|---|---|
| `training/SpaceKinovaDynamic_10Hz.m` | `SpaceKinova_MotionProfile` | 10 Hz PPO agent, half circle from the upright pose, 8.5 s (`Circle/PPO/ppo_10hz.mat`) |
| `training/SpaceKinovaDynamic.m` | `SpaceKinova_MotionProfile` | 10 Hz PPO variant on a second half circle (center [0.479, −0.005, 1.136] m), 16 s. The other five algorithms are included as commented blocks |
| `training/SpaceKinova_CDR.m` | `SpaceKinova_MotionProfile_40Hz` | CDR with per-episode randomization. Only trajectory randomization (CDR-1) is active |
| `training/SpaceKinova_CDR_FRO.m` | `SpaceKinova_MotionProfile_CDR` | CDR in phase sub-batches with model recompilation (mass, actuator delay, friction) |
| `training/SpaceKinova_Point.m` | `SpaceKinova_MotionProfile_point_fixed` | set-point PPO agent, five-stage start/target curriculum, up to 3000 episodes |
| `training/SpaceKinova_Point_CDR.m` | `SpaceKinova_MotionProfile_point_fixed` | set-point variant with actuator delay and observation noise |

Saving is commented out in most training scripts. After a run, `agent` and `trainingStats` stay in the workspace.

## Trained agents

| Agent | Rate | Where it is used |
|---|---|---|
| `Torque/Circular/{PPO,TRPO,DDPG,TD3,SAC,PG}.mat` | 40 Hz | algorithm comparison (MATLAB default hyperparameters) |
| `Torque/Linear/...` | 40 Hz | thesis: triangle path and PPO ablations |
| `MotionProfile/Circle/PPO/SpaceKinova_PPO_agent_motionprofile.mat` | 40 Hz | PPO baseline, hardware runs 008–011 |
| `MotionProfile/{Circle,Linear}/PPO/Optimized.mat` | 40 Hz | PPO with Bayesian-optimized hyperparameters |
| `MotionProfile/CDR/PPO/*.mat` | 40 Hz | one agent per CDR feature and the combinations CDR2-4 and CDR1-4. `CDR2-4.mat` in hardware runs 012–014 |
| `MotionProfile/Circle/PPO/ppo_10hz.mat` | 10 Hz | frequency-matched agent, hardware runs 012 (10 Hz) and 074–078 |
| `MotionProfile/NonSingular/*.mat` | 40 / 10 Hz | agents for the second half circle |
| `MotionProfile/point/test_agent_rand2.mat` | 10 Hz | set-point evaluation in simulation |
| `MotionProfile/point/test_agent_fixed1.mat` | 10 Hz | set-point hardware runs 068–073 |

All agents use two hidden layers with 128 ReLU units for actor and critic.

## Hardware

The deployment scripts connect to the Gen3 through `kortexApiMexInterface` (default IP 192.168.0.10). They
replicate the post-action chain of the models and add a safety layer: hardware velocity cap, soft-limit braking, fault
check, stop at a position error above 0.4 m (tracking) or 2.0 m (set-point), and a timing watchdog. Always start with `cfg.dryRun = true`.

| Script | Purpose |
|---|---|
| `hardware/deploy/deploy_agent_kinova_improved.m` | V2.1, tracking, extended logging (runs 008–014) |
| `hardware/deploy/deploy_agent_kinova_robust_timing.m` | V2.2, tracking with guarded reference time and timing diagnosis (run 012, 10 Hz) |
| `hardware/deploy/deploy_agent_kinova_robust_timing2.m` | V2.3, tracking with corrected observation, 10 Hz, 17 s reference (runs 074–078) |
| `hardware/deploy/deploy_agent_kinova_point.m` | set-point deployment at 10 Hz, end-effector state from Kortex `tool_pose`/`tool_twist` (runs 068–073) |
| `hardware/campaign/deploy_tracking_v24.m` | V2.4 for the 2026 campaign: conditions from `campaign_plan.m`, one explicit command factor, the stop step is logged, per-step timing of every stage, observation and full command chain, Kortex pose, metadata with git hash and agent MD5. Logs in `data/hardware/campaign/` |
| `hardware/campaign/measure_loop_timing.m` | loop-time measurement with the robot at rest (zero commands) for send only, send + feedback, + FK, and the full loop |
| `hardware/campaign/deploy_setpoint_v24.m` | set-point V2.4 for the campaign: fixed start poses, end-effector from the URDF FK as in training (V3.0 used the Kortex `tool_pose`, 0.121 m further along the tool axis), one command factor, logged stop step, height guard, kinematic dry run |
| `hardware/campaign/make_setpoint_starts.m` | generates the fixed start list `setpoint_starts.mat`/`.csv` (15 starts over seven start distances to the nominal training target) and overview plots, offline |
| `hardware/campaign/check_setpoint_starts.m` | lab check: moves to each start pose, asks for approval, logs measured vs. predicted `tool_pose` to `data/hardware/campaign/setpoint_start_check.csv` |
| `evaluation/campaign/analyze_campaign.py` | builds Table III, Table IV, Fig. 7 and the set-point table and figure (final error over start distance) directly from the campaign logs, plus `paper_numbers.csv` |
| `hardware/playback/playback_variants.m` | open-loop playback of `data/dq_cmd/*.mat` at three time scales (runs 001–006) |
| `hardware/playback/kinova_test.m` | joint-velocity jog test |
| `hardware/analysis/compare_sim2real.m` | tracking KPIs of real and simulated runs on a common phase axis |
| `hardware/analysis/evaluate_p2p_hardware.m` | set-point KPIs of the hardware runs |
| `hardware/analysis/analyze_tracking_error*.m` | joint and Cartesian error of the playback runs |

The run files in `data/hardware/` contain `data` (time series) and `meta` (agent file, rate, speed scale, script,
date). The number in the label (`seed11`, ...) counts runs, it is not an RL seed.

| Folder / runs | Content |
|---|---|
| `playback_runs/run_001`–`006` | open-loop playback, first half circle (001–003) and second half circle (004–006) |
| `deploy_logs/run_008`–`014` | 40 Hz agents, speed scale 0.3 to 1.0 |
| `deploy_logs/run_012_agent_train_seed10_okay.mat` | 10 Hz agent, 8.5 s reference, speed scale 0.6 |
| `runs/run_068`–`073` | set-point agent, six start/target pairs |
| `runs/run_074`–`078` | 10 Hz agent, 17 s reference |
| `runs/run_079`–`088` | simulated episodes of the 10 Hz agent in the same format |
| `runs/*.csv`, `analysis_output/summary.csv` | KPI summaries |

## Known limitations

- The deployment scripts V2.1 and V2.2 build the observation differently from the models: order
  `[ep; ev; q; dq; v_base; w_base; e_ori]` instead of `[ep; ev; v_base; w_base; q; dq; e_ori]`, error sign
  reference − state instead of state − reference, and the end-effector orientation in the base-orientation slot. V2.3
  (`robust_timing2`) and the set-point script use the training convention. Runs 008–014 were recorded with the old
  observation.
- `deploy_agent_kinova_robust_timing2.m` scales the commands by a fixed factor of 0.35 on top of `speedScale`. The run
  files store only `speedScale`.
- The joints are driven as prescribed motion. Joint damping and friction therefore change only the computed torques,
  not the motion or the base reaction.
- `SpaceKinova_MotionProfile_CDR.slx` does not read the randomization variables that `SpaceKinova_CDR_FRO.m` writes.
  To retrain with CDR, link base mass, joint damping and delay length to `base_mass_factor`, `joint_damping` and
  `act_delay_steps` first.
- The agents are saved with `UseExplorationPolicy = true`, so the evaluation scripts run the stochastic policy. The
  deployment scripts switch to the deterministic policy.
- The hardware runs scale the commands after the post-action chain (`speedScale` in `meta`), and the hardware cap is
  75 % of the joint limits.
- The IK weights `[1 1 1 0.05 0.05 0.05]` in the tracking scripts weight orientation strongly and position weakly
  (MATLAB order is orientation first).

## License

The code in this repository is released under the MIT License, see `LICENSE`.

`ros_kortex/` contains the Gen3 7-DOF meshes from [Kinovarobotics/ros_kortex](https://github.com/Kinovarobotics/ros_kortex)
under the BSD 3-Clause license in `ros_kortex/LICENSE`. The same applies to `robot/GEN3-7DOF-VISION_ARM_URDF_V12.urdf`
and to the Kinova link and joint data in `robot/SpaceKinova.urdf`, which was generated with
`robot/make_spacekinova_urdf.m`.

## Citation

Steven Patrick Ermisch, "Sim-to-Real-Uebertragung einer Reinforcement Learning getriebenen kartesischen
Bewegungsplanung eines 7 DoF frei-schwebenden Weltraumroboters", Bachelor thesis, Frankfurt University of Applied
Sciences, 2026.

```bibtex
@thesis{ermisch2026spacekinova,
  author = {Ermisch, Steven Patrick},
  title  = {Sim-to-Real-Uebertragung einer Reinforcement Learning getriebenen kartesischen Bewegungsplanung
            eines 7 DoF frei-schwebenden Weltraumroboters},
  school = {Frankfurt University of Applied Sciences},
  type   = {Bachelor thesis},
  year   = {2026}
}
```
