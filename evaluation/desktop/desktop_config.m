function cfg = desktop_config(varargin)
%DESKTOP_CONFIG  Standardeinstellungen fuer eine Auswerte-Episode (Desktop-Plan D0).
%   cfg = desktop_config()                   Standard: CDR2-4, 40 Hz, 65 kg, ohne Stoerung
%   cfg = desktop_config('Ts_agent', 0.1)    einzelne Felder ueberschreiben
%
%   Felder:
%     model        Simulink-Modell. 'SK_desktop' (parametrisiert, desktop_build_model) oder ein
%                  Originalmodell aus models/ zum Vergleich. Originalmodelle ignorieren die
%                  Felder base_mass, delay_steps und damp_scale.
%     agentFile    .mat-Datei mit der Variable agent
%     agentLabel   Kurzname fuer Tabellen
%     Ts           Solver-Schritt [s]
%     Ts_agent     Agenten-Takt [s]. Weicht er von der Trainingsrate ab, laeuft der Agent mit
%                  diesem Takt, Filter und Rate Limiter rechnen im selben Takt (wie auf der Hardware)
%     T            Episodendauer [s]
%     T_path       Dauer der Halbkreisbahn [s] (Standard T)
%     r, center    Halbkreis in der x-z-Ebene. Standard ist die Trainingsbahn, sie beginnt in der
%                  gestreckten Nullstellung (A27)
%     shape        'halfcircle' (Standard) oder 'triangle' (Dreieck ueber dieselben drei Eckpunkte wie in
%                  calculate_kpi_spacekinova.m: Start, Scheitel bei T/2, Ende), nur bei ref_timing 'wall'
%     mirror_x     true = gespiegelter Halbkreis (x = cx - r sin statt cx + r sin), gleicher Start und
%                  gleiche Richtung nach unten. Annahme fuer die "mirrored half-circle" aus E1 (D5)
%     base_mass    Basismasse [kg]. 1e9 haelt die Basis praktisch fest (D3)
%     delay_steps  Aktionsverzoegerung in Agentenschritten
%     damp_scale   Faktor auf die Gelenkdaempfung
%     slew         Anstieg des Rate Limiters [1/s]. Training 0,5. V2.1 rechnete ihn fest mit 25 ms pro
%                  Schritt, bei laengeren Schritten entspricht das 0,5 * 0,025 / Ts_agent
%     cmd_scale    Faktor auf den Befehl nach der Kette (speedScale, A18)
%     obs_mode     0 = Beobachtung wie im Training, 1 = wie Deploy-Skript V2.1 (A22)
%     obs_noise    29x1 Standardabweichungen des Beobachtungsrauschens (Reihenfolge wie im Training), Standard 0
%     ref_timing   'wall' = Referenz nach Zeit. 'sample' = Referenz rueckt pro Agentenschritt um Ts_ref_step
%                  vor (Verhalten von V2.1), die Geschwindigkeitsreferenz bleibt die nominale
%     Ts_ref_step  Referenzvorschub pro Schritt bei 'sample' [s], Standard 0,025
%     explore      true = stochastische Policy wie in den alten KPI-Skripten (A28),
%                  false = deterministisch wie auf der Hardware
%     seed         rng-Seed, wirkt nur bei explore = true
%     keepTs       Zeitreihen im Ergebnis behalten

cfg = struct();
cfg.model       = 'SK_desktop';
cfg.agentFile   = sk_path('SavedAgents', 'MotionProfile', 'CDR', 'PPO', 'CDR2-4.mat');
cfg.agentLabel  = 'CDR2-4';
cfg.Ts          = 0.005;
cfg.Ts_agent    = 0.025;
cfg.T           = 8.5;
cfg.T_path      = [];
cfg.r           = 0.2;
cfg.center      = [0.0, -0.025, 1.687 - 0.2];
cfg.shape       = 'halfcircle';
cfg.mirror_x    = false;
cfg.base_mass   = 65;
cfg.delay_steps = 0;
cfg.damp_scale  = 1;
cfg.slew        = 0.5;
cfg.cmd_scale   = 1;
cfg.obs_mode    = 0;
cfg.obs_noise   = zeros(29, 1);
cfg.ref_timing  = 'wall';
cfg.Ts_ref_step = 0.025;
cfg.explore     = false;
cfg.seed        = 0;
cfg.keepTs      = true;
cfg.label       = '';

for k = 1:2:numel(varargin)
    name = varargin{k};
    assert(isfield(cfg, name), 'desktop_config:unknownField', 'Unbekanntes Feld: %s', name);
    cfg.(name) = varargin{k+1};
end
if isempty(cfg.T_path)
    cfg.T_path = cfg.T;
end
end
