function n = desktop_obs_randn()
%DESKTOP_OBS_RANDN  29 standardnormalverteilte Zufallszahlen fuer das Beobachtungsrauschen in SK_desktop (D7).
%   Wird aus dem Block "Obs Transform" als extrinsische Funktion aufgerufen, damit rng-Seeds aus MATLAB wirken.
n = randn(29, 1);
end
