%% Point + Vector stabilization controller

clear; clc; close all;
R2D = 180/pi;

NOISE_FREE = true;    
SEED       = 0;       


makePID   = @() PIDRate(0.55, 7.0);
makeLADRC = @() LADRCRate(1/0.018, 35.0, 180.0);

kinds  = {'point', 'vector', 'stabilize'};
titles = {'Point - step to 20 deg bearing (base still)', ...
          'Vector - track 40 deg/s moving target (rate FF)', ...
          'Stabilize - hold 0 deg while base sways \pm30 deg/s @ 1.5 Hz'};

R = struct();
fprintf('%s\n', repmat('=', 1, 74));
for i = 1:numel(kinds)
    s = kinds{i};
    R.(s).PID   = run_scenario(s, makePID,   NOISE_FREE, SEED);
    R.(s).LADRC = run_scenario(s, makeLADRC, NOISE_FREE, SEED);
    fprintf('%s\n', titles{i});
    fprintf('   PID    : %s\n', metric_str(s, R.(s).PID));
    fprintf('   LADRC  : %s\n', metric_str(s, R.(s).LADRC));
    fprintf('%s\n', repmat('-', 1, 74));
end


figure('Position', [100 100 900 950], 'Color', 'w');
cPID = [0.847 0.353 0.188];
cLAD = [0.325 0.290 0.718];
cREF = [0.37  0.37  0.35 ];

for i = 1:numel(kinds)
    s  = kinds{i};
    rp = R.(s).PID;
    rl = R.(s).LADRC;
    subplot(3, 1, i); hold on; grid on;

    if strcmp(s, 'stabilize')
        hP = plot(rp.t, rp.th * R2D, 'Color', cPID, 'LineWidth', 1.4);
        hL = plot(rl.t, rl.th * R2D, 'Color', cLAD, 'LineWidth', 1.4);
        yline(0, '--', 'Color', cREF);
        ylabel('pointing error [deg]');
        yyaxis right;
        hB = plot(rp.t, rp.wb * R2D, ':', 'Color', [0.70 0.70 0.66], 'LineWidth', 1.0);
        ylabel('base rate [deg/s]');
        set(gca, 'YColor', [0.53 0.53 0.50]);
        yyaxis left;
        legend([hP hL hB], {'PID', 'LADRC', 'base rate'}, 'Location', 'best');
    else
        plot(rp.t, rp.ref * R2D, '--', 'Color', cREF, 'LineWidth', 1.4);
        plot(rp.t, rp.th  * R2D,       'Color', cPID, 'LineWidth', 1.4);
        plot(rl.t, rl.th  * R2D,       'Color', cLAD, 'LineWidth', 1.4);
        ylabel('pointing angle [deg]');
        legend({'target', 'PID', 'LADRC'}, 'Location', 'best');
    end

    title(titles{i});
    xlim([0 rp.t(end)]);
    if i == numel(kinds), xlabel('time [s]'); end
end
sgtitle('Cascaded PID vs LADRC - turret Point / Vector / Stabilize');



function r = run_scenario(kind, makeRate, noiseFree, seed)
    D2R = pi/180;
    rng(seed, 'twister');                 
    plant = TurretAxis();
    if noiseFree
        plant.gyro_noise = 0;
        plant.enc_noise  = 0;
    end
    ctrl = CascadeController(makeRate());

    T = 3.0; dt = 1e-3;
    n = round(T / dt);
    t  = (0:n-1)' * dt;
    th = zeros(n,1); ref = zeros(n,1); u = zeros(n,1); wb = zeros(n,1);

    W_TGT  = 40 * D2R;   
    A_BASE = 30 * D2R;    
    F_BASE = 1.5;         
    t0     = 0.5;         

    for k = 1:n
        tk = t(k);
        switch kind
            case 'point'
                if tk >= t0, th_t = 20 * D2R; else, th_t = 0; end
                w_ff = 0; w_base = 0;
            case 'vector'
                if tk >= t0
                    th_t = W_TGT * (tk - t0); w_ff = W_TGT;
                else
                    th_t = 0; w_ff = 0;
                end
                w_base = 0;
            otherwise  
                th_t = 0; w_ff = 0;
                if tk >= t0
                    w_base = A_BASE * sin(2*pi*F_BASE*(tk - t0));
                else
                    w_base = 0;
                end
        end

        th_m  = plant.encoder();
        w_m   = plant.gyro();
        tau   = ctrl.step(th_t, w_ff, th_m, w_m, dt);
        u(k)  = plant.step(tau, w_base, dt);
        th(k) = plant.th; ref(k) = th_t; wb(k) = w_base;
    end

    r = struct('t', t, 'th', th, 'ref', ref, 'u', u, 'wb', wb, ...
               'dt', dt, 't0', t0, 'w_tgt', W_TGT, 'a_base', A_BASE, 'f_base', F_BASE);
end


function s = metric_str(kind, r)
    R2D = 180/pi; D2R = pi/180;
    t = r.t; th = r.th; ref = r.ref; t0 = r.t0;
    switch kind
        case 'point'
            final     = 20 * D2R;
            peak      = max(th(t >= t0));
            overshoot = max(0, (peak - final) / final * 100);
            band      = 0.02 * final;
            idx       = find(abs(th - final) > band, 1, 'last');
            if isempty(idx), ts = 0; else, ts = t(idx) - t0; end
            s = sprintf('overshoot %4.1f%%   settle(2%%) %5.0f ms', overshoot, ts*1000);
        case 'vector'
            m   = t >= 1.0;
            rms = sqrt(mean((th(m) - ref(m)).^2)) * R2D;
            lag = mean(ref(m) - th(m)) * R2D;
            s = sprintf('tracking RMS %5.3f deg   mean lag %5.3f deg', rms, lag);
        otherwise  
            m        = t >= (t0 + 0.2);
            rms      = sqrt(mean(th(m).^2)) * R2D;
            base_rms = r.a_base / sqrt(2) * R2D;
            atten    = 20 * log10((rms*D2R) / (r.a_base/sqrt(2)) + 1e-12);
            s = sprintf('pointing RMS %5.3f deg   base sway %4.1f deg  (%5.1f dB)', ...
                        rms, base_rms, atten);
    end
end
