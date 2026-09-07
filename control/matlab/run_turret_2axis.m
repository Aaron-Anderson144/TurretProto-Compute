%% Two-axis pan/tilt Point + Vector stabilization controller ->>
%-------------------------------------------------------------------
%  TurretGimbal2Axis driven by one cascade per axis, run
%  through a single mission that overlays all three behaviors:
%   - Point  : diagonal step to (az=30 deg, el=20 deg)
%   - Vector : az ramps at 30 deg/s, el follows an arc (rate FF on both)
%   - Stabilize : base sways +/-25 deg/s @ 1.3 Hz on both axes throughout
%  Requires: TurretGimbal2Axis.m, run_2axis_scenario.m, PIDRate.m,
%            LADRCRate.m, CascadeController.m  ( be in same folder)
%

clear; clc; close all;
R2D = 180/pi;
NOISE_FREE = true;     
SEED       = 0;


pPID = TurretGimbal2Axis();
azPID = CascadeController(PIDRate(0.55, 7.0));   
elPID = CascadeController(PIDRate(0.45, 7.0));   
rP = run_2axis_scenario(pPID, azPID, elPID, NOISE_FREE, SEED);


pLAD = TurretGimbal2Axis();
azLAD = CascadeController(LADRCRate(1/0.015, 35, 180));   
elLAD = CascadeController(LADRCRate(1/0.012, 35, 180));  
rL = run_2axis_scenario(pLAD, azLAD, elLAD, NOISE_FREE, SEED);


errP = R2D*sqrt((rP.az-rP.azRef).^2 + (rP.el-rP.elRef).^2);  
errL = R2D*sqrt((rL.az-rL.azRef).^2 + (rL.el-rL.elRef).^2);
mHold = (rP.t >= rP.t0+0.3) & (rP.t < rP.t1);                
mTrk  =  rP.t >= rP.t1+0.3;                                   

fprintf('%s\n', repmat('=',1,66));
fprintf('Point+sway  el-hold RMS : PID %.3f deg   LADRC %.3f deg\n', ...
    R2D*sqrt(mean((rP.el(mHold)-rP.elRef(mHold)).^2)), ...
    R2D*sqrt(mean((rL.el(mHold)-rL.elRef(mHold)).^2)));
fprintf('Vector+sway 2D-err  RMS : PID %.3f deg   LADRC %.3f deg\n', ...
    sqrt(mean(errP(mTrk).^2)), sqrt(mean(errL(mTrk).^2)));
fprintf('Vector+sway 2D-err  max : PID %.3f deg   LADRC %.3f deg\n', ...
    max(errP(mTrk)), max(errL(mTrk)));
fprintf('%s\n', repmat('=',1,66));


figure('Position',[80 80 1100 820], 'Color','w');
cP = [0.847 0.353 0.188]; cL = [0.325 0.290 0.718]; cR = [0.37 0.37 0.35];
t = rP.t; t0 = rP.t0; t1 = rP.t1;


subplot(2,2,1); hold on; grid on;
plot(t, rP.azRef*R2D, '--','Color',cR,'LineWidth',1.3);
plot(t, rP.az*R2D,   'Color',cP,'LineWidth',1.4);
plot(t, rL.az*R2D,   'Color',cL,'LineWidth',1.4);
xline(t0,':'); xline(t1,':');
ylabel('azimuth [deg]'); title('Azimuth (pan)');
legend({'target','PID','LADRC'},'Location','northwest'); xlim([0 t(end)]);


subplot(2,2,2); hold on; grid on;
plot(t, rP.elRef*R2D, '--','Color',cR,'LineWidth',1.3);
plot(t, rP.el*R2D,   'Color',cP,'LineWidth',1.4);
plot(t, rL.el*R2D,   'Color',cL,'LineWidth',1.4);
xline(t0,':'); xline(t1,':');
ylabel('elevation [deg]'); title('Elevation (tilt) - fights gravity');
legend({'target','PID','LADRC'},'Location','best'); xlim([0 t(end)]);


subplot(2,2,3); hold on; grid on;
plot(rP.azRef*R2D, rP.elRef*R2D, '--','Color',cR,'LineWidth',1.6);
plot(rP.az*R2D, rP.el*R2D, 'Color',cP,'LineWidth',1.0);
plot(rL.az*R2D, rL.el*R2D, 'Color',cL,'LineWidth',1.0);
plot(rP.azRef(1)*R2D, rP.elRef(1)*R2D, 'o','Color',cR,'MarkerSize',5);
xlabel('azimuth [deg]'); ylabel('elevation [deg]');
title('Aim path in az-el plane'); legend({'target path','PID','LADRC'},'Location','best');


subplot(2,2,4); hold on; grid on;
plot(t, errP, 'Color',cP,'LineWidth',1.3);
plot(t, errL, 'Color',cL,'LineWidth',1.3);
xline(t0,':'); xline(t1,':');
xlabel('time [s]'); ylabel('2D pointing error [deg]');
title('Total pointing error'); legend({'PID','LADRC'},'Location','best');
xlim([0 t(end)]); ylim([0 inf]);

sgtitle('Two-axis pan/tilt - coupled Point / Vector / Stabilize  (PID vs LADRC)');
