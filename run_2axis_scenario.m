function r = run_2axis_scenario(plant, azCtrl, elCtrl, noiseFree, seed)


    D2R = pi/180;
    rng(seed, 'twister');                
    if noiseFree
        plant.gyro_noise = 0;
        plant.enc_noise  = 0;
    end

    T = 4.0; dt = 1e-3; n = round(T/dt);
    t  = (0:n-1)' * dt;
    az = zeros(n,1); el = zeros(n,1);
    azRef = zeros(n,1); elRef = zeros(n,1);

    t0 = 0.5;   
    t1 = 2.0;  

    AZ_PT = 30*D2R;  EL_PT = 20*D2R;                 
    AZ_RATE = 30*D2R;                                
    EL_AMP  = 8*D2R; EL_F = 0.35;                    
    A_BASE  = 25*D2R; F_BASE = 1.3;                  

    for k = 1:n
        tk = t(k);

        
        if tk < t0
            azT = 0; elT = 0; azFF = 0; elFF = 0;
        elseif tk < t1
            azT = AZ_PT; elT = EL_PT; azFF = 0; elFF = 0;          
        else
            te  = tk - t1;                                          
            azT = AZ_PT + AZ_RATE*te;             azFF = AZ_RATE;
            elT = EL_PT + EL_AMP*sin(2*pi*EL_F*te);
            elFF = EL_AMP*2*pi*EL_F*cos(2*pi*EL_F*te);
        end

        
        if tk >= t0
            wbP = A_BASE*sin(2*pi*F_BASE*(tk - t0));
            wbT = A_BASE*sin(2*pi*F_BASE*(tk - t0) + pi/2);
        else
            wbP = 0; wbT = 0;
        end

       
        azM = plant.encPan();   elM = plant.encTilt();
        wpM = plant.gyroPan();  wtM = plant.gyroTilt();
        tPan = azCtrl.step(azT, azFF, azM, wpM, dt);
        tTil = elCtrl.step(elT, elFF, elM, wtM, dt);
        plant.step(tPan, tTil, wbP, wbT, dt);

        az(k) = plant.s(1); el(k) = plant.s(3);
        azRef(k) = azT;     elRef(k) = elT;
    end

    r = struct('t', t, 'az', az, 'el', el, 'azRef', azRef, 'elRef', elRef, ...
               't0', t0, 't1', t1, 'dt', dt);
end
