classdef TurretGimbal2Axis < handle


    properties
        J0    = 0.015          
        dJ    = 0.010          
        Jtilt = 0.012          
        mgl   = 0.15           
        b     = 0.02           
        tau_c = 0.03           
        tau_sat = 1.5          
        td_pan  = 0.02         
        td_tilt = 0.0          
        gyro_noise = 0.5 * pi/180
        enc_noise  = 0.02 * pi/180
        fric_eps   = 0.5 * pi/180
        s = [0; 0; 0; 0]       
    end

    methods
        function ds = deriv(obj, s, tau_pan, tau_tilt, wb_pan, wb_tilt)
            psiDot = s(2); th = s(3); thDot = s(4);
            Jpan  = obj.J0 + obj.dJ * cos(th)^2;
            Jpanp = -obj.dJ * sin(2*th);                 
            
            fp = obj.b*(psiDot - wb_pan)  + obj.tau_c*tanh((psiDot - wb_pan)/obj.fric_eps);
            ft = obj.b*(thDot  - wb_tilt) + obj.tau_c*tanh((thDot  - wb_tilt)/obj.fric_eps);
            psiDdot = (tau_pan  - fp - obj.td_pan  - Jpanp*thDot*psiDot) / Jpan;
            thDdot  = (tau_tilt - ft - obj.td_tilt + 0.5*Jpanp*psiDot^2 ...
                       - obj.mgl*cos(th)) / obj.Jtilt;
            ds = [psiDot; psiDdot; thDot; thDdot];
        end

        function [tau_pan, tau_tilt] = step(obj, tau_pan, tau_tilt, wb_pan, wb_tilt, dt, sub)
            if nargin < 7, sub = 10; end
            tau_pan  = min(max(tau_pan,  -obj.tau_sat), obj.tau_sat);
            tau_tilt = min(max(tau_tilt, -obj.tau_sat), obj.tau_sat);
            h = dt / sub;
            for i = 1:sub
                k1 = obj.deriv(obj.s,            tau_pan, tau_tilt, wb_pan, wb_tilt);
                k2 = obj.deriv(obj.s + 0.5*h*k1, tau_pan, tau_tilt, wb_pan, wb_tilt);
                k3 = obj.deriv(obj.s + 0.5*h*k2, tau_pan, tau_tilt, wb_pan, wb_tilt);
                k4 = obj.deriv(obj.s +     h*k3, tau_pan, tau_tilt, wb_pan, wb_tilt);
                obj.s = obj.s + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
            end
        end

        
        function y = gyroPan(obj),  y = obj.s(2) + obj.gyro_noise*randn(); end
        function y = gyroTilt(obj), y = obj.s(4) + obj.gyro_noise*randn(); end
        function y = encPan(obj),   y = obj.s(1) + obj.enc_noise*randn();  end
        function y = encTilt(obj),  y = obj.s(3) + obj.enc_noise*randn();  end

        function E = energy(obj)
            psiDot = obj.s(2); th = obj.s(3); thDot = obj.s(4);
            Jpan = obj.J0 + obj.dJ*cos(th)^2;
            E = 0.5*obj.Jtilt*thDot^2 + 0.5*Jpan*psiDot^2 + obj.mgl*sin(th);
        end
    end
end
