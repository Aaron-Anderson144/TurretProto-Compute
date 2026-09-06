classdef TurretAxis < handle


    properties
        J          = 0.015          
        b          = 0.02          
        tau_c      = 0.03           
        tau_bias   = 0.05           
        tau_sat    = 1.5            
        gyro_noise = 0.5 * pi/180   
        enc_noise  = 0.02 * pi/180  
        fric_eps   = 0.5 * pi/180   
        th         = 0              
        w          = 0              
    end

    methods
        function wd = wdot(obj, w, tau, w_base)
            wr   = w - w_base;                                   
            fric = obj.b * wr + obj.tau_c * tanh(wr / obj.fric_eps);
            wd   = (tau - fric - obj.tau_bias) / obj.J;
        end

        function tau = step(obj, tau, w_base, dt, sub)
           
            if nargin < 5, sub = 10; end
            tau = min(max(tau, -obj.tau_sat), obj.tau_sat);
            h = dt / sub;
            for i = 1:sub
                w   = obj.w;
                k1w = obj.wdot(w,             tau, w_base); k1t = w;
                k2w = obj.wdot(w + 0.5*h*k1w, tau, w_base); k2t = w + 0.5*h*k1w;
                k3w = obj.wdot(w + 0.5*h*k2w, tau, w_base); k3t = w + 0.5*h*k2w;
                k4w = obj.wdot(w +     h*k3w, tau, w_base); k4t = w +     h*k3w;
                obj.w  = obj.w  + (h/6) * (k1w + 2*k2w + 2*k3w + k4w);
                obj.th = obj.th + (h/6) * (k1t + 2*k2t + 2*k3t + k4t);
            end
        end

        function y = gyro(obj)
            y = obj.w + obj.gyro_noise * randn();
        end

        function y = encoder(obj)
            y = obj.th + obj.enc_noise * randn();
        end
    end
end
