classdef CascadeController < handle


    properties
        rate                    
        Kp_p     = 10
        Kd_p     = 0.3
        slew_max = 300 * pi/180 
        th_prev  = []           
    end

    methods
        function obj = CascadeController(rate_ctrl, Kp_p, Kd_p, slew_max)
            obj.rate = rate_ctrl;
            if nargin >= 2 && ~isempty(Kp_p),     obj.Kp_p     = Kp_p;     end
            if nargin >= 3 && ~isempty(Kd_p),     obj.Kd_p     = Kd_p;     end
            if nargin >= 4 && ~isempty(slew_max), obj.slew_max = slew_max; end
        end

        function u = step(obj, th_target, w_ff, th_meas, w_meas, dt)
            if isempty(obj.th_prev), obj.th_prev = th_meas; end
            dth         = (th_meas - obj.th_prev) / dt;
            obj.th_prev = th_meas;
            w_pos = obj.Kp_p * (th_target - th_meas) - obj.Kd_p * (dth - w_ff);
            w_sp  = w_ff + w_pos;
            w_sp  = min(max(w_sp, -obj.slew_max), obj.slew_max);
            u     = obj.rate.update(w_sp, w_meas, dt);
        end
    end
end
