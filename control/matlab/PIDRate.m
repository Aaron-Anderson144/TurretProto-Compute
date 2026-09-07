classdef PIDRate < handle

    properties
        Kp
        Ki
        u_sat = 1.5
        integ = 0
    end

    methods
        function obj = PIDRate(Kp, Ki, u_sat)
            obj.Kp = Kp;
            obj.Ki = Ki;
            if nargin >= 3, obj.u_sat = u_sat; end
        end

        function u = update(obj, w_sp, w_meas, dt)
            e       = w_sp - w_meas;
            u_unsat = obj.Kp * e + obj.Ki * (obj.integ + e * dt);
            u       = min(max(u_unsat, -obj.u_sat), obj.u_sat);
            
            if (u == u_unsat) || (sign(e) ~= sign(u_unsat))
                obj.integ = obj.integ + e * dt;
            end
        end
    end
end
