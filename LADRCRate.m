classdef LADRCRate < handle


    properties
        b0
        kp
        b1
        b2
        u_sat  = 1.5
        z1     = 0      
        z2     = 0      
        u_prev = 0
    end

    methods
        function obj = LADRCRate(b0, wc, wo, u_sat)
            obj.b0 = b0;
            obj.kp = wc;
            obj.b1 = 2 * wo;
            obj.b2 = wo * wo;
            if nargin >= 4, obj.u_sat = u_sat; end
        end

        function u = update(obj, w_sp, w_meas, dt)
            
            err    = w_meas - obj.z1;
            obj.z1 = obj.z1 + dt * (obj.z2 + obj.b0 * obj.u_prev + obj.b1 * err);
            obj.z2 = obj.z2 + dt * (obj.b2 * err);
            u0     = obj.kp * (w_sp - obj.z1);
            u      = (u0 - obj.z2) / obj.b0;          
            u      = min(max(u, -obj.u_sat), obj.u_sat);
            obj.u_prev = u;
        end
    end
end
