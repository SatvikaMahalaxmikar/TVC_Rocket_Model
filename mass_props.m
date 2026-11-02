function [T, m, s_cg, I] = mass_props(tm, P)
%MASS_PROPS  Thrust and time-varying mass properties.
%   tm   time since ignition [s]
%   T    thrust [N]
%   m    total mass [kg]
%   s_cg centre of gravity station from the nose [m]
%   I    [Ixx; Iyy; Izz] about the CG [kg m^2]

T = 0;
if tm <= 0
    Icum = 0;
else
    Icum = 0;
    for i = 1:numel(P.tc_t)-1
        t0 = P.tc_t(i);   t1 = P.tc_t(i+1);
        T0 = P.tc_T(i);   T1 = P.tc_T(i+1);
        if tm >= t1
            Icum = Icum + 0.5*(T0+T1)*(t1-t0);
        else
            sl = (T1-T0)/(t1-t0);
            dt = tm - t0;
            T = T0 + sl*dt;
            Icum = Icum + T0*dt + 0.5*sl*dt*dt;
            break
        end
    end
end
T    = P.T_scale*T;                       % motor-to-motor thrust variation (mass flow unchanged)
mp   = P.mp0*(1 - Icum/P.Itot);
m    = P.m_dry + mp;
s_cg = (P.m_dry*P.s_dry + mp*P.s_prop)/m;
Iyy  = P.Iyy_dry + P.m_dry*(s_cg-P.s_dry)^2 + mp*(s_cg-P.s_prop)^2;
I    = [P.Ixx; Iyy; Iyy];
end
