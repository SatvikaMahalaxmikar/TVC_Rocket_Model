function wind = wind_model(t, P)
%WIND_MODEL  Steady wind plus one 1-cos gust (inertial frame, m/s).
wind = P.wind_ss;
if t >= P.gust_t0 && t < P.gust_t0 + P.gust_T
    wind = wind + P.gust_amp*0.5*(1 - cos(2*pi*(t - P.gust_t0)/P.gust_T));
end
end
