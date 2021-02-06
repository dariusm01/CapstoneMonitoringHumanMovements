function Qk = SecondOrderPNC(SD,dt)

% Still need to integrate!

Qk = [0.2500*SD*dt^4              0              0 0.5000*SD*dt^3              0              0 0.5000*SD*dt^2              0              0
             0 0.2500*SD*dt^4              0              0 0.5000*SD*dt^3              0              0 0.5000*SD*dt^2              0
             0              0 0.2500*SD*dt^4              0              0 0.5000*SD*dt^3              0              0 0.5000*SD*dt^2
0.5000*SD*dt^3              0              0        SD*dt^2              0              0          SD*dt              0              0
             0 0.5000*SD*dt^3              0              0        SD*dt^2              0              0          SD*dt              0
             0              0 0.5000*SD*dt^3              0              0        SD*dt^2              0              0          SD*dt
0.5000*SD*dt^2              0              0          SD*dt              0              0             SD              0              0
             0 0.5000*SD*dt^2              0              0          SD*dt              0              0             SD              0
             0              0 0.5000*SD*dt^2              0              0          SD*dt              0              0             SD];
end

