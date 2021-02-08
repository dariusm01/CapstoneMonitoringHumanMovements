function Qk = SecondOrderPNC(SD,dt)

Qk = [0.05*SD*dt^5         0              0  0.125*SD*dt^4              0              0  0.167*SD*dt^3              0              0;
            0   0.05*SD*dt^5              0              0  0.125*SD*dt^4              0              0  0.167*SD*dt^3              0;
            0              0   0.05*SD*dt^5              0              0  0.125*SD*dt^4              0              0  0.167*SD*dt^3;
0.125*SD*dt^4              0              0  0.333*SD*dt^3              0              0    0.5*SD*dt^2              0              0;
            0  0.125*SD*dt^4              0              0  0.333*SD*dt^3              0              0    0.5*SD*dt^2              0;
            0              0  0.125*SD*dt^4              0              0  0.333*SD*dt^3              0              0    0.5*SD*dt^2;
0.167*SD*dt^3              0              0    0.5*SD*dt^2              0              0          SD*dt              0              0;
            0  0.167*SD*dt^3              0              0    0.5*SD*dt^2              0              0          SD*dt              0;
            0              0  0.167*SD*dt^3              0              0    0.5*SD*dt^2              0              0          SD*dt];
end

