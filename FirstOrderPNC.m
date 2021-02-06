function Qk = FirstOrderPNC(SD,dt)

% Still need to integrate!

Qk = [SD*dt^2     0       0 SD*dt     0,     0
             0 SD*dt^2       0     0  SD*dt     0;
             0       0 SD*dt^2     0     0 SD*dt;
         SD*dt       0       0    SD     0     0;
             0   SD*dt       0     0    SD     0;
             0       0   SD*dt     0     0    SD];
end

