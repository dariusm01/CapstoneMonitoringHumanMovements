function Qk = FirstOrderPNC(SD,dt)

Qk = [(SD*dt^3)/3     0       0 (SD*dt^2)/2     0     0
             0 (SD*dt^3)/3    0     0  (SD*dt^2)/2    0;
             0       0 (SD*dt^3)/3     0     0 (SD*dt^2)/2;
         (SD*dt^2)/2       0      0    SD*dt     0   0;
             0   (SD*dt^2)/2      0     0    SD*dt    0;
             0       0   (SD*dt^2)/2    0     0    SD*dt];
end

