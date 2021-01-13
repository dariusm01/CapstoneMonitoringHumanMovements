load("FusionAlgoComparison.mat")

% Comparing the roll angle (φ) estimations 
figure(1)
plot(time, PhiEKF,'LineWidth',1)
grid on
hold on
plot(time, PhiUKF,'LineWidth',1)
plot(time, PhiComp,'LineWidth',1)
ylabel("Degrees [°]")
title("Roll Angle \phi using 3 state estimators [\alpha = 0.98]")
leg = legend("\phi EKF", "\phi UKF", "\phi Complimentary Filter",'Location','southeast');
leg.FontSize = 12;
hold off

% Comparing the pitch angle (θ) estimations 
figure(2)
plot(time, ThetaEKF,'LineWidth',1)
grid on
hold on
plot(time, ThetaUKF,'LineWidth',1)
plot(time, ThetaComp,'LineWidth',1)
ylabel("Degrees [°]")
title("Pitch Angle \theta using 3 state estimators [\alpha = 0.98]")
lgd = legend("\theta EKF", "\theta UKF", "\theta Complimentary Filter",'Location','southeast');
lgd.FontSize = 12;
hold off


