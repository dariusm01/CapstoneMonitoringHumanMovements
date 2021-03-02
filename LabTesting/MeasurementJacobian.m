function H = MeasurementJacobian(Phi,Theta,Psi,PhiDot,ThetaDot,PsiDot)

h1 = inline("cos(Theta)*cos(Psi) + cos(Theta)*sin(Psi) - sin(Theta)","Theta","Psi");

h2 = inline("(cos(Psi)*sin(Theta)*sin(Phi)) + (cos(Phi)*cos(Phi) + sin(Theta)*sin(Phi)*sin(Psi)) + cos(Theta)*sin(Phi)","Phi","Theta","Psi");

h3 = inline("(cos(Phi)*cos(Psi)*sin(Theta)) - (cos(Psi)*sin(Phi) + cos(Phi)*sin(Theta)*sin(Psi)) + cos(Theta)*cos(Phi)","Phi","Theta","Psi");

h4 = inline("PhiDot -(PsiDot*sin(Theta))","PhiDot","Theta","PsiDot");

h5 = inline("ThetaDot*cos(Phi) + PsiDot*cos(Theta)*sin(Phi)","Phi","Theta","PsiDot","ThetaDot");

h6 = inline("PsiDot*cos(Theta)*cos(Phi) - ThetaDot*sin(Phi)","PsiDot","Theta","Phi","ThetaDot");

%% Getting Partial Derivatives

dh1_dPhi = inline(diff(h1(Theta,Psi),Phi),"Theta","Psi");
dh2_dPhi = inline(diff(h2(Phi,Theta,Psi),Phi),"Phi","Theta","Psi");
dh3_dPhi = inline(diff(h3(Phi,Theta,Psi),Phi),"Phi","Theta","Psi");
dh4_dPhi = inline(diff(h4(PhiDot,Theta,PsiDot),Phi),"PhiDot","Theta","PsiDot");
dh5_dPhi = diff(h5,Phi);
dh6_dPhi = diff(h6,Phi);


dh1_dTheta = diff(h1,Theta);
dh2_dTheta = diff(h2,Theta);
dh3_dTheta = diff(h3,Theta);
dh4_dTheta = diff(h4,Theta);
dh5_dTheta = diff(h5,Theta);
dh6_dTheta = diff(h6,Theta);


dh1_dPsi = diff(h1,Psi);
dh2_dPsi = diff(h2,Psi);
dh3_dPsi = diff(h3,Psi);
dh4_dPsi = diff(h4,Psi);
dh5_dPsi = diff(h5,Psi);
dh6_dPsi = diff(h6,Psi);


dh1_dPhiDot = diff(h1,PhiDot);
dh2_dPhiDot = diff(h2,PhiDot);
dh3_dPhiDot = diff(h3,PhiDot);
dh4_dPhiDot = diff(h4,PhiDot);
dh5_dPhiDot = diff(h5,PhiDot);
dh6_dPhiDot = diff(h6,PhiDot);


dh1_dThetaDot = diff(h1,ThetaDot);
dh2_dThetaDot = diff(h2,ThetaDot);
dh3_dThetaDot = diff(h3,ThetaDot);
dh4_dThetaDot = diff(h4,ThetaDot);
dh5_dThetaDot = diff(h5,ThetaDot);
dh6_dThetaDot = diff(h6,ThetaDot);


dh1_dPsiDot = diff(h1,PsiDot);
dh2_dPsiDot = diff(h2,PsiDot);
dh3_dPsiDot = diff(h3,PsiDot);
dh4_dPsiDot = diff(h4,PsiDot);
dh5_dPsiDot = diff(h5,PsiDot);
dh6_dPsiDot = diff(h6,PsiDot);


H = [dh1_dPhi dh1_dTheta dh1_dPsi dh1_dPhiDot dh1_dThetaDot dh1_dPsiDot;
     dh2_dPhi dh2_dTheta dh2_dPsi dh2_dPhiDot dh2_dThetaDot dh2_dPsiDot;
     dh3_dPhi dh3_dTheta dh3_dPsi dh3_dPhiDot dh3_dThetaDot dh3_dPsiDot;
     dh4_dPhi dh4_dTheta dh4_dPsi dh4_dPhiDot dh4_dThetaDot dh4_dPsiDot;
     dh5_dPhi dh5_dTheta dh5_dPsi dh5_dPhiDot dh5_dThetaDot dh5_dPsiDot;
     dh6_dPhi dh6_dTheta dh6_dPsi dh6_dPhiDot dh6_dThetaDot dh6_dPsiDot];
 
end 





