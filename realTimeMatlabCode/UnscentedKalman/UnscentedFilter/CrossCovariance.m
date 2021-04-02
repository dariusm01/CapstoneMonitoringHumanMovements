function Pxz = CrossCovariance(Mu_x, Mu_z, SigmasX, SigmasZ, Wc)

dim = size(SigmasX);

P = zeros(length(dim(1)));

    for i = 1:length(SigmasX)

        x = SigmasX(:,i) - Mu_x;
        z = SigmasZ(:,i) - Mu_z;

        P = (Wc(i)*(x*z.'))+P;
    end 
    
Pxz = P;
end 
