#include <BasicLinearAlgebra.h>
using namespace BLA;

// https://github.com/tomstewart89/BasicLinearAlgebra/blob/master/examples/HowToUse/HowToUse.ino

void setup() {

    Serial.begin(9600);
  
    // Identity Matrix
    BLA::Matrix<3,3> I = {1, 0, 0,
                          0, 1, 0,
                          0, 0, 1};

    // Process Model
    
    BLA::Matrix<3,3> F{I};
    
    BLA::Matrix<3,1> xk_1;
  
    xk_1.Fill(0); // column vector of zeros

    BLA::Matrix<3,1> zk{xk_1};
  
    // Input
    float dt = 0.01;
  
    BLA::Matrix<3,3> G = {dt, 0, 0,
                          0, dt, 0,
                          0, 0, dt};
                          

    BLA::Matrix<3,3> Pk_1 = {500, 0,  0,
                              0, 500, 0,
                              0,  0, 500};
                              

    // Observation Matrix
    BLA::Matrix<3,3> H{I};

    
}

void loop() {

    // Input 
    BLA::Matrix<3,1> u = {20,0,5}; // change these to Euler rates
    
    // Prediction
    BLA::Matrix<3,1> xkp = (F * xk_1 + G * u);


         // ======= Propagate Covariance =======
    BLA::Matrix<3,3> tempCov;
    
    // F * Pk_1 = tempCov
    Multiply(F.Ref(),Pk_1.Ref(),tempCov); // Using references for speed
  
    BLA::Matrix<3,3> tempTranspose = ~F; // F^T
   
    BLA::Matrix<3,3> Mk;

    // Mk = F*Pk_1*F.'+ Qk;  
    Multiply(tempCov.Ref(),tempTranspose.Ref(),Mk); // Using references for speed

        // ======= Innovation Covariance =======
    BLA::Matrix<3,3> tempCov2;

    // H * Mk = tempCov2
    Multiply(H.Ref(),Mk.Ref(),tempCov2); // Using references for speed

    BLA::Matrix<3,3> tempTranspose2 = ~H; // H^T

    BLA::Matrix<3,3> Sk;

    // Sk = H*Mk*H.' + Rk;
    Multiply(tempCov2.Ref(),tempTranspose2.Ref(),Sk); // Using references for speed

}
