#include <BasicLinearAlgebra.h>
using namespace BLA;

// https://github.com/tomstewart89/BasicLinearAlgebra/blob/master/examples/HowToUse/HowToUse.ino

// =========== Initialzing values ===========

    float phi{0};
    float theta{0};
    float psi{0};
  
    // Identity Matrix
    
    BLA::Matrix<3,3> I = {1, 0, 0,
                          0, 1, 0,
                          0, 0, 1};

    // Process Model
    
    BLA::Matrix<3,3> F{I};
    
    BLA::Matrix<3,1> xk_1;
  
    xk_1.Fill(0); // column vector of zeros

    BLA::Matrix<3,1> xk{xk_1};

    BLA::Matrix<3,1> zk{xk_1};
  
    // Input
    
    float dt = 0.01;
  
    BLA::Matrix<3,3> G = {dt, 0, 0,
                          0, dt, 0,
                          0, 0, dt};
                          
    // Process Covariance
    
    BLA::Matrix<3,3> Pk_1 = {500, 0,  0,
                              0, 500, 0,
                              0,  0, 500};

    BLA::Matrix<3,3> Pk;

    Pk.Fill(0);

    BLA::Matrix<3,3> Mk{Pk};

    // Process Noise Covariance

    BLA::Matrix<3,3> Qk = {0.01, 0,  0,
                            0, 0.01, 0,
                            0,  0, 0.01};     

    // Measurement Covariance

    BLA::Matrix<3,3> Rk = {0.1, 0,  0,
                            0, 0.1, 0,
                            0,  0, 0.1};                              
                              
    // Jacobian Matrix
    
    BLA::Matrix<3,3> H{I};


    // Innovation Covariance

    BLA::Matrix<3,3> Sk{Mk};

    // Residual
    
    BLA::Matrix<3,1> yk; // residual

    yk.Fill(0); // column vector of zeros

    // Kalman Gain
    
    BLA::Matrix<3,3> K;

    K.Fill(0);

    BLA::Matrix<3,3> IKH;

    IKH.Fill(0);

    BLA::Matrix<3,3> KRK{IKH};
    

void setup() {

    Serial.begin(9600);
    
}

void loop() {



    /*  Use sensor to gather data here:
     *
     *
     *
     *
     *
     *
     *
     *
     */

    BLA::Matrix<3,3> EulerKinematic = {1, sin(phi)*tan(theta),  cos(phi)*tan(theta),
                                       0, cos(phi), -sin(phi),
                                       0, sin(phi)/cos(theta), cos(phi)/cos(theta)};

    BLA::Matrix<3,1> Gyro = {gx,gy,gz};
     
  
    // Input 
    
    BLA::Matrix<3,1> u; // Euler rates

    Multiply(EulerKinematic.Ref(),Gyro.Ref(),u); // Using references for speed
    
    //  ======= Prediction =======
    
    BLA::Matrix<3,1> xkp = (F * xk_1 + G * u);


    // ======= Propagate Covariance =======
    
    BLA::Matrix<3,3> tempCov;
    
    // F * Pk_1 = tempCov
    Multiply(F.Ref(),Pk_1.Ref(),tempCov); 
  
    BLA::Matrix<3,3> F_T = ~F; // F^T
   

    // Mk = F*Pk_1*F.'+ Qk;  
    Multiply(tempCov.Ref(),F_T.Ref(),Mk); 

    Mk+= Qk;


    // ======= Innovation Covariance =======
    
    BLA::Matrix<3,3> tempCov2;

    // H * Mk = tempCov2
    Multiply(H.Ref(),Mk.Ref(),tempCov2);

    BLA::Matrix<3,3> tempTranspose = ~H; // H^T

    // Sk = H*Mk*H.' + Rk;
    Multiply(tempCov2.Ref(),tempTranspose.Ref(),Sk);

    Sk += Rk;

    
    // ======= Measurement =======

    BLA::Matrix<3,1> zk = {ax,ay,az}; // Accelerometer values

    BLA::Matrix<3,1> AccelModel = {-sin(theta),
                                    cos(theta)*sin(phi),
                                    cos(theta)*cos(phi)};

    
    BLA::Matrix<3,3> Jacobian = {0,                   -cos(theta),           0,
                                 cos(theta)*cos(phi), -sin(theta)*sin(phi),  0,
                                -cos(theta)*sin(phi), -sin(theta)*cos(phi),  0};

    H = Jacobian;                            

    // ======= Residual =======
    
    Subtract(zk.Ref(), AccelModel.Ref(), yk); 


    // ======= Kalman Gain =======

    BLA::Matrix<3,3> Sk_inv = Sk.Inverse(); // ... this may cause some issues

    /* int res;
     * Sk_inv = Sk.Inverse(&res); // if this is zero, the inverse does not exist 
     *  
     *  // ======== Pseudo Inverse ========
     *  
     *  if &res == 0 {
     *  try:
     *    Sk_inv = (~Sk*Sk).Inverse() * ~Sk;
     *  or  
     *    Sk_inv = ~Sk * (~Sk*Sk).Inverse() 
     *  }
     */
    
    BLA::Matrix<3,3> H_T = ~H; // H^T

    BLA::Matrix<3,3> tempK;

    tempK.Fill(0);

    Multiply(Mk.Ref(),H_T.Ref(),tempK);

    Multiply(tempK.Ref(),Sk_inv.Ref(),K); 


    // ======= Update State =======

    xk = xk_1 + K*yk;

    phi =   xk(1);   // x-axis rotation
    theta = xk(2);   // y-axis rotation
    psi =   xk(3);   // z-axis rotation


    // ======= Update Uncertainty =======

    IKH = (I.Ref() - K.Ref()*H.Ref());

    KRK = K.Ref() * Rk.Ref() * (~K.Ref());

    // Pk = (I - K*H)*Mk*(I - K*H).' + (K*Rk*K.');

    Pk = (IKH * Mk * (~IKH)) + KRK;

    /*  Print stuff?:
     *
     *
     *
     *
     *
     *
     *
     *
     */

    // Redefining for next timestep

    xk_1 = xk;

    Pk_1 = Pk;


}
