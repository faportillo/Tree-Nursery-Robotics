function [kPose, P] = nurseryEKF(kPose, odo, z, P, V, W)
%Extended Kalman Filter takes in dead reckoning states, covariance
%matrices, and noisy gps reading and returns new, filtered states and
%updated covariance matrices

%kPose -> kalman filter pose
%odo -> [distance change, angle change]' column vector from model
%z -> [x, y, theta]' column vector from gps/compass noisy
%P -> Estimate of state uncertainty
%V -> estimate of process noise covariance
%W -> estimate of sensor noise covariance

x = kPose(1);
y = kPose(2);
theta = kPose(3);
dDist = odo(1);
dTheta = odo(2);

%Estimate the new state w/ nonlinear dead reckoning
kPose = [kPose(1) + odo(1)*cos(kPose(3)); kPose(2) + odo(1)*cos(kPose(3)); kPose(3) + odo(2)];
%Calculate Jacobians
Fx = [1 0 -dDist*sin(theta); 0 1 dDist*cos(theta); 0 0 1];
Fv = [cos(theta)  0; sin(theta) 0; 0 1];
%estimate new linearized state
%kPose = kPose + Fx*(x_nonlin - kPose) + Fv*odo;
%Update the uncertainty
P = Fx*P*transpose(Fx) + Fv*V*transpose(Fv);
%Innovation covariance
S = P + W;
%Kalman gain matrix
K = P*inv(S);
%Compute innovation/residuals
v = z - kPose; 
%Update step
kPose = kPose + K*v;
P = P - K*P;