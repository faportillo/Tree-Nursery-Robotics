

%Tracking object using kalman filter

clear all
%% simulation duration and sampling rate
duration = 20; % (s)
dt = .1;  % (s) sampling rate

%% Define update equations (Coefficent matrices): A physics based model for where we expect the train to be [state transition (state + velocity)] + [input control (acceleration)]
F = [1 dt; 0 1] ; % state transition matrix:  expected flight of the Quail (state prediction)
B = [dt^2/2; dt]; %input control matrix:  expected effect of the input accceleration on the state.
H = [1 0]; % measurement matrix

%% define main variables
u = 1.5; % define acceleration magnitude
X= [0; 0]; %initialized state--it has two components: [position; velocity]
Accel_noise_mag = .5; % process noise the variability in how fast the object is speeding up (stdv of acceleration: meters/sec^2)
Q = Accel_noise_mag^2 * [dt^4/4 dt^3/2; dt^3/2 dt^2]; % Ex convert the process noise (stdv) into covariance matrix

Sensor_noise_mag = 10;  %measurement noise: (stdv of location, in meters)
R = Sensor_noise_mag^2;% Ez convert the measurement noise (stdv) into covariance matrix

%% initialize result variables
X_true = []; % Object's TRUE state trajectory 
X_sensed = []; % object's SENSED state trajectory

%% simulate actual and sensed trajectories
for t = 0 : dt: duration

    % Generate the object's true (noisy) trajectory
    w = Accel_noise_mag * [(dt^2/2)*randn; dt*randn];
    X = F * X+ B * u + w;
    
    X_true = [X_true; X(1)];
    
    % Generate what the sensor sees; noise is included.
    v = Sensor_noise_mag * randn;
    y = H * X+ v;
    X_sensed = [X_sensed; y];
end


%% Now perform kalman filtering
%initize estimation variables
X= [0; 0]; % re-initialized state
X_estimate = X;  %x_estimate of initial location
X_loc_estimate = zeros(1, 100); %  Object position estimate
P = Q; % Initialize covariance matrix P

for t = 1:length(X_true)
    %%%%%%%%%%%% Kalman filter equations
    X_estimate = F * X_estimate + B * u;
    P = F * P * F' + Q;
    K = P * H' / (H * P * H' + R);
    X_estimate = X_estimate + K * (X_sensed(t) - H * X_estimate);
    P = P - K * H * P;
    
    %%%%%%%%%%%%%%%%% End of Kalman filter equations
    
    %Store for plotting
    if t > length(X_loc_estimate)
      X_loc_estimate = [X_loc_estimate, zeros(1, length(X_loc_estimate))];
    end
    X_loc_estimate(t) = X_estimate(1);
end

% PLOT the results
tt = 0 : dt : duration;
%plot true path (blue)
plot(tt,X_true,'--b.'); hold on;
%plot sensed and estimated path when a running average filter is used.
X_smooth = X_sensed;
plot(tt,X_sensed,'-k.', tt, X_smooth, '-g.');
%plot Kalman estimated path on same plot
plot( tt,X_loc_estimate(1:length(tt)),'-r.');
axis([0 20 -30 200]);


rms(X_true - X_smooth)
rms(X_true - X_loc_estimate)