% A simple kalman example
% States x = [x , y, vx, vy]
% Input u = [ax, ay]
% Initail state
clear all; clc;
dt = 0.1;
x0 = [0, 0, 1, 0];  % Driving at 1m/s from the origin along the x-axis.
x0real = x0;
u0 = [0, 0];        % No acceleration input no steering
x_est_plot = []; y_est_plot = [];
x_real_plot = []; y_real_plot = [];
innovation = [0;0];


% Defining the dynamics 2D motion
% The state transition model, "dynamic model" (sometimes F)
A = [1, 0, dt, 0;...
     0, 1, 0,  dt;...
     0, 0, 1, 0;...
     0, 0, 0, 1];
% The control input model
B = [0,    0;...
     0,    0;...
     dt,   0;...
     0,    dt];
 
% The Observation model, From the measurements to what is part of your
% state.
H = [1, 0, 0, 0;...
     0, 1, 0, 0]; %Measure x and y and no velocity. 

% After haven seen how "bad"/"noicy" a signal is we update this matrixes
% manually.
% Measurement error (unknown? -no from GNSS sensor)
R = diag([0.05^2, 0.05^2]); % 5cm standard deviation on position (x and y)
% Process noice (disturbances to the system?), STD^2 = Covar, this is the
% covariance matrix for process noice (model noice) 
Q = diag([0.1^2, 0.2^2, (0.01*dt)^2, (0.01*dt)^2]); % No disturbance in position, 0.2m/s²*dt in disturbance in vx due to problem in ax (therefore *dt), low disturbance in ax



% Postteriori estimate covariance matrix, a measure of the estimated
% accuracy of the state estimate
P0 = diag([1, 1, 1, 1]);

 %% TODO 
 %make the model get measurement at 1 Hz and predict at 10Hz 
 %% Observation 
 for i = 1:30
 % Creating random noise
 temp = randn(2,1);
 tempQ = rand(4,1);
 v = sqrt(temp'*R*temp);
 w = sqrt(tempQ'*Q*tempQ);
 % Measurement z of the true state x (This is what comes out from position
 % sensor and velocity sensor?!)
 x1real = A*x0real' + B*u0' + w';
 z1 = H*x1real + v';
 % for next step
 x_real_plot = [x_real_plot, x1real(1)];
 y_real_plot = [y_real_plot, x1real(2)];
 x0real = x1real';
 % v is observation noise with covariance R


%% Predict step
x1 = A*x0' + B*u0';
% covariance matrix P updated
P1 = A*P0'*A' + Q;

%%  Update
% Inovation or measurement pre-fit residual
y1 = z1 - H*x0';
innovation = [innovation, y1];
%Innovation or pre-fit residual covariance
S1 = H*P1*H' + R;

%Optimal Kalman gain
K = P1*H'*inv(S1);

% The magic! Updated (a posteriori) state estimate
x1 = x1 + K*y1;

% Updated ( a posteriori) estimate covariance matrix
P1 = (diag([1,1,1,1])- K*H)*P1;

% Measurement post-fit residual
y1 = z1 -H*x1;
innovation = [innovation, y1];
% Getting ready for next step
x0=x1';
PO = P1; 

%% Saving for plotting

% Plot the true state x compared to the estimated state 
x_est_plot = [x_est_plot, x1(1)];
y_est_plot = [y_est_plot, x1(2)];
 end
%% plot
% The position
figure
plot(x_est_plot, y_est_plot, x_real_plot, y_real_plot)
axis equal; legend('estimate', 'real')

% The error/residual or innovation
figure
plot(innovation(1,:))
figure
plot(innovation(2,:))
