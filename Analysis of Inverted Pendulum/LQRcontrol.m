clc; clear; close all;

%% System Matrices
A = [0 0 1 0;
     0 0 0 1;
     0 123.98 -1.577 0;
     0 111.62 -0.7253 0];

B = [0; 0; 56.38; 25.98];
C = eye(4);
D = zeros(4,1);

%% LQR Design
Q = diag([10 200 1 1]);   % Penalize pendulum angle strongly
R = 1;

K = lqr(A,B,Q,R);

disp('LQR Gain Matrix K:');
disp(K);

%% Closed-loop system
Acl = A - B*K;

disp('Closed-loop Eigenvalues:');
disp(eig(Acl));

sys_cl = ss(Acl,B,C,D);

%% Step Response
figure;
step(sys_cl);
title('Closed-Loop Step Response');
xlabel('Time (s)');
ylabel('States');
grid on;

saveas(gcf,'step_response.png');

%% Impulse Response
figure;
impulse(sys_cl);
title('Closed-Loop Impulse Response');
grid on;

saveas(gcf,'impulse_response.png');

%% Open-loop Bode Plot
sys = ss(A,B,C,D);

figure;
bode(sys);
title('Open-Loop Frequency Response');
grid on;

saveas(gcf,'bode_plot.png');

%% Observer Design
C_obs = [1 0 0 0;
         0 1 0 0];

poles = [-10 -11 -12 -13];

L = place(A',C_obs',poles)';

disp('Observer Gain Matrix L:');
disp(L);