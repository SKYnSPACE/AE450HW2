clc; clear all; close all;

%% Part 1. DYNAMIC MODEL OF A SYSTEM
disp(repmat('=',1,80));
disp('Part 1. DYNAMIC MODEL OF A SYSTEM');

% Parameters and coefficients
gamma_e = 0;
alpha_e = 9.4;
V_0 = 178;
m = 17642;
I_yy = 165669;
c_barbar = 4.889;
S = 49.239;
rho = 0.3809;
g = 9.81;
D2R = pi/180;

% Derivatives
X_u = 0.0076;       Z_u = -0.7273;      M_u = 0.0340;
X_w = 0.0483;       Z_w = -3.1245;      M_w = -0.2169;
X_wdot = 0.0;       Z_wdot = -0.3997;   M_wdot = -0.5910;
X_q = 0.0;          Z_q = -1.2109;      M_q = -1.2732;
X_eta = 0.0618;     Z_eta = -0.3741;    M_eta = -0.5581;

% Non-dimensional form
mPrime = m/(1/2*rho*V_0*S);
I_yyPrime = I_yy/(1/2*rho*V_0*S*c_barbar);
W_e = V_0 * sin(alpha_e*D2R);
U_e = V_0 * cos(alpha_e*D2R);

M = [mPrime, -X_wdot*c_barbar/V_0, 0, 0 ;
    0, mPrime-Z_wdot*c_barbar/V_0, 0, 0 ;
    0, -M_wdot*c_barbar/V_0, I_yyPrime, 0;
    0, 0, 0, 1];
APrime = [X_u, X_w, X_q*c_barbar-mPrime*W_e, -mPrime*g*cos(alpha_e*D2R);
        Z_u, Z_w, Z_q*c_barbar+mPrime*U_e, -mPrime*g*sin(alpha_e*D2R);
        M_u, M_w, M_q*c_barbar 0;
        0, 0, 1, 0];
BPrime = [V_0*X_eta;
        V_0*Z_eta;
        V_0*M_eta;
        0];

% Summarized form
A = M\APrime
x_u = A(1,1);       z_u = A(2,1);       m_u = A(3,1);
x_w = A(1,2);       z_w = A(2,2);       m_w = A(3,2);
x_q = A(1,3);       z_q = A(2,3);       m_q = A(3,3);
x_theta = A(1,4);   z_theta = A(2,4);   m_theta = A(3,4);

B = M\BPrime
x_eta = B(1,1);     z_eta = B(2,1);     m_eta = B(3,1);

disp(repmat('=',1,80));

%% Part 2. TRANSIENT RESPONSE OF THE SYSTEM
disp(repmat('=',1,80));
disp('Part 2. TRANSIENT RESPONSE OF THE SYSTEM');

disp('** Eigenvalues **');
lambda = sort(eig(A),'ComparisonMethod','real')

disp('** Short Period **');
omega_s = sqrt(lambda(1)*lambda(2))
zeta_s = -(lambda(1)+lambda(2))/2/omega_s

disp('** Phugoid **');
omega_p = sqrt(lambda(3)*lambda(4))
zeta_p = -(lambda(3)+lambda(4))/2/omega_p

disp(repmat('=',1,80));

%% Part 3. MODEL APPROXIMATION
disp(repmat('=',1,80));
disp('Part 3. MODEL APPROXIMATION');

disp('** Short Period **');
A_s = [z_w z_q;
        m_w m_q];
B_s = [z_eta;
        m_eta];
% Calculation by Eigenvalues
reducedLambda_s = eig(A_s)
reducedOmega_s = sqrt(reducedLambda_s(1)*reducedLambda_s(2))
reducedZeta_s = -(reducedLambda_s(1)+reducedLambda_s(2))/2/reducedOmega_s
% % Calculation by (Cook, 6.20)
% reducedOmega_s = sqrt(m_q*z_w - m_w*U_e)
% reducedZeta_s = -(m_q + z_w)/2/reducedOmega_s

disp('** Phugoid **');
A_p = [x_u - x_w*(m_u*U_e - m_q*z_u)/(m_w*U_e - m_q*z_w), -g;
        (m_u*z_w - m_w*z_u)/(m_w*U_e - m_q*z_w), 0];
B_p = [x_eta - x_w*(m_eta*U_e - m_q*z_eta)/(m_w*U_e - m_q*z_w);
        (m_eta*z_w - m_w*z_eta)/(m_w*U_e - m_q*z_w)];
% Calculation by Eigenvalues
reducedLambda_p = eig(A_p)
reducedOmega_p = sqrt(reducedLambda_p(1)*reducedLambda_p(2))
reducedZeta_p = -(reducedLambda_p(1)+reducedLambda_p(2))/2/reducedOmega_p
% % Calculation by (Cook, 6.20)
% reducedOmega_s = sqrt(-g*z_u/U_e)
% reducedZeta_s = -x_u/2/reducedOmega_p

disp(repmat('=',1,80));

%% Part 4. SETTING UP A DESIGN CRITERIA
disp(repmat('=',1,80));
disp('Part 4. SETTING UP A DESIGN CRITERIA');

desiredOmega_s = 3
desiredZeta_s = 0.7

syms s;
eqn = s^2+2*desiredZeta_s*desiredOmega_s*s + desiredOmega_s.^2 == 0;
desiredLambda_s = eval(solve(eqn,s))

disp(repmat('=',1,80));

%% Part 5. CONTROLLER DESIGN
disp(repmat('=',1,80));
disp('Part 5. CONTROLLER DESIGN');

% Transfer Functions
disp('** Transfer Functions **');
num1 = [m_eta, -m_eta*z_w];
den1 = [1, -(m_q+z_w), (m_q*z_w - m_w*U_e)];
G_eta2q = tf(num1, den1)

num2 = [m_eta, -m_eta*z_w];
den2 = [1, -(m_q+z_w), (m_q*z_w - m_w*U_e), 0];
G_eta2theta = tf(num2, den2)

% Pitch Angle Feedback
figure(1)
set(gcf, 'Position',[0 100 400 800])

subplot(2,1,1)
title('Pitch Angle Feedback w/ Positive gain')
rlocus(G_eta2theta)
hold on; 
plot(real(desiredLambda_s),imag(desiredLambda_s),'rx','Markersize',10)
xlim([-3 3]); ylim([-3 3]); grid on; legend('Pole Locations w/r/t K>0','Desired Pole Locations');
hold off;

subplot(2,1,2)
title('Pitch Angle Feedback w/ Negative gain')
rlocus(-G_eta2theta)
hold on; 
plot(real(desiredLambda_s),imag(desiredLambda_s),'rx','Markersize',10)
xlim([-3 3]); ylim([-3 3]); grid on; legend('Pole Locations w/r/t K<0','Desired Pole Locations');
hold off;

% Pitch Angle and Rate Feedback
figure(2)
set(gcf, 'Position',[400 100 400 800])

subplot(2,1,1)
Kq = 0;
G_utheta2theta = minreal(G_eta2q / ((1 - Kq*G_eta2q)*tf([1 0],1)));
rlocus(-G_utheta2theta)
hold on; 
plot(real(desiredLambda_s),imag(desiredLambda_s),'rx','Markersize',10)
xlim([-4 3]); ylim([-4 4]); grid on; legend('Pole Locations w/r/t K_? (Kq=0)','Desired Pole Location');
hold off;

subplot(2,1,2)
Kq = 0.5;
G_utheta2theta = minreal(G_eta2q / ((1 - Kq*G_eta2q)*tf([1 0],1)));
rlocus(-G_utheta2theta)
hold on; 
plot(real(desiredLambda_s),imag(desiredLambda_s),'rx','Markersize',10)
xlim([-4 3]); ylim([-4 4]); grid on; legend('Pole Locations w/r/t K_? (Kq=0.5)','Desired Pole Location');
hold off;


disp('** Gain Tuning (Tune Kq, Ktheta near the magnitude==1, and the phase angle==+-180[deg]) **');
Kq = 0.754;
Ktheta=-1.41;
magnitude = abs(Ktheta * evalfr(G_eta2theta,desiredLambda_s(1)) - desiredLambda_s(1) * Kq * evalfr(G_eta2theta,desiredLambda_s(1)))
phase = angle(Ktheta * evalfr(G_eta2theta,desiredLambda_s(1)) - desiredLambda_s(1) * Kq * evalfr(G_eta2theta,desiredLambda_s(1)))

figure(3)
set(gcf, 'Position',[800 100 400 350])
G_utheta2theta = minreal(G_eta2q / ((1 - Kq*G_eta2q)*tf([1 0],1)))
rlocus(-G_utheta2theta)
hold on; 
plot(real(desiredLambda_s),imag(desiredLambda_s),'rx','Markersize',10)
xlim([-4 3]); ylim([-4 4]); grid on; legend('Pole Locations w/r/t K_? (Kq=0.754)','Desired Pole Location');
hold off;

disp(repmat('=',1,80));

%% Part 6. CLOSED-LOOP TIME RESPONSES
disp(repmat('=',1,80));
disp('Part 6. CLOSED-LOOP TIME RESPONSES');

x0 = [1;1;1;1];     % initial(perturbed) state
C = eye(4);         % output variables
D = zeros(4,1);     % no disturbances

disp('** Background simulation running... **');
sim('HW2Part6');
disp('** DONE! **');

figure(4)
set(gcf, 'Position',[300 50 800 800])

subplot(2,2,1)
plot(uOut.time, uOut.signals(1).values, uOut.time, uOut.signals(2).values)
title('u Response'); xlabel('Time[sec]'); ylabel('Amplitude'); 
legend('Open-loop response (no control)','Closed-loop response (feedback)');
grid on;

subplot(2,2,2)
plot(wOut.time, wOut.signals(1).values, wOut.time, wOut.signals(2).values)
title('w Response'); xlabel('Time[sec]'); ylabel('Amplitude'); 
legend('Open-loop response (no control)','Closed-loop response (feedback)');
grid on;

subplot(2,2,3)
plot(qOut.time, qOut.signals(1).values, qOut.time, qOut.signals(2).values)
title('q Response'); xlabel('Time[sec]'); ylabel('Amplitude'); 
legend('Open-loop response (no control)','Closed-loop response (feedback)');
grid on;

subplot(2,2,4)
plot(thetaOut.time, thetaOut.signals(1).values, thetaOut.time, thetaOut.signals(2).values)
title('theta Response'); xlabel('Time[sec]'); ylabel('Amplitude'); 
legend('Open-loop response (no control)','Closed-loop response (feedback)');
grid on;


disp(repmat('=',1,80));

%% Part 7. PHASE-LEAD COMPENSATOR
disp(repmat('=',1,80));
disp('Part 7. PHASE-LEAD COMPENSATOR');

disp('** Loading sisotool... **');

% Design form the scratch
% G = -G_eta2theta;
% sisotool(G);

% Load from the saved file
sisotool('HW2Part7.mat');

disp('** DONE! **');

disp(repmat('=',1,80));
