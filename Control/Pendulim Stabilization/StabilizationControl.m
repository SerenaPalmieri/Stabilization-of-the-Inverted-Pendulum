%% Preliminary operations
clear all
close all
clc

% Add the whole folder in the path
cd(fileparts(matlab.desktop.editor.getActiveFilename));
addpath(genpath('../../'));
rmpath(genpath('..\Cestino'));

fprintf('Executing Main Script...')
set(0, 'DefaultFigureVisible', 'off');
run('MainScript.m');
set(0, 'DefaultFigureVisible', 'on');
fprintf(' Done!\n\n')

%% Matrici sistema linearizzato in posizione alta
A = Aup;
B = Bup;
C = Cup;
D = [0;0];
%% Analisi del sistema
fprintf('------------------\n System analisys\n------------------\n')
fprintf('Observability and controllability:\n')
% Osservabilità
Ob = obsv(A,C);
fprintf('The rank of the observability matrix of the system is %d\n', rank(Ob));
fprintf('The rank of the observability matrix if C = [1 0 0 0] is %d\n', rank(obsv(A,[1,0,0,0])));

% Controllabilità
Con = ctrb(A,B);
fprintf('The rank of the controllability matrix of the system is %d\n', rank(Con));

fprintf('\n')

%invariant zeros del sistema:
fprintf('Invariant zeros:\n')
fprintf('...of the base system (if any is present): %d\n', tzero(A,B,C,D))
fprintf('...of the system with C = [1 0 0 0] (if any is present): %d\n', tzero(A,B,[1,0,0,0],0))
fprintf('...of the system with C = [0 1 0 0] (if any is present) : %d\n', tzero(A,B,[0,1,0,0],0))

fprintf('\n\n')

%% Osservatore 
L = place(A',C',[-60, -65, -58, -63])';
Ao = A-L*C;
Bo = [B,L];
Co = eye(4);
Co_real = [0,0,1,0;
           0,0,0,1];
sysObserver = ss(Ao,Bo,Co,0);
IC = [0,0,0.3,0];
%% Pole placement con integratore
% inserisco integratore, ne posso mettere solo uno perchè ho un unico ingresso
% scelgo theta1 (su theta2 non è possibile)
Cint = [1,0,0,0];
Ai = [A, [0;0;0;0;];
    -Cint, 0];
Bi = [ B; 0];
Ci = [0,0,1,0,0];
Coi = ctrb(Ai,Bi);

fprintf('--------------------------------------\n Pole placement + integrator analisys\n--------------------------------------\n')
fprintf('The rank of the controllability matrix of the system with integrator on theta1 is %d\n', rank(Coi));
fprintf('\n\n')

% Provare su sist reale per capire quanto veloci servano
Kpp_i = place(Ai, Bi, [-8 -8.1 -8.2 -8.5 -9]);   
Kpp_state = Kpp_i(1:4);
Kpp_int = Kpp_i(5);

%% Pole placement base
Kpp = place(A, B, [-0.25 -11 -13 -12]);

% Kpp funzionante:
% [-0.573521645205115,11.489119401008754,-0.474140627745285,0.812846237979813]

% % Provare su sist reale per capire quanto veloci servano
% Kpp_i = place(Ai, Bi, [-2 -4 -5 -6 -8]);   
% Kpp_state = Kpp_i(1:4);
% Kpp_int = Kpp_i(5);
% 
% %% Pole placement base
% Kpp = place(A, B, [-2 -8 -10 -15]);
% Prendere Kpp_int per integratore e Kpp anzichè Kpp_state

%% Kalman filter
% so che A,C è osservabile e A,B contr
% rumore di misura messo di default, rumore su input va messo come input
sys = ss(A,[B, eye(4)],C,zeros(2,5),'InputName',{'u' 'v1', 'v2', 'v3', 'v4'},'OutputName','y');
alfa = 10000;

% metodo per recuperare robustezza per LQG ipotesi però è numInput=numOutput
% e zeri con parte reale negativa, le due cose assieme sono impossibili
% Q = alfa*(B*B'); % covariance of white noise on input (dim = num noises on state)
% R = eye(2);% covariance of white noise on output (mat dim 2 perchè 2 output)

% alternativa è stimare R e Q, stabilità è assicurata, robustezza no
R = 0.002 * eye(2); % sensibilità encoder
%Bkal = [0;0;0.026;0.1039];
%fprintf('The rank of the controllability matrix using Bkal = [0;0;5;5]: %d\n', rank(ctrb(A, Bkal)));
%Q = Bkal*Bkal'; % perchè so che (A,B) è controllabile
%Q = zeros(4,4); 
%Q(3,3) = 100;
%Q(4,4) = 100;
Q = 100*[0,0,0,0;
    0,0,0,0;
    0,0,0.026,-0.02;
    0,0,-0.02,0.1039];
N = 0; % covariance of the 2, assume =0
% N = [0.01,0.01;
%     0.01,0.01;
%     1,0.01;
%     0.01,1;]
[kalmf,L2,P] = kalman(sys,Q,R,N);
%kalm = returned as a state-space (ss) model. The resulting estimator has
% inputs [u;y] and outputs [y_est, x_est]

%% LQ control
% so che A,B reachable
% Q = C'*C; % perchè so che A,C è obs. magari fattore per scalare
% R = 1;
Q = 10*[1 10 0.001 0.001].'*[1 10 0.001 0.001];
R = 0.1;
[Klq, S, polesLQ] = lqr(A,B,Q,R);

%% LQ enlarged sys
fprintf('-----------------------------\n LQ enlarged system analisys\n-----------------------------\n')
fprintf('Observability and controllability:\n')
fprintf('The rank of the observability matrix of the system with C = [1,0,0,0,0] is %d\n', rank(obsv(Ai,[1,0,0,0,0]))); % non obs, non posso usare C=[10000] in Q
fprintf('The rank of the observability matrix of the system with C = [0,1,0.001,0.001,1] is %d\n', rank(obsv(Ai,[0,1,0.001,0.001,1]))); % obs, per forza 1 in integratore
Q = [0,10,0.001,0.001,10].'*[0,10,0.001,0.001,10];
R = 0.1;
[Klqi, Si, polesLQi] = lqr(Ai,Bi,Q,R);
Klqi_int = Klqi(5);
Klqi_state = Klqi(1:4);

%% calcolo close loop behaviour
Acl_pp = A-B*Kpp_state;
ClosedPP = ss(Acl_pp, B,[1,0,0,0],0);
ClosedPP_th2 = ss(Acl_pp, B,[0,1,0,0],0);
integratorePP = tf([-Kpp_int],[1, 0]);
sist_tot_PP = feedback(integratorePP*ClosedPP,1); % quando si usano derivate

Acl_lq = A-B*Klqi_state;
ClosedLQ = ss(Acl_lq, B,[1,0,0,0],0);
ClosedLQ_th2 = ss(Acl_lq, B,[0,1,0,0],0);
integratoreLQ = tf([-Klqi_int],[1, 0]);
sist_tot_LQ = feedback(integratoreLQ*ClosedLQ,1); % quando si usano derivate

% PP con observer lungberg
CperOb = [1,0,0,0;
          0,1,0,0;
          0,0,0,0];
DperOb = [0;0;1];
systemPerOb = ss(A,B,CperOb,DperOb);
ClosedPP_obs = feedback(systemPerOb, Kpp_state*sysObserver);
ClosedPP_obs_theta1 = ss(ClosedPP_obs.A, ClosedPP_obs.B, [1,0,0,0,0,0,0,0],0);
sist_tot_PP_obs = feedback(integratorePP*ClosedPP_obs_theta1,1);

% LQ con observer lungberg
ClosedLQ_obs = feedback(systemPerOb, Klqi_state*sysObserver);
ClosedLQ_obs_theta1 = ss(ClosedLQ_obs.A, ClosedLQ_obs.B, [1,0,0,0,0,0,0,0],0);
sist_tot_LQ_obs = feedback(integratoreLQ*ClosedLQ_obs_theta1,1);

% PP con kalman
system_kalman = ss(kalmf.A,kalmf.B, [1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1], zeros(4,3));
ClosedPP_kal = feedback(systemPerOb, Kpp_state*system_kalman);
ClosedPP_kal_theta1 = ss(ClosedPP_kal.A, ClosedPP_kal.B, [1,0,0,0,0,0,0,0],0);
sist_tot_PP_kal = feedback(integratorePP*ClosedPP_kal_theta1,1);

% LQ con kalman
ClosedLQ_kal = feedback(systemPerOb, Klqi_state*system_kalman);
ClosedLQ_kal_theta1 = ss(ClosedLQ_kal.A, ClosedLQ_kal.B, [1,0,0,0,0,0,0,0],0);
sist_tot_LQ_kal = feedback(integratoreLQ*ClosedLQ_kal_theta1,1);

%% time series lyapunov
file = load('swingup_energy.mat')
fieldNames = fieldnames(file);
data = file.(fieldNames{1});
theta1 = data(2,:);
theta2 = data(3,:);
time = data(1,:);
theta1_lyap = timeseries(theta1, time);
theta2_lyap = timeseries(theta2, time);

%% time series per observer validation
file  = load('TestSCTR_PPobs_dist0.8Pulse_ref0.mat')
fieldNames = fieldnames(file);
data = file.(fieldNames{1});
theta1 = data(2,2699:end);
theta2 = data(3,2699:end);
time = data(1,1:end-2699+1);
input = data(5,2699:end);
theta1_PP_obs = timeseries(theta1, time);
theta2_PP_obs = timeseries(theta2, time);
input_PP_obs = timeseries(input, time);

%% time series kalman validation 
file  = load('TestSCTR_LQkal_dist0.8Pulse_ref0.mat')
fieldNames = fieldnames(file);
data = file.(fieldNames{1});
theta1 = data(2,1970:end);
theta2 = data(3,1970:end);
time = data(1,1:end-1970+1);
input = data(5,1970:end);
theta1_LQ_kal = timeseries(theta1, time);
theta2_LQ_kal = timeseries(theta2, time);
input_LQ_kal = timeseries(input, time);

%% time series PP time validation
file = load('TestSCTR_PPder_dist0.8Pulse_1step.mat')
fieldNames = fieldnames(file);
data = file.(fieldNames{1});
theta1 = data(2,2758:end);
theta2 = data(3,2758:end);
time = data(1,1:end-2758+1);
reference = data(4,2758:end);
disturbo = data(6,2758:end);
theta1_PP_der = timeseries(theta1, time);
theta2_PP_der = timeseries(theta2, time);
reference_PP_der = timeseries(reference, time);
disturbo_PP_der = timeseries(disturbo,time);

%% time series LQ time validation
file = load('TestSCTR_LQder_dist0.8Pulse_1step.mat')
fieldNames = fieldnames(file);
data = file.(fieldNames{1});
theta1 = data(2,2795:end);
theta2 = data(3,2795:end);
time = data(1,1:end-2795+1);
reference = data(4,2795:end);
disturbo = data(6,2795:end);
theta1_LQ_der = timeseries(theta1, time);
theta2_LQ_der = timeseries(theta2, time);
reference_LQ_der = timeseries(reference, time);
disturbo_LQ_der = timeseries(disturbo,time);
