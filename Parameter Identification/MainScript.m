%% Preliminary operations
clear all
close all

cd(fileparts(matlab.desktop.editor.getActiveFilename));
addpath(genpath('..\'));
rmpath(genpath('..\Cestino'));

%% Load Parameters from saved files
load('LinearParameters.mat')
load('NonLinearParameters.mat')
% load('OldNonLinearParameters.mat')
Ts = 0.1;

%% Choose which parameters to use for the controllers
Parameters_PID = LinearParameters; %notazione PID aggiunta perch� i test reali erano fatti con i parametri vecchi, il PID deve rimanere quello vecchio per poter confrontare
Parameters = NonLinearParameters; %Suggested
Parameters_fake = struct('m1', 0.095, 'l1', 0.085, 'm2', 0.024, 'l2', 0.129, 'I2', 3.3282e-06, 'Kf2', 8.0e-05, 'g', 9.81, 'Km', 0.044032246988858, 'Rm', 8.143917370672490, 'Jm', 1.062203322222969e-05, 'bm', 2.221811683261875e-14, 'x1', 0.0425, 'x2', 0.0645, 'Jeq', 2.235981801246140e-04, 'Kf1', 8.0e-04, 'Ks', 0.00283);

%% Evaluate transfer functions
% Model 1: Downright linearized model without spring
[tf1_1,tf1_2] = tfModel1(Parameters);

% Model 2: Downright linearized model with spring
[tf2_1,tf2_2, Adown, Bdown, Cdown] = tfModel2(Parameters);
[tf2_1_PID,tf2_2_PID, Adown_PID, Bdown_PID, Cdown_PID] = tfModel2(Parameters_PID);

% Model 3: Experiment #01: Fixed Theta2, without spring 
tf3 = tfModel3(Parameters);

% Model 4: Experiment #01: Fixed Theta2, with spring 
tf4 = tfModel4(Parameters);

% Model 5: Upright linearized model without spring
[tf5_1,tf5_2] = tfModel5(Parameters);

% Model 6: Upright linearized model with spring
[tf6_1,tf6_2, Aup, Bup, Cup] = tfModel6(Parameters);

%% Load data to simulink
% Time series for model validation
[tsVs, omegaVs, amplVs] = sinFileToTS('TestTOT_0.5sin(1).mat');
[tsVwn, Vwn] = wnFileToTS('TestTOT_wn_2.mat');
[tsVsw, Vsw] = swFileToTS('sw_1V1Hz.mat');
[tsVstep, amplVstep, stopTime] = stepFileToTS('TestTOT_0.8sca_STATGAIN.mat');

% Time serie for motor model testing
[tsM, omegaM, amplM] = sinFileToTS('TestMotor_0.5sin(14).mat');

% Time serie for physical model testing
[tsP, omegaP, amplP] = sinFileToTS('TestTOT_2sin(11).mat'); 

% Time serie for the pendulum
tsVpend = PendFileToTS('Test2_vert(1).mat');