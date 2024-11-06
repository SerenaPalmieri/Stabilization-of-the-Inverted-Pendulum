%% Preliminary operations
clear all
close all
clc

%Add the whole folder in the path
cd(fileparts(matlab.desktop.editor.getActiveFilename));
addpath(genpath('../../'));
rmpath(genpath('..\Cestino'));

fprintf('Executing Main Script...')
set(0, 'DefaultFigureVisible', 'off');
run('MainScript.m');
set(0, 'DefaultFigureVisible', 'on');
fprintf(' Done!\n\n')

%% Requirements
V_max = 10;

%% Original linearized tf
G_num = tf2_1.num;
G_den = tf2_1.den;
G = tf(G_num, G_den);
G_zeros = zero(G);
G_poles = pole(G);

%% Original linearized tf with the old parameters
G_num_PID = tf2_1_PID.num;
G_den_PID = tf2_1_PID.den;
G_PID = tf(G_num_PID, G_den_PID);
G_zeros_PID = zero(G_PID);
G_poles_PID = pole(G_PID);

%% PID controller frequency based
%da decommentare quando si rifanno i test reali con il nuovo valore del
%fitting

%L con i nuovi parametri 
L_gain = 150;
L_zeros = G_zeros;
L_poles = [0; G_poles(1); G_poles(2); -25];
[num_L, den_L] = zp2tf(L_zeros, L_poles, L_gain);
L = tf(num_L, den_L);

%L con i vecchi parametri
oldL_gain = 150;
oldL_zeros = G_zeros_PID;
oldL_poles = [0; G_poles_PID(1); G_poles_PID(2); -25];
[oldnum_L, oldden_L] = zp2tf(oldL_zeros, oldL_poles, oldL_gain);
oldL = tf(oldnum_L, oldden_L);

% figure(1);
% hold on;
% bodemag(G_PID);
% bodemag(oldL);
% bodemag(feedback(oldL, 1));
% title('bode vecchi parametri');
% hold off;
% 
% figure(2);
% hold on;
% bodemag(G);
% bodemag(L);
% bodemag(feedback(L, 1));
% title('bode nuovi parametri');
% hold off;

 % nuovo controllore
 Controller_tf = minreal(L/G);
 Controller_PID = pid(Controller_tf);
 [Kp, Ki, Kd, Tf] = piddata(Controller_PID);
 N = 1/Tf;

%vecchio controllore
% Controller_tf = minreal(oldL/G_PID);
% Controller_PID = pid(Controller_tf);
% [Kp, Ki, Kd, Tf] = piddata(Controller_PID);
% N = 1/Tf;
% newL = Controller_tf*G; %%L che si ottiene con il veccio controllo ma i nuovi parametri nel sistema

% figure(3);
% hold on;
% bodemag(G);
% bodemag(newL);
% bodemag(feedback(newL, 1));
% title('bode nuovi parametri ma PID vecchi');
% hold off;

%% Plots
figure;
subplot(1, 2, 1)
hold on
grid on
bode(G);
margin(G);
bode(L);
bode(Controller_tf);
legend('G(s)-uncontrolled system', 'L(s)-open loop controlled system', 'C(s)-controller');
title('Transfer functions')
hold off;

subplot(1, 2, 2)
hold on
grid on
bode(G);
bode(feedback(L, 1));
legend('G(s)-uncontrolled system', 'Closed loop controlled system');
title('Uncontrolled vs controlled system')
hold off

figure
hold on
bode(feedback(L, 1))
grid on
%% Save the experimental data (you can run just this part to change experiment)
% Time serie for the control
[outputTS, inputTS] = ContrFileToTS(strcat('testCtrBeam_0.8Step', '.mat'));
% [outputTS, inputTS] = ContrFileToTS(strcat('testCtrBeam_1Sin(3.5)', '.mat'));