%% Preliminary operations
clear
close all
clc

%Add the whole folder in the path
cd(fileparts(matlab.desktop.editor.getActiveFilename));
addpath(genpath('../'));
rmpath(genpath('..\Cestino'));

fprintf('Executing Main Script...')
set(0, 'DefaultFigureVisible', 'off');
run('MainScript.m');
run('Vectors_GainPhaseAmpOme.m');
set(0, 'DefaultFigureVisible', 'on');
fprintf(' Done!\n\n')

%% open loop analysis from Vin to theta1
G1_num = tf2_1.num;
G1_den = tf2_1.den;
G1 = tf(G1_num, G1_den);

%% open loop analysis from Vin to theta2
G2_num = tf2_2.num;
G2_den = tf2_2.den;
G2 = tf(G2_num, G2_den);

%% plot open loop tf
figure(1);
subplot(1, 2, 1)
hold on
grid on
margin(G1);
title('Open Loop Vin->theta1');
grid on
hold off;

subplot(1, 2, 2)
hold on
grid on
margin(G2);
title('Open Loop Vin->theta2');
grid on
hold off;


%% approximation of bode diagram considering the non linear system (simulation)
w_messy_s = omegas_Sim;
mag_messy_s = gainsdB_Sim;
phase_messy_s = phases_Sim;
matrix_mag_s = [w_messy_s(:), mag_messy_s(:), phase_messy_s(:)];
sorted_matrix_mag_s = sortrows(matrix_mag_s);
w_s = sorted_matrix_mag_s(:, 1);
mag_s = sorted_matrix_mag_s(:, 2);
phase_s = sorted_matrix_mag_s(:, 3);


%% approximation of bode diagram considering the non linear system(reality)

w_messy_r = omegas;
mag_messy_r = gainsdB;
phase_messy_r = phases;
matrix_r = [w_messy_r(:), mag_messy_r(:), phase_messy_r(:)];
sorted_matrix_r = sortrows(matrix_r);
w_r = sorted_matrix_r(:, 1);
w_r_forPhases = sorted_matrix_r(1:end-2, 1);
mag_r = sorted_matrix_r(:, 2);
phase_r = sorted_matrix_r(1:end-2, 3);
phase_r(1) = -37;

%% plot
figure(2);
% subplot(2, 1, 1);
hold on
bodemag(G1, 'b');
grid on;
plot(w_r, mag_r, '-ro', w_s, mag_s, '-go');
legend('linearized model', 'real experiments', 'non linear model');
title('Magnitudes frequency validation');
hold off;

figure(3);
% subplot(2, 1, 1);
hold on
Gp = bodeplot(G1);
setoptions(Gp,'MagVisible','off');
grid on;
plot(w_r_forPhases, phase_r, '-ro', w_s, phase_s, '-go');
legend('linearized model', 'real experiments', 'non linear model');
title('Phases frequency validation');
hold off;
