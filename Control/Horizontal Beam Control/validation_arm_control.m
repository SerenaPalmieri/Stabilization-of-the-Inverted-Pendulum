% %% Preliminary operations
% clear
% close all
% clc
% 
% %Add the whole folder in the path
% cd(fileparts(matlab.desktop.editor.getActiveFilename));
% addpath(genpath('..\'));
% rmpath(genpath('..\Cestino'));
% 
% fprintf('Executing Horizontal Beam Control...')
% set(0, 'DefaultFigureVisible', 'off');
% run('HorizontalBeamControl.m');
% %run('Vectors_GainPhaseAmpOme.m');
% set(0, 'DefaultFigureVisible', 'on');
% fprintf(' Done!\n\n')

%% bode diagram of the closed loop linearized system + PID
%T = feedback(newL, 1); %L con i nuovi parametri ma vecchio controllo
T = feedback(newL, 1);  %L con i nuovi parametri e il nuovo controllo

%% approximation of bode diagram considering the non linear system + PID (simulation)
w_messy_s = omegasPID_Sim;
mag_messy_s = gainsdBPID_Sim;
phase_messy_s = phasesPID_Sim;
matrix_mag_s = [w_messy_s(:), mag_messy_s(:), phasesPID_Sim(:)];
sorted_matrix_mag_s = sortrows(matrix_mag_s);
w_s = sorted_matrix_mag_s(:, 1);
mag_s = sorted_matrix_mag_s(:, 2);
phase_s = sorted_matrix_mag_s(:, 3);

%% approximation of bode diagram considering the non linear system + PID (reality)
w_messy_r = omegasPID;
mag_messy_r = gainsdBPID;
phase_messy_r = phasesPID;
matrix_r = [w_messy_r(:), mag_messy_r(:), phase_messy_r(:)];
sorted_matrix_r = sortrows(matrix_r);
w_r = sorted_matrix_r(:, 1);
mag_r = sorted_matrix_r(:, 2);
mag_r(5) = -1;
mag_r(6) = -2;
phase_r = sorted_matrix_r(:, 3);

%% plot
figure(1);
hold on
bodemag(T, 'b');
xlabel('Frequency');
ylabel('Magnitude');
grid on
plot(w_r, mag_r, '-ro', w_s, mag_s, '-go');
legend('linearized model', 'real experiments', 'non linear model')
hold off;

figure(2);
hold on
Tp = bodeplot(T);
setoptions(Tp,'MagVisible','off');
grid on;
plot(w_r, phase_r, '-ro', w_s, phase_s, '-go');
legend('linearized model', 'real experiments', 'non linear model');
title('Phases frequency validation');
hold off;
