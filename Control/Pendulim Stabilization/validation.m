%% Preliminary operations
clear
close all
clc

% Add the whole folder in the path
cd(fileparts(matlab.desktop.editor.getActiveFilename));
addpath(genpath('..\..\'));
rmpath(genpath('..\Cestino'));

fprintf('Executing Stabilization control script...')
set(0, 'DefaultFigureVisible', 'off');
run('StabilizationControl.m');
set(0, 'DefaultFigureVisible', 'on');
fprintf(' Done!\n\n')

PP = 0;
LQ = 1;

% frequencies = [0.1 0.3 0.5 1 3 5 10 30];
frequencies = logspace(log10(0.1),log10(30),50);
Amplitude = 1;

%% Simulation results
ppSim = getSimulationResults(PP, frequencies, Amplitude);
lqSim = getSimulationResults(LQ, frequencies, Amplitude);

%% Experimental results
expPPder = getPlotData('PPder');
expPPobs = getPlotData('PPobs');
expLQder = getPlotData('LQder');
expLQkal = getPlotData('LQkal');

%% Accorgimenti
expLQkal(2, 7) = -35;
expLQkal(3, 7) = 35;

%% Plots
plotResults(expPPder, ppSim.derivative, 'Pole Placement, derivatives')
plotResults(expPPobs, ppSim.observer, 'Pole Placement, Luenberg observer')
plotResults(expLQder, lqSim.derivative, 'Linear Quadratic Control, derivatives')
plotResults(expLQkal, lqSim.kalman, 'Linear quadratic control, Kalman filter')

%% Helper functions
function plotResults(realSystem, nonLinearSystem, Title)
    figure
    sgtitle(Title);

    % Magnitude plot
    subplot(2,1,1)
    hold on
    xlabel('Frequency (rad/s)');
    ylabel('Magnitude (dB)');
    plot(realSystem(1,:), realSystem(2,:), 'rx', 'MarkerSize', 7)
    plot(nonLinearSystem(1,:), nonLinearSystem(2,:), 'b');
    legend('Real experiments', 'Non-linear model')
    grid on
    xscale('log')
    xlim([0.04 100])
    ylim([-100 50])
    xlabel('Frequency (rad/s)')
    ylabel('Angle (deg)')
    hold off;
    
    % Phase plot
    subplot(2,1,2)
    hold on
    plot(realSystem(1,:), realSystem(3,:), 'rx',  'MarkerSize', 7)
    plot(nonLinearSystem(1,:), nonLinearSystem(3,:), 'b');
    legend('Real experiments', 'Non-linear model')
    grid on;
    xscale('log')
    xlim([0.04 100])
    xlabel('Frequency (rad/s')
    ylabel('Angle (deg)')
    hold off;
end