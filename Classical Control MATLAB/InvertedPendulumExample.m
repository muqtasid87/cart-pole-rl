%% Control of an Inverted Pendulum on a Cart
% This uses |systune| to control an inverted pendulum on a cart.


%% Pendulum/Cart Assembly
% The cart/pendulum assembly is depicted in Figure 1 and modeled in
% Simulink(R) using Simscape(TM) Multibody(TM).
%
% <<../pendulum1.png>>
%
% *Figure 1: Inverted pendulum on a cart*
%
% <<../pendulum2.png>>
%
% *Figure 2: Simscape Multibody model*
%
% This system is controlled by exerting a variable force $F$ on the cart.
% The controller needs to keep the pendulum upright while moving the cart
% to a new position or when the pendulum is nudged forward (impulse
% disturbance $dF$).

%% Control Structure
% The upright position is an unstable equilibrium for the inverted
% pendulum. The unstable nature of the plant makes the control task more
% challenging. For this example, the following two-loop control
% structure is used:

open_system('rct_pendulum.slx')
set_param('rct_pendulum','SimMechanicsOpenEditorOnUpdate','off');

%%
% The inner loop uses a second-order state-space controller to stabilize
% the pendulum in its upright position ($\theta$ control), while the outer
% loop uses a Proportional-Derivative (PD) controller to control the cart
% position. 

%% Design Requirements
% Use |TuningGoal| requirements to specify the desired closed-loop
% behavior. Specify a response time of 3 seconds for tracking a setpoint
% change in cart position $x$.

% Tracking of x command
req1 = TuningGoal.Tracking('xref','x',3);

%%
% To adequately reject impulse disturbances $dF$ on the tip of
% the pendulum, use an LQR penalty of the form
%
% $$ \int_0^\infty (16 \theta^2(t) + x^2(t) + 0.01 F^2(t)) dt $$
%
% that emphasizes a small angular deviation $\theta$ and limits the control
% effort $F$.

% Rejection of impulse disturbance dF
Qxu = diag([16 1 0.01]);
req2 = TuningGoal.LQG('dF',{'Theta','x','F'},1,Qxu);

%%
% For robustness, require at least 6 dB of gain margin and 40 degrees
% of phase margin at the plant input.

% Stability margins
req3 = TuningGoal.Margins('F',6,40);

%%
% Finally, constrain the damping and natural frequency of the closed-loop
% poles to prevent jerky or underdamped transients.

% Pole locations
MinDamping = 0.5;
MaxFrequency = 45;
req4 = TuningGoal.Poles(0,MinDamping,MaxFrequency);

%% Control System Tuning
% The closed-loop system is unstable for the initial values of the PD and
% state-space controllers (1 and $2/s$, respectively). We use
% |systune| to jointly tune these two controllers. Use the |slTuner|
% interface to specify the tunable blocks and register the plant
% input |F| as an analysis point for measuring stability margins.

ST0 = slTuner('rct_pendulum',{'Position Controller','Angle Controller'});
addPoint(ST0,'F');

%%
% Next, use |systune| to tune the PD and state-space controllers subject
% to the performance requirements specified above. Optimize the
% tracking and disturbance rejection performance (soft requirements)
% subject to the stability margins and pole location constraints
% (hard requirements).

rng(0)
Options = systuneOptions('RandomStart',5);
[ST, fSoft] = systune(ST0,[req1,req2],[req3,req4],Options);

%%
% The best design achieves a value close to 1 for the soft requirements
% while satisfying the hard requirements (|Hard|<1). This means that
% the tuned control system nearly achieves the target performance for
% tracking and disturbance rejection while satisfying the stability margins
% and pole location constraints.

%% Validation
%%
% These plots confirm that the first two requirements are nearly satisfied
% while the last two are strictly enforced. Next, plot the responses to
% a step change in position and to a force impulse on the cart.

T = getIOTransfer(ST,{'xref','dF'},{'x','Theta'});
figure('Position',[100   100   650   420]);
subplot(121), step(T(:,1),10)
title('Tracking of set point change in position')
subplot(122), impulse(T(:,2),10)
title('Rejection of impulse disturbance')

%%
% The responses are smooth with the desired settling times. Inspect the
% tuned values of the controllers.

C1 = getBlockValue(ST,'Position Controller')

%%

C2 = zpk(getBlockValue(ST,'Angle Controller'))



%%
% To complete the validation, upload the tuned values to Simulink and
% simulate the nonlinear response of the cart/pendulum assembly.
% A video of the resulting simulation appears below.

writeBlockValue(ST)


%%
set_param('rct_pendulum','SimMechanicsOpenEditorOnUpdate','on');

%% Calculate Response Metrics
%% Classical Controller Disturbance Rejection Analysis
%
% This script analyzes the performance of the classical controller 
% for an inverted pendulum system from 'rct_pendulum.slx' when 
% subjected to an impulse disturbance.
%
% It calculates and displays performance metrics (Overshoot, Rise Time, 
% Settling Time, MSE) and plots the system response.
%
% PREREQUISITES:
% 1. MATLAB with Simulink, Control System Toolbox, and Simscape Multibody.
% 2. The Simulink model file 'rct_pendulum.slx' must be in the current 
%    MATLAB path.

clear; clc; close all;

%% SECTION 1: Classical Controller Analysis
disp('--- Analyzing Classical Controller ---');

try
    % Load the Simulink model for the classical controller
    model_classical = 'rct_pendulum';
    open_system(model_classical);

    % Use the same setup as the example to tune the controller
    % This ensures the controller is ready for analysis.
    % Create an slTuner interface
    STO = slTuner(model_classical, {'Position Controller', 'Angle Controller'});
    addPoint(STO, 'F');

    % Define tuning goals as per the example
    req1 = TuningGoal.Tracking('xref', 'x', 3);
    Qxu = diag([16 1 0.01]); 
    req2 = TuningGoal.LQG('dF', {'Theta', 'x', 'F'}, 1, Qxu);
    req3 = TuningGoal.Margins('F', 6, 40);
    req4 = TuningGoal.Poles(0, 0.5, 45);

    % Tune the controller
    rng(0); % for reproducibility
    Options = systuneOptions('RandomStart', 5);
    [ST, ~] = systune(STO, [req1, req2], [req3, req4], Options);

    % Get the transfer function from the disturbance 'dF' to the outputs
    % 'x' (cart position) and 'Theta' (pole angle).
    T_classical = getIOTransfer(ST, {'dF'}, {'x', 'Theta'});

    % Simulate the impulse response for 10 seconds
    [y_classical, t_classical] = impulse(T_classical, 10);

    % Extract data for cart position and pole angle
    data_classical_x = y_classical(:, 1);
    data_classical_theta = y_classical(:, 2);

    disp('Classical controller analysis complete.');

catch ME
    disp('ERROR in Classical Controller Analysis:');
    disp('Please ensure ''rct_pendulum.slx'' is in the path and you have the required toolboxes.');
    disp(ME.message);
    return;
end

%% SECTION 2: Performance Calculation and Display
disp('--- Calculating Metrics and Plotting Results ---');

% --- Calculate metrics ---
metrics_classical_x = calculatePerformanceMetrics(t_classical, data_classical_x);
metrics_classical_theta = calculatePerformanceMetrics(t_classical, data_classical_theta);

% --- Display results in a formatted table ---
fprintf('\n\n===== CLASSICAL CONTROLLER PERFORMANCE =====\n');
fprintf('%-28s | %-25s\n', 'Metric', 'Value');
fprintf([repmat('-', 1, 55) '\n']);
fprintf('CART POSITION (x)\n');
fprintf('%-28s | %-25.4f\n', 'Overshoot (Peak Deviation)', metrics_classical_x.Overshoot);
fprintf('%-28s | %-25.4f\n', 'Rise Time (s)', metrics_classical_x.RiseTime);
fprintf('%-28s | %-25.4f\n', 'Settling Time (s)', metrics_classical_x.SettlingTime);
fprintf('%-28s | %-25.6f\n', 'Mean Squared Error (MSE)', metrics_classical_x.MSE);
fprintf([repmat('-', 1, 55) '\n']);
fprintf('PENDULUM ANGLE (theta, rad)\n');
fprintf('%-28s | %-25.4f\n', 'Overshoot (Peak Deviation)', metrics_classical_theta.Overshoot);
fprintf('%-28s | %-25.4f\n', 'Rise Time (s)', metrics_classical_theta.RiseTime);
fprintf('%-28s | %-25.4f\n', 'Settling Time (s)', metrics_classical_theta.SettlingTime);
fprintf('%-28s | %-25.6f\n', 'Mean Squared Error (MSE)', metrics_classical_theta.MSE);
fprintf('==============================================\n\n');


%% SECTION 3: Plotting Results
figure('Name', 'Classical Controller: Impulse Disturbance Rejection');

% Plot Cart Position (x)
subplot(2, 1, 1);
plot(t_classical, data_classical_x, 'b-', 'LineWidth', 1.5);
grid on;
title('Cart Position (x) Response to Impulse Disturbance');
xlabel('Time (seconds)');
ylabel('Position (m)');
legend('Classical Controller', 'Location', 'northeast');

% Plot Pendulum Angle (theta)
subplot(2, 1, 2);
plot(t_classical, data_classical_theta, 'b-', 'LineWidth', 1.5);
grid on;
title('Pendulum Angle (theta) Response to Impulse Disturbance');
xlabel('Time (seconds)');
ylabel('Angle (radians)');
legend('Classical Controller', 'Location', 'northeast');


%% --- Helper Function for Metric Calculation ---
function metrics = calculatePerformanceMetrics(time, data, settling_criterion)
    % Calculates key performance metrics for a response curve.
    if nargin < 3
        settling_criterion = 0.02; % Default: 2% settling time
    end
    time = time(:); 
    data = data(:);
    
    % --- Overshoot (Peak Deviation) ---
    [peak_value, idx_peak] = max(abs(data));
    metrics.Overshoot = peak_value;
    
    % --- Rise Time (10% to 90% of peak) ---
    rising_time = time(1:idx_peak);
    rising_data = abs(data(1:idx_peak));
    
    idx_10 = find(rising_data >= 0.1 * peak_value, 1, 'first');
    idx_90 = find(rising_data >= 0.9 * peak_value, 1, 'first');
    
    if ~isempty(idx_10) && ~isempty(idx_90)
        t_10 = rising_time(idx_10);
        t_90 = rising_time(idx_90);
        metrics.RiseTime = t_90 - t_10;
    else
        metrics.RiseTime = NaN; % Not applicable
    end

    % --- Settling Time ---
    settling_band = settling_criterion * peak_value;
    last_out_of_band_idx = find(abs(data) > settling_band, 1, 'last');
    
    if isempty(last_out_of_band_idx) || last_out_of_band_idx == length(time)
        metrics.SettlingTime = NaN;
    else
        metrics.SettlingTime = time(last_out_of_band_idx + 1);
    end
    
    % --- Mean Squared Error (MSE) ---
    metrics.MSE = mean(data.^2);
end

