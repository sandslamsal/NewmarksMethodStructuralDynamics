clc
clear

% Modal parameters
wn = 6.283;   % Eigen-frequency (rad/s)
eta = 0.05;   % Damping ratio
wd = wn * sqrt(1 - eta^2); % Damped eigen frequency (rad/s)
M = 0.2533;   % Mass
K = wn^2 * M; % Stiffness
C = 2 * eta * M * wn; % Damping

% Time domain
t = 0:0.05:10;  % Time from 0 to 10 with a step of 0.05
dt = 0.05;      % Time step

% Harmonic force parameters
F0 = 10;        % Amplitude of force
w = wd;         % Pulsation of the harmonic force
F = F0 * sin(pi() * t / 0.6); % Expression of the harmonic force

% Newmark's method parameters (Constant Average Acceleration)
gamma = 1/2;
beta = 1/4;

% Initialize
u = zeros(size(t));
v = zeros(size(t));
a = zeros(size(t));

% Initial conditions
u(1) = 0;
v(1) = 0;
a(1) = (F(1) - C * v(1) - K * u(1)) / M;

% Precompute coefficients
a1 = 1 / (beta * (dt^2)) * M + gamma / (beta * dt) * C;
a2 = 1 / (beta * dt) * M + (gamma / beta - 1) * C;
a3 = (1 / (2 * beta) - 1) * M + dt * (gamma / (2 * beta) - 1) * C;
k_hat = K + a1;

% Initialize arrays for table
p_hat_arr = zeros(size(t));
a_new_arr = zeros(size(t));
v_new_arr = zeros(size(t));
u_new_arr = zeros(size(t));

for i = 1:length(t)
    if i == length(t)
        % Don't calculate new values on the last iteration
        p_hat_arr(i) = F(i) + a1 * u(i) + a2 * v(i) + a3 * a(i);
        a_new_arr(i) = a(i);
        v_new_arr(i) = v(i);
        u_new_arr(i) = u(i);
    else
        % Calculate new values using Newmark's method
        p_hat = F(i) + a1 * u(i) + a2 * v(i) + a3 * a(i);
        u_new = p_hat / k_hat;
        v_new = gamma / (beta * dt) * (u_new - u(i)) + (1 - gamma / beta) * v(i) + dt * (1 - gamma / (2 * beta)) * a(i);
        a_new = 1 / (beta * (dt^2)) * (u_new - u(i)) - 1 / (beta * dt) * v(i) - (1 / (2 * beta) - 1) * a(i);

        % Store values for table
        p_hat_arr(i) = p_hat;
        a_new_arr(i) = a_new;
        v_new_arr(i) = v_new;
        u_new_arr(i) = u_new;

        % Update for the next iteration
        u(i + 1) = u_new;
        v(i + 1) = v_new;
        a(i + 1) = a_new;
    end
end

% Convert all arrays to column vectors
t = t(:);
F = F(:);
p_hat_arr = p_hat_arr(:);
a_new_arr = a_new_arr(:);
v_new_arr = v_new_arr(:);
u_new_arr = u_new_arr(:);

% Create a table for better readability
T = table(t, F, p_hat_arr, a_new_arr, v_new_arr, u_new_arr, ...
    'VariableNames', {'Time', 'Force', 'Predicted_Force', 'Acceleration', 'Velocity', 'Displacement'});
disp('Results Table:');
disp(T);

% Plotting the results
figure;

subplot(3,1,1);
plot(t, u, 'b-', 'LineWidth', 1.5);
title('Displacement');
ylabel('Displacement (m)');
xlabel('Time (s)');
grid on;

subplot(3,1,2);
plot(t, v, 'r-', 'LineWidth', 1.5);
title('Velocity');
ylabel('Velocity (m/s)');
xlabel('Time (s)');
grid on;

subplot(3,1,3);
plot(t, a, 'g-', 'LineWidth', 1.5);
title('Acceleration');
ylabel('Acceleration (m/s^2)');
xlabel('Time (s)');
grid on;
