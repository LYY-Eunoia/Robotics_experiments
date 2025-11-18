function [t_strike, q_strike, qd_strike, qdd_strike] = PlanStrikeAction(q_approach, config, strike_angle, strike_duration, sample_time)
    % PlanStrikeAction - Plan the striking action for a single note
    %
    % Input:
    %   q_approach - 1x6 joint angles at approach position (before strike)
    %   config - Robot configuration structure
    %   strike_angle - Rotation angle for joint 5 in radians (default: pi/2)
    %   strike_duration - Duration of strike motion in seconds (default: 0.1s)
    %   sample_time - Sampling time in seconds (default: 0.001s)
    %
    % Output:
    %   t_strike - Time vector for strike action
    %   q_strike - Joint angles during strike (Nx6 matrix)
    %   qd_strike - Joint velocities during strike (Nx6 matrix)
    %   qdd_strike - Joint accelerations during strike (Nx6 matrix)
    
    if nargin < 3
        strike_angle = pi/2;  % 90 degrees
    end
    if nargin < 4
        strike_duration = 0.1;  % 0.1 seconds
    end
    if nargin < 5
        sample_time = 0.001;  % 1ms sampling
    end
    
    % Time vector for strike action (down and up)
    % Total duration = 2 * strike_duration (down + up)
    total_duration = 2 * strike_duration;
    t_strike = (0:sample_time:total_duration)';
    N = length(t_strike);
    
    % Initialize joint trajectories
    q_strike = zeros(N, 6);
    qd_strike = zeros(N, 6);
    qdd_strike = zeros(N, 6);
    
    % Joints 1-4 and 6 remain constant
    for i = 1:N
        q_strike(i, :) = q_approach;
    end
    
    % Joint 5 performs the striking motion (simple sinusoidal motion)
    % Down stroke: 0 to strike_duration
    % Up stroke: strike_duration to total_duration
    half_N = round(N/2);
    
    for i = 1:N
        t_current = t_strike(i);
        
        if t_current <= strike_duration
            % Down stroke: rotate from 0 to strike_angle
            progress = t_current / strike_duration;
            % Use smooth acceleration/deceleration (cosine profile)
            q_strike(i, 5) = q_approach(5) + strike_angle * (1 - cos(pi * progress)) / 2;
            qd_strike(i, 5) = strike_angle * (pi / (2 * strike_duration)) * sin(pi * progress);
            qdd_strike(i, 5) = strike_angle * (pi^2 / (2 * strike_duration^2)) * cos(pi * progress);
        else
            % Up stroke: rotate from strike_angle back to 0
            t_up = t_current - strike_duration;
            progress = t_up / strike_duration;
            q_strike(i, 5) = q_approach(5) + strike_angle * (1 + cos(pi * progress)) / 2;
            qd_strike(i, 5) = -strike_angle * (pi / (2 * strike_duration)) * sin(pi * progress);
            qdd_strike(i, 5) = -strike_angle * (pi^2 / (2 * strike_duration^2)) * cos(pi * progress);
        end
    end
    
    % Ensure other joints have zero velocity and acceleration
    for j = [1, 2, 3, 4, 6]
        qd_strike(:, j) = 0;
        qdd_strike(:, j) = 0;
    end
end
