function [t_seg, q_seg, qd_seg, qdd_seg] = PlanJointSpaceMotion(q_start, q_end, duration, sample_time)
    % PlanJointSpaceMotion - Plan smooth joint-space motion between two configurations
    %
    % Input:
    %   q_start - Starting joint configuration (1x6 vector)
    %   q_end - Ending joint configuration (1x6 vector)
    %   duration - Motion duration in seconds
    %   sample_time - Sampling time in seconds (default: 0.001s)
    %
    % Output:
    %   t_seg - Time vector
    %   q_seg - Joint positions (Nx6 matrix)
    %   qd_seg - Joint velocities (Nx6 matrix)
    %   qdd_seg - Joint accelerations (Nx6 matrix)
    %
    % Uses a 5th-order polynomial trajectory for smooth motion
    
    if nargin < 4
        sample_time = 0.001;  % 1ms sampling
    end
    
    % Time vector
    t_seg = (0:sample_time:duration)';
    N = length(t_seg);
    
    % Initialize output arrays
    q_seg = zeros(N, 6);
    qd_seg = zeros(N, 6);
    qdd_seg = zeros(N, 6);
    
    % For each joint, use quintic (5th order) polynomial trajectory
    % Boundary conditions: zero velocity and acceleration at start and end
    for joint = 1:6
        theta_0 = q_start(joint);
        theta_f = q_end(joint);
        
        % 5th order polynomial coefficients
        % theta(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        % With boundary conditions:
        %   theta(0) = theta_0, theta'(0) = 0, theta''(0) = 0
        %   theta(T) = theta_f, theta'(T) = 0, theta''(T) = 0
        
        T = duration;
        a0 = theta_0;
        a1 = 0;
        a2 = 0;
        a3 = 10 * (theta_f - theta_0) / T^3;
        a4 = -15 * (theta_f - theta_0) / T^4;
        a5 = 6 * (theta_f - theta_0) / T^5;
        
        % Compute position, velocity, acceleration for all time steps
        for i = 1:N
            t = t_seg(i);
            t2 = t * t;
            t3 = t2 * t;
            t4 = t3 * t;
            t5 = t4 * t;
            
            % Position
            q_seg(i, joint) = a0 + a1*t + a2*t2 + a3*t3 + a4*t4 + a5*t5;
            
            % Velocity (first derivative)
            qd_seg(i, joint) = a1 + 2*a2*t + 3*a3*t2 + 4*a4*t3 + 5*a5*t4;
            
            % Acceleration (second derivative)
            qdd_seg(i, joint) = 2*a2 + 6*a3*t + 12*a4*t2 + 20*a5*t3;
        end
    end
end
