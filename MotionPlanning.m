% Author: 
%
% ABSTRACT:  ���׹켣�滮������ļ����ѿ����ռ�ֱ�߹켣���ٶȹ滮
%
% INPUTS:InterpolationCycle      ���㲽������λs ����Ϊ0.001s
%        TotalLength             �켣���ȣ�1xN���飬NΪ�켣���ɶ�������λm����rad
%        Vmax                    �켣���ٶ�����ٶȣ�1xN���飬��λm/s ��rad/s
%        Acc                     �����ٶȣ�1xN���飬��λm/s^2 �� rad/s^2
%        Dec                     �����ٶȣ�1xN���飬��λm/s^2 �� rad/s^2
%
% OUTPUTS:MotionData             �켣��ʽ������
%         s 	                 �켣�������飬NxM���飬��λm �� rad
%         v 	                 �ٶ����飬NxM���飬��λm/s ��rad/s
%         a 	                 ���ٶ����飬NxM���飬��λm/s^2 �� rad/s^2
%         t 	                 ʱ�����飬Mx1���飬��λs
%         maxtime 	             ����ʱ�䣬��λs

function MotionData = MotionPlanning(TotalLength, Vmax, Accel, Decel, Ts)
    % Number of axes
    N = length(TotalLength);
    
    % Initialize arrays to store timing parameters for each axis
    t_accel = zeros(1, N);  % Acceleration time
    t_const = zeros(1, N);  % Constant velocity time
    t_decel = zeros(1, N);  % Deceleration time
    t_total = zeros(1, N);  % Total time for each axis
    v_actual = zeros(1, N); % Actual max velocity (may be reduced for triangular profile)
    
    % Calculate motion parameters for each axis independently
    for i = 1:N
        S = abs(TotalLength(i));  % Total displacement (absolute value)
        V = Vmax(i);              % Maximum velocity
        A = Accel(i);             % Acceleration
        D = Decel(i);             % Deceleration
        
        % Time to accelerate to max velocity
        t_acc = V / A;
        % Time to decelerate from max velocity
        t_dec = V / D;
        % Distance covered during acceleration
        s_acc = 0.5 * A * t_acc^2;
        % Distance covered during deceleration
        s_dec = 0.5 * D * t_dec^2;
        
        % Check if trapezoidal or triangular profile
        if (s_acc + s_dec) <= S
            % Trapezoidal profile: reaches max velocity
            t_accel(i) = t_acc;
            t_decel(i) = t_dec;
            % Distance during constant velocity phase
            s_const = S - s_acc - s_dec;
            t_const(i) = s_const / V;
            v_actual(i) = V;
        else
            % Triangular profile: doesn't reach max velocity
            % Calculate actual peak velocity
            v_peak = sqrt(2 * S * A * D / (A + D));
            v_actual(i) = v_peak;
            t_accel(i) = v_peak / A;
            t_decel(i) = v_peak / D;
            t_const(i) = 0;
        end
        
        % Total time for this axis
        t_total(i) = t_accel(i) + t_const(i) + t_decel(i);
    end
    
    % Synchronize: use maximum time across all axes
    maxtime = max(t_total);
    
    % Generate time vector
    M = ceil(maxtime / Ts) + 1;  % Number of time samples
    t = (0:M-1)' * Ts;  % Time vector (Mx1)
    
    % Initialize output matrices
    s = zeros(N, M);  % Position (NxM)
    v = zeros(N, M);  % Velocity (NxM)
    a = zeros(N, M);  % Acceleration (NxM)
    
    % Generate trajectory for each axis
    for i = 1:N
        S = TotalLength(i);  % Total displacement (with sign)
        sign_S = sign(S);
        if sign_S == 0
            sign_S = 1;  % Handle zero displacement case
        end
        S_abs = abs(S);
        
        A = Accel(i);
        D = Decel(i);
        V = v_actual(i);
        
        t1 = t_accel(i);
        t2 = t_accel(i) + t_const(i);
        t3 = t_total(i);
        
        for j = 1:M
            t_curr = t(j);
            
            if t_curr <= t1
                % Acceleration phase
                a(i, j) = sign_S * A;
                v(i, j) = sign_S * A * t_curr;
                s(i, j) = sign_S * 0.5 * A * t_curr^2;
            elseif t_curr <= t2
                % Constant velocity phase
                a(i, j) = 0;
                v(i, j) = sign_S * V;
                s(i, j) = sign_S * (0.5 * A * t1^2 + V * (t_curr - t1));
            elseif t_curr <= t3
                % Deceleration phase
                t_dec_elapsed = t_curr - t2;
                a(i, j) = -sign_S * D;
                v(i, j) = sign_S * (V - D * t_dec_elapsed);
                s_before = 0.5 * A * t1^2 + V * (t2 - t1);
                s(i, j) = sign_S * (s_before + V * t_dec_elapsed - 0.5 * D * t_dec_elapsed^2);
            else
                % Motion complete, hold final position
                a(i, j) = 0;
                v(i, j) = 0;
                s(i, j) = S;
            end
        end
    end
    
    % Package results into output structure
    MotionData.s = s;
    MotionData.v = v;
    MotionData.a = a;
    MotionData.t = t;
    MotionData.maxtime = maxtime;
end
