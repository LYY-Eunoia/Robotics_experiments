function theta_solutions = Ikine6s(T_target, config)
    % Ikine6s 计算6自由度机器人的逆运动学解
    % 输入:
    %   T_target - 末端执行器目标变换矩阵(4x4)
    %   config   - 机器人结构参数(包含L1, L2, L3, L4)
    % 输出:
    %   theta_solutions - 8组关节角解(8x6矩阵，弧度)
    
    % 提取机器人结构参数
    L1 = config.L1;
    L2 = config.L2;
    L3 = config.L3;
    % L4 = config.L4;  % 预留参数
    
    % 定义机器人螺旋轴与基座矩阵(结构固定参数)
    w0 = [0,0,0,0,0,0; 0,1,1,0,1,0; 1,0,0,1,0,1];  % 角速度分量(3x6)
    q0 = [0,0,0,0,0,0; 0,0,0,0,0,0; 0,491,941,941,1391,1475];  % 位置分量(3x6)
    T0 = [-1,0,0,0; 0,-1,0,0; 0,0,1,1475; 0,0,0,1];  % 基座变换矩阵
    
    % 计算螺旋轴(6x6，每列对应一个关节的螺旋)
    x0 = zeros(6,6);
    for i = 1:6
        w_i = w0(1:3,i);
        q_i = q0(1:3,i);
        x0(1:3,i) = -cross(w_i, q_i);  % 线速度分量
        x0(4:6,i) = w_i;               % 角速度分量
    end
    
    % 计算相对变换矩阵 G = T_target / T0 = T_target * inv(T0)
    G = T_target / T0;
    
    % 计算参考点在G坐标系下的位置
    pw = [0; 0; L1 + L2 + L3; 1];  % 齐次参考点
    q = G * pw;                    % 变换后的参考点(q(1:3)为x,y,z)
    
    % 求解theta1(肩关节旋转角)
    t11 = atan2(-q(2),-q(1));  % 用atan2确保象限正确
    t12 = atan2(q(2),q(1)); 
    % 求解theta2和theta3(大臂和小臂关节角)
    [t21, t22, t31, t32] = cal_theta23(q, L1, L2, L3);
    if all(isnan([t21, t22, t31, t32]))
        theta_solutions = [];  % 无有效解
        return;
    end
    
    % 初始化8组解的矩阵
    theta_solutions = zeros(8,6);
    % 填充theta1, theta2, theta3的组合解
    theta_solutions(1:8,1) = [t11; t11; t11; t11; t12; t12; t12; t12];  % theta1的2种可能
    theta_solutions(1:8,2) = [-t21; -t21; -t22; -t22; t21; t21; t22; t22];  % theta2的2种可能
    theta_solutions(1:8,3) = [t31; t31; t32; t32; t32; t32; t31; t31];  % theta3的2种可能
    
    % 求解theta4, theta5, theta6(腕部关节角)
    for i = 1:4
        theta1 = theta_solutions(2*i,1);
        theta2 = theta_solutions(2*i,2);
        theta3 = theta_solutions(2*i,3);
        
        % 计算前3关节的旋转矩阵累积
        T1 = exp_rot(x0(:,1), -theta1);  % 关节1的旋转
        T2 = exp_rot(x0(:,2), -theta2);  % 关节2的旋转
        T3 = exp_rot(x0(:,3), -theta3);  % 关节3的旋转
        R1 = T3*T2*T1;                % 前3关节总旋转
        
        % 提取G的旋转部分，计算腕部旋转矩阵
        G_rot = G(1:3,1:3);
        M = R1(1:3,1:3) * G_rot;
        
        theta6 = atan2(-M(3,2), M(3,1));  % theta6
        theta5 = acos(M(3,3));
        theta4 = atan2(M(2,3),M(1,3));
        theta_solutions(2*i,5)=theta5;
        theta_solutions(2*i,4)=theta4;
        if theta6 > 0
            theta_solutions(2*i,6) = theta6-pi;
            theta_solutions(2*i-1,6) = theta6;
            theta_solutions(2*i-1,5) = -theta5;
            if theta4>0
                theta_solutions(2*i-1,4) = theta4-pi;
            else
                theta_solutions(2*i-1,4) = theta4+pi;
            end
        else
            theta_solutions(2*i,6) = theta6+pi;
            theta_solutions(2*i-1,6) = theta6;
            theta_solutions(2*i-1,5) = -theta5;
            if theta4>0
                theta_solutions(2*i-1,4) = theta4-pi;
            else
                theta_solutions(2*i-1,4) = theta4+pi;
            end
        end
        
    end

    % 内部函数：计算theta2和theta3的解
    function [t21, t22, t31, t32] = cal_theta23(q, L1, L2, L3)
        v = q(1)^2 + q(2)^2 + (q(3) - L1)^2;  % 距离平方
        sqrt_v = sqrt(v);
        
        % 检查工作空间可行性
        if (sqrt_v + L3 > L2) && (abs(sqrt_v - L3) < L2)
            n = v + L2^2 - L3^2;
            n1 = L2^2 + L3^2 - v;
            t2 = acos((q(3) - L1) / sqrt_v);
            o2 = acos(n / (2 * L2 * sqrt_v));
            o3 = acos(n1 / (2 * L2 * L3));
            t21 = t2 + o2;
            t22 = t2 - o2;
            t31 = pi - o3;
            t32 = o3 - pi;
        elseif abs(sqrt_v + L3 - L2) < 1e-6  % 共线特殊情况
            t31 = pi;
            t32 = -pi;
            t2 = acos((q(3) - L1) / sqrt_v);
            t21 = t2;
            t22 = -t2;
        else
            disp("目标位置超出工作空间，无逆解");
            t21 = NaN;
            t22 = NaN;
            t31 = NaN;
            t32 = NaN;
        end
    end

    % 内部函数：计算旋转关节的旋转矩阵(指数映射)
    function T = exp_rot(x, t)
        w = x(4:6);  % 旋转轴
        v = x(1:3);
        if norm(w) < eps
            R = eye(3);  % 移动关节(此处为旋转关节，理论上不触发)
        else
            w_norm = norm(w);
            w_unit = w / w_norm;
            w_hat = skew(w_unit);  % 斜对称矩阵
            % 罗德里格斯公式
            R = eye(3) + sin(w_norm * t) * w_hat + (1 - cos(w_norm * t)) * (w_hat * w_hat);
            p = (eye(3) - R) * (cross(w_unit, v / w_norm)) + ...
            (w_unit * w_unit' * v * t);
        end
        T =[R,p;0,0,0,1];
    end

    % 内部函数：斜对称矩阵转换
    function S = skew(w)
        S = [0, -w(3), w(2);
             w(3), 0, -w(1);
             -w(2), w(1), 0];
    end

end