% Author: GitHub Copilot
%
% ABSTRACT:  根据时间和路径参数，计算单个时间点的关节/笛卡尔空间状态。
%
% INPUTS:    time            当前时刻, double
%            MotionData      由MotionPlanning生成的路径参数结构体
%            loc1            路径起点位姿，1xN向量
%            loc2            路径终点位姿，1xN向量
%             
% OUTPUTS:   q               当前时间点的位姿向量，1xN。单位：m或rad
%            qd              当前时间点的速度向量，1xN。单位：m/s或rad/s
%            qdd             当前时间点的加速度向量，1xN。单位：m/s^2或rad/s^2       
%
function [q, qd, qdd] = TrajGena(time, MotionData, loc1, loc2)

    num_axes = length(loc1); % 获取自由度数量
    q = zeros(1, num_axes);
    qd = zeros(1, num_axes);
    qdd = zeros(1, num_axes);
    
    % 确保时间在有效范围内
    if time > MotionData.T
        time = MotionData.T;
    end
    if time < 0
        time = 0;
    end

    for i = 1:num_axes
        % 从MotionData结构体中提取该轴的参数
        ta = MotionData.ta(i);
        tv = MotionData.tv(i);
        v_max = MotionData.v(i);
        a_max = MotionData.a(i);
        dir = MotionData.dir(i); % 运动方向 (+1 或 -1)

        % 初始化当前轴的s, v, a
        s_i = 0; % 位移
        v_i = 0; % 速度
        a_i = 0; % 加速度

        % --- 根据当前时间所在的区间，使用运动学公式计算s, v, a ---
        
        % 加速段 (0 <= time <= ta)
        if time <= ta
            a_i = a_max;
            v_i = a_max * time;
            s_i = 0.5 * a_max * time^2;
        
        % 匀速段 (ta < time <= ta + tv)
        elseif time <= ta + tv
            a_i = 0;
            v_i = v_max;
            % 位移 = 加速段位移 + 匀速段位移
            s_i = 0.5 * a_max * ta^2 + v_max * (time - ta);
        
        % 减速段 (ta + tv < time <= T)
        else
            % 注意：此处减速度大小假设等于加速度大小 a_max
            time_decel = time - (ta + tv); % 在减速段经过的时间
            a_i = -a_max;
            v_i = v_max - a_max * time_decel;
            % 位移 = 加速段位移 + 匀速段位移 + 减速段位移
            s_accel = 0.5 * a_max * ta^2; % 加速段总位移
            s_const = v_max * tv;       % 匀速段总位移
            s_decel = v_max * time_decel - 0.5 * a_max * time_decel^2; % 减速段位移
            s_i = s_accel + s_const + s_decel;
        end
        
        % --- 计算最终的 q, qd, qdd ---
        % q   = 起点 + 归一化位移 * 总距离
        % qd  = 归一化速度 * 总距离
        % qdd = 归一化加速度 * 总距离
        % 注意: MotionData.s(i) 是总位移的绝对值
        
        total_dist = loc2(i) - loc1(i);
        
        % 如果总距离为0，则始终保持在起点
        if total_dist == 0
            q(i) = loc1(i);
            qd(i) = 0;
            qdd(i) = 0;
        else
            q(i) = loc1(i) + (s_i / MotionData.s(i)) * total_dist;
            qd(i) = (v_i / MotionData.s(i)) * total_dist;
            qdd(i) = (a_i / MotionData.s(i)) * total_dist;
        end
    end
end