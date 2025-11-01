%
% ABSTRACT:  梯形速度轨迹规划核心文件，计算轨迹的运动参数和完整轨迹点。
%            适用于关节空间或笛卡尔空间。
%
% INPUTS:    TotalLength             轨迹总长度，1xN数组，N为自由度数。单位：m 或 rad
%            Vmax                    最大匀速速度，1xN数组。单位：m/s 或 rad/s
%            Accel                   最大加速度，1xN数组。单位：m/s^2 或 rad/s^2
%            Decel                   最大减速度，1xN数组。单位：m/s^2 或 rad/s^2
%            Ts                      计算步长（采样周期），单位：s。默认 0.001s
%
% OUTPUTS:   MotionData              轨迹参数结构体
%            s                       轨迹长度数组，NxM数组。单位：m 或 rad
%            v                       速度数组，NxM数组。单位：m/s 或 rad/s
%            a                       加速度数组，NxM数组。单位：m/s^2 或 rad/s^2
%            t                       时间数组，Mx1数组。单位：s
%            maxtime                 总运动时间，单位：s

function [MotionData, s, v, a, t, maxtime] = MotionPlanning(TotalLength, Vmax, Accel, Decel, Ts)
    
    num_axes = length(TotalLength); % 获取自由度数量
    
    
    % 初始化存储每个轴参数的cell数组
    ta = zeros(1, num_axes); % 加速时间
    td = zeros(1, num_axes); % 减速时间
    tv = zeros(1, num_axes); % 匀速时间
    T = zeros(1, num_axes);  % 总时间
    v_actual = zeros(1, num_axes); % 实际达到的最大速度

    % --- 步骤 1: 为每个轴独立计算运动参数 ---
    for i = 1:num_axes
        % 检查是否能达到最大速度 Vmax
        if Vmax(i)^2 * (Accel(i) + Decel(i)) > 2 * Accel(i) * Decel(i) * TotalLength(i)
            % 三角形速度曲线 (无法达到Vmax)
            v_actual(i) = sqrt(2 * TotalLength(i) * Accel(i) * Decel(i) / (Accel(i) + Decel(i)));
            ta(i) = v_actual(i) / Accel(i);
            td(i) = v_actual(i) / Decel(i);
            tv(i) = 0;
        else
            % 梯形速度曲线 (可以达到Vmax)
            v_actual(i) = Vmax(i);
            ta(i) = Vmax(i) / Accel(i);
            td(i) = Vmax(i) / Decel(i);
            tv(i) = TotalLength(i) / Vmax(i) - (ta(i) + td(i)) / 2;
        end
        T(i) = ta(i) + tv(i) + td(i); % 计算当前轴的总时间
    end

    % --- 步骤 2: 时间同步 ---
    % 所有轴必须在同一时间完成运动，因此以最长的运动时间为准
    maxtime = max(T);
    
    % --- 步骤 3: 根据同步后的总时间，重新计算每个轴的运动参数 ---
    MotionData.s = zeros(1, num_axes);
    MotionData.v = zeros(1, num_axes);
    MotionData.a = zeros(1, num_axes);
    MotionData.ta = zeros(1, num_axes);
    MotionData.td = zeros(1, num_axes);
    MotionData.tv = zeros(1, num_axes);
    MotionData.T = maxtime;
    MotionData.dir = sign(TotalLength); % 运动方向
    
    L_abs = abs(TotalLength); % 取绝对值进行计算

    for i = 1:num_axes
        % 检查原规划时间是否小于同步时间，若是，则需要重新计算参数
        L_abs_i = L_abs(i);
    
        % --- 新增的健壮性判断 ---
        if L_abs_i < eps % 如果当前轴的总位移为零
            % 对于静止的轴，所有运动参数都为零
            MotionData.s(i) = 0;
            MotionData.v(i) = 0;
            MotionData.a(i) = 0;
            MotionData.ta(i) = 0;
            MotionData.td(i) = 0;
            MotionData.tv(i) = maxtime; % 整个过程都在"匀速"零
            continue; % 直接处理下一个轴
        end
        % --- 判断结束 ---
        if T(i) < maxtime
            % 尝试以原最大加速度Accel(i)进行规划
            delta = Accel(i)^2 * maxtime^2 - 4 * Accel(i) * L_abs(i);
            if delta >= 0
                % 可以通过降低最大速度和加速度来实现时间同步（梯形或三角形）
                v_new = (Accel(i) * maxtime - sqrt(delta)) / 2;
                a_new = Accel(i);
                d_new = (v_new^2) / (2*L_abs(i) - v_new*maxtime);
            else
                % 必须降低加速度才能在maxtime内完成
                a_new = maxtime^2 / (2 * L_abs(i)) * (maxtime - sqrt(maxtime^2 - 4 * L_abs(i) / Accel(i)));
                a_new = Accel(i)^2 * maxtime^2 / (2*L_abs(i) + Accel(i)*maxtime^2);
                v_new = a_new * maxtime / 2;
            end
            
            % 统一用梯形公式计算（当tv=0时即为三角形）
            v_actual(i) = (L_abs(i) + Accel(i)*Decel(i)*maxtime^2/(2*(Accel(i)+Decel(i)))) / maxtime;
            v_actual(i) = (Accel(i)*maxtime - sqrt(Accel(i)^2*maxtime^2 - 2*Accel(i)*(Accel(i)+Decel(i))*L_abs(i)/(Decel(i)))) * Decel(i) / (Accel(i)+Decel(i));
            
            % 重新计算速度和加速度以匹配maxtime
            v_recalc = (maxtime - sqrt(maxtime^2 - (2 * L_abs(i) / Accel(i)))) * Accel(i);
            if ~isreal(v_recalc) || v_recalc > Vmax(i) % 如果无法达到或超过Vmax
                a_recalc = Vmax(i) / (maxtime - L_abs(i)/Vmax(i));
                v_recalc = Vmax(i);
            else
                 a_recalc = Accel(i);
            end

            MotionData.a(i) = a_recalc;
            MotionData.v(i) = v_recalc;
            MotionData.ta(i) = v_recalc / a_recalc;
            MotionData.td(i) = v_recalc / a_recalc; % 假设加减速能力相同
            MotionData.tv(i) = maxtime - MotionData.ta(i) - MotionData.td(i);
            MotionData.s(i) = L_abs(i);
        else % 如果当前轴是时间最长的轴，直接使用原参数
            MotionData.s(i) = L_abs(i);
            MotionData.v(i) = v_actual(i);
            MotionData.a(i) = Accel(i);
            MotionData.ta(i) = ta(i);
            MotionData.td(i) = td(i);
            MotionData.tv(i) = tv(i);
        end
    end

    % --- 步骤 4: 生成完整的轨迹数据点 ---
    t = (0:Ts:maxtime)'; % 时间向量
    num_steps = length(t);
    s = zeros(num_axes, num_steps);
    v = zeros(num_axes, num_steps);
    a = zeros(num_axes, num_steps);

    for i = 1:num_axes
        % 提取同步后的参数
        ta_sync = MotionData.ta(i);
        td_sync = MotionData.td(i);
        tv_sync = MotionData.tv(i);
        v_sync = MotionData.v(i);
        a_sync = MotionData.a(i);
        dir = MotionData.dir(i);

        for j = 1:num_steps
            time_current = t(j);
            % 加速段
            if time_current <= ta_sync
                a(i, j) = dir * a_sync;
                v(i, j) = dir * a_sync * time_current;
                s(i, j) = dir * 0.5 * a_sync * time_current^2;
            % 匀速段
            elseif time_current <= ta_sync + tv_sync
                a(i, j) = 0;
                v(i, j) = dir * v_sync;
                s(i, j) = dir * (0.5 * a_sync * ta_sync^2 + v_sync * (time_current - ta_sync));
            % 减速段
            else
                time_in_decel = time_current - (ta_sync + tv_sync);
                a(i, j) = -dir * a_sync; % 假设减速度大小等于加速度
                v(i, j) = dir * (v_sync - a_sync * time_in_decel);
                s_accel_const = dir * (0.5 * a_sync * ta_sync^2 + v_sync * tv_sync);
                s(i, j) = s_accel_const + dir * (v_sync * time_in_decel - 0.5 * a_sync * time_in_decel^2);
            end
        end
    end
end