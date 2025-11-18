% % ---------------------- 定义多段轨迹参数 ----------------------
% % 每段轨迹参数：[axis, start_pos, end_pos, t_d, dd_acc, fixed_x, fixed_y, fixed_z]
% segments = {
%     % 第一段：x轴从0→50mm，时长1s
%     {'z', 0, -50, 1, 300, 0, 0, 0};
%     % 第二段：y轴从0→30mm，时长0.8s（x保持50mm，z保持0）
%     {'z', -50, 0, 1, 300, 0, 0, -50};
%     % 第三段：z轴从0→20mm，时长1.2s（x保持50mm，y保持30mm）
%     {'x', 0, -150, 3, 100, 0, 0, 0};
%     % 
%     {'z', 0, -50, 1, 300, -150, 0, 0};
%     % 第二段：y轴从0→30mm，时长0.8s（x保持50mm，z保持0）
%     {'z', -50, 0, 1, 300, -150, 0, -50};
% };
% 
% % ---------------------- 初始化存储变量（统一为行向量） ----------------------
% all_t = [];   % 总时间数组（行向量）
% all_x = [];   % 总x坐标数组（行向量）
% all_y = [];   % 总y坐标数组（行向量）
% all_z = [];   % 总z坐标数组（行向量）
% time_offset = 0;  % 时间偏移量
% 
% % ---------------------- 循环调用并拼接轨迹 ----------------------
% for i = 1:size(segments, 1)
%     % 提取当前段参数
%     axis = segments{i}{1};
%     start_pos = segments{i}{2};
%     end_pos = segments{i}{3};
%     t_d = segments{i}{4};
%     dd_acc = segments{i}{5};
%     fixed_x = segments{i}{6};
%     fixed_y = segments{i}{7};
%     fixed_z = segments{i}{8};
% 
%     % 调用函数生成当前段轨迹
%     [t_seg, x_seg, y_seg, z_seg] = LFPB_3D_Trajectory(...
%         axis, start_pos, end_pos, t_d, dd_acc, fixed_x, fixed_y, fixed_z);
% 
%     % 时间偏移（确保连续）
%     t_seg = t_seg + time_offset;
% 
%     % 拼接：用逗号（横向拼接），跳过重复点
%     if i == 1
%         all_t = t_seg;
%         all_x = x_seg;
%         all_y = y_seg;
%         all_z = z_seg;
%     else
%         % 跳过当前段第一个点（与上一段终点重叠）
%         all_t = [all_t, t_seg(2:end)];  % 横向拼接行向量
%         all_x = [all_x, x_seg(2:end)];
%         all_y = [all_y, y_seg(2:end)];
%         all_z = [all_z, z_seg(2:end)];
%     end
% 
%     % 更新时间偏移量
%     time_offset = time_offset + t_d;
% end
% 
% % ---------------------- 可视化完整轨迹 ----------------------
% figure('Name', '多段LFPB轨迹拼接', 'Position', [100, 100, 1000, 800]);
% 
% % 子图1：x-t曲线
% subplot(3, 1, 1);
% plot(all_t, all_x, 'LineWidth', 2, 'Color', [0.2, 0.5, 0.8]);
% grid on;
% xlabel('时间 t (s)', 'FontSize', 11);
% ylabel('x轴坐标 (mm)', 'FontSize', 11);
% title('x-t 完整轨迹', 'FontSize', 13, 'FontWeight', 'bold');
% % axis tight;
% 
% % 子图2：y-t曲线
% subplot(3, 1, 2);
% plot(all_t, all_y, 'LineWidth', 2, 'Color', [0.8, 0.3, 0.3]);
% grid on;
% xlabel('时间 t (s)', 'FontSize', 11);
% ylabel('y轴坐标 (mm)', 'FontSize', 11);
% title('y-t 完整轨迹', 'FontSize', 13, 'FontWeight', 'bold');
% % axis tight;
% 
% % 子图3：z-t曲线
% subplot(3, 1, 3);
% plot(all_t, all_z, 'LineWidth', 2, 'Color', [0.3, 0.8, 0.3]);
% grid on;
% xlabel('时间 t (s)', 'FontSize', 11);
% ylabel('z轴坐标 (mm)', 'FontSize', 11);
% title('z-t 完整轨迹', 'FontSize', 13, 'FontWeight', 'bold');
% % axis tight;
% 
% % 总标题和布局调整
% sgtitle('多段LFPB拼接轨迹时间序列', 'FontSize', 15, 'FontWeight', 'bold');
% set(gcf, 'Position', [100, 100, 1000, 800]);
% whitebg(gcf, [1, 1, 1]);
% 
% % ---------------------- 原有函数（无需修改） ----------------------
% function [t, x, y, z] = LFPB_3D_Trajectory(axis, start_pos, end_pos, t_d, dd_acc, fixed_x, fixed_y, fixed_z)
%     t = 0:1e-3:t_d;  % 行向量时间数组
%     switch lower(axis)
%         case 'x'
%             x = arrayfun(@(t_val) LFPB_Trajectory(t_val, start_pos, end_pos, t_d, dd_acc), t);
%             y = ones(size(t)) * fixed_y;
%             z = ones(size(t)) * fixed_z;
%         case 'y'
%             y = arrayfun(@(t_val) LFPB_Trajectory(t_val, start_pos, end_pos, t_d, dd_acc), t);
%             x = ones(size(t)) * fixed_x;
%             z = ones(size(t)) * fixed_z;
%         case 'z'
%             z = arrayfun(@(t_val) LFPB_Trajectory(t_val, start_pos, end_pos, t_d, dd_acc), t);
%             x = ones(size(t)) * fixed_x;
%             y = ones(size(t)) * fixed_y;
%         otherwise
%             error('无效的轴选择，请输入''x''、''y''或''z''');
%     end
% end
% 
% function theta = LFPB_Trajectory(t, theta_o, theta_f, t_d, dd_theta_b)
%     % LFPB_Trajectory 计算LFPB轨迹在时间t处的位置（支持正向/反向运动）
%     % 输入参数：
%     %   t           - 当前时间
%     %   theta_o     - 初始位置
%     %   theta_f     - 最终位置
%     %   t_d         - 总时长
%     %   dd_theta_b  - 加速度大小（绝对值，正数）
%     % 输出参数：
%     %   theta       - 时间t对应的位置
% 
%     t_o = 0; 
%     t_f_total = t_d;  % 总时长终点
% 
%     % 1. 计算运动方向（1：正向；-1：反向）
%     direction = sign(theta_f - theta_o);
%     if direction == 0
%         % 起始位置 = 终止位置（静止）
%         theta = theta_o;
%         return;
%     end
% 
%     % 2. 带符号的加速度（方向决定符号）
%     dd_theta = direction * dd_theta_b;
% 
%     % 3. 修正加速度约束（用绝对值判断，避免方向影响）
%     delta = abs(theta_f - theta_o);  % 位置差的绝对值
%     acc_constraint = 4 * delta / t_d^2;  % 约束为正数
%     if dd_theta_b < acc_constraint  % 用加速度大小判断
%         error('加速度不满足约束条件：dd_theta_b >= 4*|theta_f - theta_o|/t_d^2');
%     end
% 
%     % 4. 计算Blend时长t_b（用绝对值计算，确保为正数）
%     numerator = sqrt( (dd_theta_b * t_d)^2 - 4 * dd_theta_b * delta );
%     t_b = t_d/2 - numerator/(2 * dd_theta_b);  % 此时t_b一定为正数
% 
%     % 5. 分段计算位置（根据方向调整公式）
%     if t >= t_o && t < t_o + t_b
%         % 加速阶段（正向：加速增加；反向：加速减少）
%         theta = theta_o + 0.5 * dd_theta * (t - t_o).^2;
%     elseif t >= t_o + t_b && t < t_f_total - t_b
%         % 线性阶段（正向：匀速增加；反向：匀速减少）
%         theta = theta_o + dd_theta * t_b * (t - t_o - t_b/2);
%     elseif t >= t_f_total - t_b && t <= t_f_total
%         % 减速阶段（正向：减速增加；反向：减速减少）
%         theta = theta_f - 0.5 * dd_theta * (t_f_total - t).^2;
%     else
%         error('时间t超出轨迹定义的时间范围（0到%.2f）', t_d);
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [all_t, all_x, all_y, all_z] = LFPB()
% LFPB - 生成多段 3D LFPB 轨迹，并返回 all_t / all_x / all_y / all_z
% 现在可作为函数调用：[t, x, y, z] = LFPB();

% ---------------------- 定义多段轨迹参数 ----------------------
% 每段轨迹参数：[axis, start_pos, end_pos, t_d, dd_acc, fixed_x, fixed_y, fixed_z]
segments = {
    {'z',  800,    319.98, 10, 20, 499.99, -155.0,  0};
    {'z',  319.98,  800,   10, 20, 499.99, -155.0,  0};
    {'y', -155.0,   154.99, 10, 20, 499.99,  0,    800};
    {'z',  800,    319.98, 10, 20, 499.99,  154.99, 0};
    {'z',  319.98,  800,   10, 20, 499.99,  154.99, 0};
};

% segments = {
%     {'z',  800,    319.98, 10, 20, 499.99, -100.0,  0};
%     {'z',  319.98,  800,   10, 20, 499.99, -100.0,  0};
%     {'y', -100.0,   100.0, 10, 10, 499.99,  0,    800};
%     {'z',  800,    319.98, 10, 20, 499.99,  100, 0};
%     {'z',  319.98,  800,   10, 20, 499.99,  100, 0};
% };

% ---------------------- 初始化存储变量 ----------------------
all_t = [];
all_x = [];
all_y = [];
all_z = [];
time_offset = 0;

% ---------------------- 循环拼接多段轨迹 ----------------------
for i = 1:size(segments, 1)
    axis = segments{i}{1};
    start_pos = segments{i}{2};
    end_pos = segments{i}{3};
    t_d = segments{i}{4};
    dd_acc = segments{i}{5};
    fixed_x = segments{i}{6};
    fixed_y = segments{i}{7};
    fixed_z = segments{i}{8};

    % 当前段轨迹
    [t_seg, x_seg, y_seg, z_seg] = LFPB_3D_Trajectory( ...
        axis, start_pos, end_pos, t_d, dd_acc, fixed_x, fixed_y, fixed_z);

    % 时间偏移
    t_seg = t_seg + time_offset;

    % 拼接（跳过重复点）
    if i == 1
        all_t = t_seg;
        all_x = x_seg;
        all_y = y_seg;
        all_z = z_seg;
    else
        all_t = [all_t, t_seg(2:end)];
        all_x = [all_x, x_seg(2:end)];
        all_y = [all_y, y_seg(2:end)];
        all_z = [all_z, z_seg(2:end)];
    end

    time_offset = time_offset + t_d;
end

% ---------------------- 可视化轨迹 ----------------------
figure('Name','LFPB 轨迹调试','Position',[100,100,1200,600]);

subplot(3,1,1);
plot(all_t, all_x,'b','LineWidth',1.5); grid on;
xlabel('t (s)'); ylabel('x (mm)'); title('x-t轨迹');

subplot(3,1,2);
plot(all_t, all_y,'r','LineWidth',1.5); grid on;
xlabel('t (s)'); ylabel('y (mm)'); title('y-t轨迹');

subplot(3,1,3);
plot(all_t, all_z,'g','LineWidth',1.5); grid on;
xlabel('t (s)'); ylabel('z (mm)'); title('z-t轨迹');

sgtitle('LFPB 多段轨迹调试','FontSize',14,'FontWeight','bold');


end  % ← 注意：这行结束函数 LFPB()


% ---------------------- 以下保持为子函数，不会污染 workspace ----------------------
function [t, x, y, z] = LFPB_3D_Trajectory(axis, start_pos, end_pos, t_d, dd_acc, fixed_x, fixed_y, fixed_z)
    t = 0:1e-3:t_d;

    switch lower(axis)
        case 'x'
            x = arrayfun(@(t_val) LFPB_Trajectory(t_val, start_pos, end_pos, t_d, dd_acc), t);
            y = ones(size(t)) * fixed_y;
            z = ones(size(t)) * fixed_z;

        case 'y'
            y = arrayfun(@(t_val) LFPB_Trajectory(t_val, start_pos, end_pos, t_d, dd_acc), t);
            x = ones(size(t)) * fixed_x;
            z = ones(size(t)) * fixed_z;

        case 'z'
            z = arrayfun(@(t_val) LFPB_Trajectory(t_val, start_pos, end_pos, t_d, dd_acc), t);
            x = ones(size(t)) * fixed_x;
            y = ones(size(t)) * fixed_y;

        otherwise
            error('无效的轴选择，应为 x / y / z');
    end
end


function theta = LFPB_Trajectory(t, theta_o, theta_f, t_d, dd_theta_b)

    t_o = 0;
    t_f_total = t_d;

    direction = sign(theta_f - theta_o);
    if direction == 0
        theta = theta_o;
        return;
    end

    dd_theta = direction * dd_theta_b;

    delta = abs(theta_f - theta_o);
    acc_constraint = 4 * delta / t_d^2;

    if dd_theta_b < acc_constraint
        error('加速度不满足约束：dd_theta_b >= 4*|Δθ|/t_d^2');
    end

    numerator = sqrt((dd_theta_b * t_d)^2 - 4 * dd_theta_b * delta);
    t_b = t_d/2 - numerator / (2 * dd_theta_b);

    if t >= t_o && t < t_o + t_b
        theta = theta_o + 0.5 * dd_theta * (t - t_o)^2;

    elseif t >= t_o + t_b && t < t_f_total - t_b
        theta = theta_o + dd_theta * t_b * (t - t_o - t_b/2);

    elseif t >= t_f_total - t_b && t <= t_f_total
        theta = theta_f - 0.5 * dd_theta * (t_f_total - t)^2;

    else
        error('时间 t 超出轨迹范围 (0 ~ %.2f)', t_d);
    end
end


