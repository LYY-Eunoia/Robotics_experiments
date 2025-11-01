% Author: 
%
% ABSTRACT:  三阶轨迹规划最基本文件，笛卡尔空间直线轨迹的速度规划
%
% INPUTS:InterpolationCycle      计算步长，单位s 设置为0.001s
%        TotalLength             轨迹长度，1xN数组，N为轨迹自由度数，单位m或者rad
%        Vmax                    轨迹匀速段最大速度，1xN数组，单位m/s 或rad/s
%        Acc                     最大加速度，1xN数组，单位m/s^2 或 rad/s^2
%        Dec                     最大减速度，1xN数组，单位m/s^2 或 rad/s^2
%
% OUTPUTS:MotionData             轨迹公式参数，
%         s 	                 轨迹长度数组，NxM数组，单位m 或 rad
%         v 	                 速度数组，NxM数组，单位m/s 或rad/s
%         a 	                 加速度数组，NxM数组，单位m/s^2 或 rad/s^2
%         t 	                 时间数组，Mx1数组，单位s
%         maxtime 	             所需时间，单位s

function MotionData = MotionPlanning(TotalLength, Vmax, Accel, Decel,Ts)


  
end
