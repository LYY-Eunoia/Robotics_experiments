% Author: 
%
% ABSTRACT:  关节空间的梯形速度规划，根据时间和路径公式参数结构体计算当前时间点，关节位置，速度和加速度
%
% INPUTS: time            当前时刻
%         MotionData      路径公式参数结构体，包括s,v,a,j,t信息
%         loc1            路径起点点位
%         loc2            路径终点点位
%             
% OUTPUTS: q              当前时间点，关节位置向量1xN，单位m或rad
%          qd             当前时间点，关节速度向量1xN,单位m/s或rad/s
%          qdd            当前时间点，关节加速度向量1xN,单位m/s^2或rad/s^2       
%
function [q,qd,qdd] = TrajGena(time, MotionData, loc1,loc2)


end

