clc 
clear all

% % 初始化数据结构
BAIRRT.l = [];
BAIRRT.t = [];
% 运行BAIRRT算法50次
for u=1:50
    fprintf('BAIRRT 第%d次运行...\n', u);
    [BAIRRT(u).l, BAIRRT(u).t] = BAIR3D();
    fprintf('BAIRRT 第%d次运行: 路径长度=%.2f, 时间=%.2f秒\n', u, BAIRRT(u).l, BAIRRT(u).t);
    
    % 清理内存
    clear -regexp ^[^BAIRRT] 
    close all hidden
end
IRT.l = [];
IRT.t = [];
for u=1:50
    fprintf('IRRT* 第%d次运行...\n', u);
    [IRT(u).l, IRT(u).t] = IRT3D();
    fprintf('IRRT* 第%d次运行: 路径长度=%.2f, 时间=%.2f秒\n', u, IRT(u).l, IRT(u).t);
    
    % 清理内存
    clear -regexp ^[^IRT3D] 
    close all hidden
end
RTS.l = [];
RTS.t = [];
% 运行BAIRRT算法50次
for u=1:50
    fprintf('RTS第%d次运行...\n', u);
    [RTS(u).l, RTS(u).t] = RTS3D();
    fprintf('RTS 第%d次运行: 路径长度=%.2f, 时间=%.2f秒\n', u, RTS(u).l, RTS(u).t);
    
    % 清理内存
    clear -regexp ^[^RTS3D] 
    close all hidden
end