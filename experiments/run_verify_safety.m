function run_verify_safety()
%RUN_VERIFY_SAFETY 一键安全性复检入口（供 finalize.ps1 调用）。
% 重跑各算法并逐段独立复检返回路径是否无碰撞，输出
% verify_safety_2d.csv / verify_safety_3d.csv。
verify_safety(20);
end
