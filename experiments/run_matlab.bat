@echo off
rem 用法: run_matlab.bat <要执行的表达式> <日志文件名>
rem 说明: 以无界面方式运行 MATLAB 并把 stdout/stderr 写入 %TEMP%\<日志文件名>。
setlocal EnableDelayedExpansion
set "EXPR=%~1"
set "LOG=%TEMP%\%~2"
matlab -batch "addpath('%~dp0'); %EXPR%" > "%LOG%" 2>&1
echo EXIT=%ERRORLEVEL% >> "%LOG%"
endlocal
