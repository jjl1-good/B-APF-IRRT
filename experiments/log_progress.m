function log_progress(fmt, varargin)
%LOG_PROGRESS 追加一行进度到 %TEMP%\bair_progress.log，并同时打印到控制台。
% 每次写入后关闭文件，确保重定向 stdout 时进度仍能立即落盘（MATLAB 的
% stdout 重定向是块缓冲，长实验期间无法实时查看）。
p = fullfile(tempdir, 'bair_progress.log');
fid = fopen(p, 'a');
if fid > 0
    fprintf(fid, '[%s] ', datestr(now, 'HH:MM:SS')); %#ok<DATST>
    fprintf(fid, fmt, varargin{:});
    fprintf(fid, '\n');
    fclose(fid);
end
fprintf(fmt, varargin{:});
fprintf('\n');
end
