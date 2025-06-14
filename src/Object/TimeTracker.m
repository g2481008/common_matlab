classdef TimeTracker < handle
    properties (Access = private)
        StartTime
        LastCheckTime
        LapTime
    end
    
    methods
        % タイマー開始
        function start(obj)
            obj.StartTime = tic;
            obj.LapTime = .0;
            obj.LastCheckTime = obj.StartTime;
        end

        % 開始からの経過時間
        function elapsed = elapsed(obj)
            elapsed = toc(obj.StartTime);
        end

        % 前回チェックからの経過時間（リセットしない）
        function interval = interval(obj)
            now_time = toc(obj.LastCheckTime);
            interval = now_time - obj.LapTime;
            obj.LapTime = now_time;
        end

        % 指定秒数xが経過したかどうかを判定（LastCheckTime は更新しない）
        function flag = hasElapsed(obj, x)
            elapsed = toc(obj.LastCheckTime)-obj.LapTime;
            flag = elapsed >= x;
        end

        % 明示的に LastCheckTime を更新
        function resetInterval(obj)
            obj.LastCheckTime = tic;
        end
    end
end
