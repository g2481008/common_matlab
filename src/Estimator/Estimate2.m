classdef Estimate2 < handle
    %% Estimator
    %% Method
    % Estimate: 初回のみ呼び出されるMethod．このクラスで用いる変数の初期定義を行う．
    % Main: 毎時刻呼び出されるMethod． 実行する推定プログラムを作る．
    %% Controllerへの推定値送信
    % "sendvartype"を構造体とし，推定値の変数名とその型を定義する(以下の中から選択)．
    % Controllerで同じ変数名として取り出すことが可能．
    % 型によって配列/行列への対応可否があります:
    % 行列可: {'int8','uint8','int16','uint16','int32','uint32','int64','uint64','single','double'}
    % 不可: {'string','char'}
    % "send"を構造体とし，送信したいデータを対応する変数に代入．
    %% 推定結果の保存
    % 結果を.matとして保存し，plotResultで使用可能．
    % "result"を構造体とし，保存したい値を格納．

    properties (Constant)
        


    end

    properties
        %=======DO NOT DELETE======
        sendvartype
        %==========================
        Allxhat % Example
        mode
        udd


    end

    methods
        function obj = Estimate2(mode,OfflinePath)            
            obj.mode = mode;
            


            % Send variable type set to Controller
            obj.sendvartype.xhat = 'double'; % example
            
            if obj.mode == 1
                % Load matfile
                obj.udd = load(OfflinePath);
            end

        end

        function [result,send] = main(obj,sensordata,Plant,dt,T)
            result.RawData = sensordata;
            result.Plant = Plant;

            if ~isempty(sensordata.LIDAR)
                % ptCloud = pointCloud(rosReadXYZ(sensordata.LIDAR));
            end
            % ptCloud = pointCloud(rosReadXYZ(sensordata.LIDAR));
            if ~isempty(sensordata.CAMERA)
                % fusion();
            end
            if ~isempty(sensordata.GNSS)
            end
            

            obj.Allxhat = zeros(4,5);
            
            
            % Save data
            % 車椅子の入力 ---------------------
            result.V = [0.3;0]; 
            % --------------------------------
            result.xhat = obj.Allxhat; % example
            
            

            % send to Controller via ROS2 topic            
            send.xhat = obj.Allxhat; % example
            



        end


    end




end
