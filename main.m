clc; clear; close all;
%% 使用方法：
% mainとは異なるMATLABセッションでexeControlを開く
% exeControl⇒mainの順で実行

% 推奨：実験前に一度実行することで処理速度が向上します



%% Global configurations
clc; close all; clear global; clear variables; warning('off','all');
conf.pc = [ismac; isunix; ispc];
conf.mk = [":"; ":"; ";"];
conf.def= "AppData/Local/Temp/Editor";
conf.usr= "./src";
conf.fpath = split(path, conf.mk(conf.pc));
conf.fcheck= and(~contains(conf.fpath, matlabroot), ~contains(conf.fpath, conf.def));
rmpath(strjoin(conf.fpath(conf.fcheck), conf.mk(conf.pc)));
addpath(genpath(conf.usr));

%%
% pool = gcp('nocreate');
% if isempty(pool)
%     parpool;
% end
vehicleType = 1; % 1:CR, 2:CR2
sensor(1) = true; % LiDAR
sensor(2) = false; % GNSS
sensor(3) = false; % Camera
sensor(4) = true; % SLAM/Localization
base_sensor = 1; % Standard sensor you use mainly. No standard:0, LiDAR:1, GNSS:2, Camera:3

ts = 0.1; % System execution frequency [s]

mode = 2; % 1: Offline (No communicate), 2:Gazebo simulation(ROS2), 3: Experiment(ROS2)
OfflinePath = "/home/student/Program/matlab_common/data/20250613/20250613_171110/userLocal.mat"; % Mat file

isParalell = false; % 非同期処理
isMultiPC = false; % 推定と制御を2台PCで分散処理 (isParalell = 1)

RID = 17; % ROS Domain ID


Timer = TimeTracker();
myEstimator = Estimate2(mode);
SF = SensorFetcher(mode);
obj = wheelchairObj(sensor,myEstimator,SF,Timer,base_sensor,ts,vehicleType,mode,isParalell,isMultiPC,RID,OfflinePath);
mySavePath = './data';
obj.mySaveFileName = string(datetime("now","Format","yyyyMMdd_HHmmss"));
Datadir = strcat(mySavePath,filesep,string(datetime("now","Format","yyyyMMdd")),filesep,obj.mySaveFileName);
mkdir(Datadir)
obj.DL = DataLogger(Datadir,"Estimate");

obj.mainLoop();

%%
plotResult(Datadir,1)

function plotResult(folderPath,keepAllSequences,mode)
    if mode ~= 1
        plot_preprocesser(folderPath,keepAllSequences)
        load(strcat(folderPath,filesep,"userLocal.mat"),"userLocal")
    end    
    % userLocal:センサデータ,車椅子情報,resultで保存した変数(cell型)をstructで格納
    % How to reffer variables:
    % ============example=============
    % 必要に応じて削除してください
    vehicle_X = userLocal.X; 
    xhat = userLocal.xhat;
    % ================================




end






