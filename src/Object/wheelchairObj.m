classdef wheelchairObj < handle
    properties
        cnt = 0; % Time step
        sensorIdx
        ros2comm
        EstClass
        DL
        SF
        Timer
        EstTimer
        mySaveFileName
        SelfEstPos
        standard
        nostd = 1
        tspan
        isprocessed = 0;
        vehicleType
        mode
        isParalell = false
        isMultiPC = false
        RID = 0
        OfflinePath
    end

    properties(Constant)
        sensorName = ["LIDAR","GNSS","CAMERA"];
        
        
        
    end
    
    methods
        function obj = wheelchairObj(sensor,EstClass,SF,Timer,base_sensor,ts,vehicleType,mode,isParalell,isMultiPC,RID,OfflinePath)
            obj.sensorIdx = sensor;
            obj.EstClass = EstClass;
            obj.SF = SF;
            obj.EstTimer = Timer;
            obj.Timer = Timer;
            if base_sensor > 0
                obj.standard = obj.sensorName(base_sensor);
                obj.nostd = 0;
            else
                obj.standard = obj.sensorName(1);
            end
            obj.tspan = ts;
            obj.vehicleType = vehicleType;
            obj.mode = mode;
            obj.isParalell = isParalell;
            obj.isMultiPC = isMultiPC;
            obj.RID = RID;
            obj.OfflinePath = OfflinePath;
            % NDTParams = struct("Voxelsize",1.0,"OutlierRatio",0.1,"MaxIterations",50,"Tolerance",[0.01,0.1],"bufferNum",30);
            % init_pose = zeros(1,12);
            % obj.SelfEstPos = Localization_func_test(NDTParams, init_pose);
        end
        
        function mainLoop(obj)
            persistent node
            sendvartype = obj.EstClass.sendvartype;
            sendvartype.sequence = 'uint32';

            % modeSpecifier(obj.mode)
            if obj.mode == 2
                setenv('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp')
                setenv("FASTDDS_BUILTIN_TRANSPORTS","UDPv4") % Avoid SHM communication
                setenv("ROS_LOCALHOST_ONLY","1")
            end
            if obj.mode == 3
                setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')
                setenv("FASTDDS_BUILTIN_TRANSPORTS","UDPv4") % Avoid SHM communication
                setenv("ROS_LOCALHOST_ONLY","0")
            end
            if obj.mode > 1
                % ROS2 TOPIC Configuration
                if isempty(node) || ~isvalid(node)
                    disp("Establishing ROS2 Node...")
                    rosshutdown
                    pause(5)
                    node = ros2node("matlab",obj.RID);
                end
                obj.ros2comm = ROS2CommManager(node,obj.vehicleType,obj.mode,obj.sensorIdx);
            else
                node = [];
            end
            switch obj.mode
                case 1 % Offline (No sensor, Use matlab only)                    
                    if obj.isParalell
                        warning("hoge")
                    end
                    if obj.isMultiPC
                        warning("hoge")
                    end
                    % disp("Loading mat file...")
                    % Umat = load(obj.OfflinePath);
                    sensorSubs = [];
                    whillSubs = [];
                case 2 % Gazebo ros2 % Shared memory
                    disp("Creating Publisher/Subscriber for Gazebo...")
                    if obj.isMultiPC
                        warning("No compatible in Gazebo mode: isMultiPC=ture")
                    end
                    sensorSubs = obj.ros2comm.genSensorSubs();
                    whillSubs = obj.ros2comm.genWhillSubs();
                    %-----------------一時的に追加．デバッグ終了後は削除------------
                    [whillPubs,cmdvel_msg] = obj.ros2comm.genWhillPubs();
                    %---------------------------------------------                           
                case 3 % EXP ros2
                  disp("Creating Publisher/Subscriber...")
                    if obj.isParalell                        
                        sensorSubs = obj.ros2comm.genSensorSubs();
                        whillSubs = obj.ros2comm.genWhillSubs();
                        [whillPubs,cmdvel_msg] = obj.ros2comm.genWhillPubs();
                        if obj.isMultiPC % EstとCtrl間にROS2使用(PC2台以上使用限定)
                            [pubs,msgs,EstVarName] = obj.ros2comm.genEstimatorPubs(sendvartype,obj.mySaveFileName);                            
                        end
                    elseif ~obj.isParalell && ~obj.isMultiPC % Direct exec.(Old matlab program flow)
                        sensorSubs = obj.ros2comm.genSensorSubs();
                        whillSubs = obj.ros2comm.genWhillSubs();
                        [whillPubs,cmdvel_msg] = obj.ros2comm.genWhillPubs();
                    else
                        error("Invalid variable: isParalell, isMultiPC")
                    end
                        
                    
                otherwise
                    error("Invalid mode number.")

            end

            
            

            
            
            % 割り込みストップ用Figure
            global stopFlag
            stopFlag = false;
            fig = figure('Name','WheelChair Common', 'KeyPressFcn',@myKeyPressFcn);

            
            
            EstData = []; % Estimation result

            % System execution
            i = 1;
            j = 1;
            
            waitPressEnterkey()
            Sys = tic;
            obj.EstTimer.start();
            obj.Timer.start();
            while ~stopFlag
                St(i) = toc(Sys);
                obj.cnt = obj.cnt+1; % Update step

                % Get sensor data
                [sensorData, Plant] = obj.SF.getSensorData(obj.sensorIdx,sensorSubs,whillSubs,obj.vehicleType,obj.SelfEstPos);
                
                
                % Write sensor data in SHM for Estimator
                    
                % Read from Controller SHM 

                % Estimator
                if obj.nostd || ~isempty(sensorData.(obj.standard)) || obj.mode == 1
                    elapsed = obj.EstTimer.elapsed();
                    [result,EstData] = obj.EstClass.main(sensorData,Plant,obj.EstTimer.interval(),elapsed);
                    EstData.sequence = uint32(obj.cnt);
                    result.send = EstData;
                    result.T = elapsed;
                    obj.DL.addData(result)
                    obj.isprocessed = 1;
                    disp(j)
                    j = j+1;
                end

                % Send to controller
                % ------一時的に削除．デバッグ終了後はコメント解除------
                % send_msgs_toCtrl(obj.cnt,EstData,EstVarName,pubs,msgs)
                %---------------------------------------------------
                
                % ------一時的に追加．デバッグ終了後は削除------
                if obj.mode ~= 1 && obj.isprocessed
                    obj.ros2comm.send_msgs_toWhill(whillPubs,cmdvel_msg,result.V)
                end
                %--------------------------------------------
                
                if obj.isprocessed
                    obj.isprocessed = 0;
                    while ~obj.Timer.hasElapsed(obj.tspan)
                    end
                end
                SysTime(i) = toc(Sys) - St(i);
                i = i+1;
                % pause(1e-3)
                drawnow
            end
            % obj.ros2comm.stopController();

            if ishandle(fig)
                close(fig);
            end

            % Output result
            obj.DL.stop();
            disp("Saving data...")
            while obj.DL.isDone() == 0
            end
            disp("Done")

            % Calculation time
            figure(12)
            plot(St,mean(SysTime)*ones(length(SysTime),1),'-r'); hold on;
            plot(St,SysTime,'b-');
            legend({'Mean','Whole time'})
            
            
        end
        function systemExec(obj)
            % ここでモードによる処理を分ける

            



        end
    end

    
end

function myKeyPressFcn(~, event)
    global stopFlag
    fprintf('Stopping system...');
    stopFlag = true;
end

