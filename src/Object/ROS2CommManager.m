classdef ROS2CommManager < handle
    %UNTITLED2 このクラスの概要をここに記述
    %   詳細説明をここに記述

    properties(Constant)
        % Estimator to Contoller
        matlabType = {'int8','uint8','int16','uint16','int32','uint32','int64','uint64','single','double','string','char'};
        ros2msgType = {'std_msgs/Int8MultiArray','std_msgs/UInt8MultiArray', ...
            'std_msgs/Int16MultiArray','std_msgs/UInt16MultiArray', ...
            'std_msgs/Int32MultiArray','std_msgs/UInt32MultiArray', ...
            'std_msgs/Int64MultiArray','std_msgs/UInt64MultiArray', ...
            'std_msgs/Float32MultiArray','std_msgs/Float64MultiArray', ...
            'std_msgs/String','std_msgs/String'};
        
        % Sensor -----------------------------------------------------------------------------------------------------------       
        SensorTopicSubs = {'/velodyne_points','/ublox_gps_node/fix','/yolo/detections','/current_pose'};
        SensorMsgtypeSubs = {'sensor_msgs/PointCloud2','sensor_msgs/NavSatFix','yolo_msgs/YoloDetectionArray','geometry_msgs/PoseStamped'};        
        % Gazebo
        % /wheelchair/pose [geometry_msgs/msg/Pose]
        gSensorTopicSubs = {'/velodyne_points',[],[],'/wheelchair/pose'};
        gSensorMsgtypeSubs = {'sensor_msgs/PointCloud2',[],[],'geometry_msgs/Pose'};

        % Wheelchair {1}:CR, {2}:CR2-----------------------------------------------------------------------------------------
        % x: left wheel(positive:CCW), y:right wheel(positive:CW,clockwise), z:dummy wheel
        WhillTopicSubs = {'/Drp5_green/whill_node/motor_speed','/whill/states/model_cr2'};
        WhillTopicPubs = {'/Drp5_green/whill_node/cmd_vel','whill_msgs/ModelCr2State'};
        WhillMsgtypeSubs = {'geometry_msgs/Vector3','/whill/controller/cmd_vel'};
        WhillMsgtypePubs = {'geometry_msgs/Twist','geometry_msgs/Twist'};
        % Gazebo
        gWhillTopicSubs = {'/wheelchair/odom'};
        gWhillTopicPubs = {'/wheelchair/diff_drive_controller/cmd_vel'};
        gWhillMsgtypeSubs = {'nav_msgs/Odometry'};
        gWhillMsgtypePubs = {'geometry_msgs/TwistStamped'};
        % /wheelchair/diff_drive_controller/cmd_vel [geometry_msgs/msg/Twist, geometry_msgs/msg/TwistStamped]
        % /wheelchair/odom [nav_msgs/msg/Odometry]

        qos_profile = struct("Reliability","reliable","Durability","volatile","History","keeplast","Depth",1)
        gqos_profile = struct("Reliability","besteffort","Durability","volatile","History","keeplast","Depth",1)
        
        
    end

    properties
        varpub
        varmsg
        node
        vehicleType
        mode


    end

    methods
        function obj = ROS2CommManager(node,vehicleType,mode)
            obj.node = node;
            obj.vehicleType = vehicleType;
            obj.mode = mode;
        end

        function [pubs,msgs,varname] = genEstimatorPubs(obj,info,FileName)  
            varname = fieldnames(info);
            n = numel(varname);
            pubs = cell(n,1);
            msgs = cell(n,1);
            VarType = cell(n,1);
            for i = 1:n
                VarType{i} = info.(varname{i});
                idx = find(strcmp(VarType{i},obj.matlabType));
                if ~isempty(idx)
                    pubs{i} = ros2publisher(obj.node, ...
                        strcat("/estimation_data",string(i)), ...
                        obj.ros2msgType{idx});
                    msgs{i} = ros2message(obj.ros2msgType{idx});
                else
                    error('Unexpected variable type.')
                end
            end
            
            % Send variables' name and their type in advance
            VarName_join = strjoin(varname,',');
            VarType_join = strjoin(VarType,',');
            VarData = strjoin({VarName_join,VarType_join,char(FileName)},'.');
            obj.varpub = ros2publisher(obj.node, ...
                        "/estimation_data0", ...
                        obj.ros2msgType{11});
            obj.varmsg = ros2message(obj.ros2msgType{11});
            obj.varmsg.data = VarData;
            send(obj.varpub,obj.varmsg);
        end    

        function [subs,VarName,isNum] = genContollerSubs(obj,VarData)
            % Estimator to Controller
            VarName = split(VarData{1},',');
            VarType = split(VarData{2},',');
            n = numel(VarType);
            subs = cell(n,1);
            idx = zeros(n,1);
            for i = 1:n
                idx(i) = find(strcmp(VarType{i},obj.matlabType),1);
                if ~isempty(idx(i))
                    subs{i} = ros2subscriber(obj.node, ...
                                strcat("/estimation_data",string(i)), ...
                                obj.ros2msgType{idx(i)});        
                else
                    error('Unexpected variable type.')
                end
            end
            isNum = idx < numel(obj.matlabType)-1;
        end

        function [sensorSubs] = genSensorSubs(obj, sensorIdx)
            if obj.mode == 2 % gazebo
                m = numel(obj.gSensorTopicSubs);
                subs = obj.gSensorTopicSubs;
                msgs = obj.gSensorMsgtypeSubs;
                qos = obj.gqos_profile;
            else
                m = numel(obj.SensorTopicSubs);
                subs = obj.SensorTopicSubs;
                msgs = obj.SensorMsgtypeSubs;
                qos = obj.qos_profile;
            end
            sensorSubs = cell(m,1);
            for i = 1:m
                if sensorIdx(i)
                    sensorSubs{i} = ros2subscriber(obj.node, ...
                        subs{i},msgs{i}, ...
                        "Reliability",qos.Reliability, ...
                        "Durability",qos.Durability, ...
                        "History",qos.History, ...
                        "Depth",qos.Depth);
                else
                    sensorSubs{i} = [];
                end
            end
        end

        function [whillSubs] = genWhillSubs(obj, vtype)
            if obj.mode == 2
                subs = obj.gWhillTopicSubs;
                msgs = obj.gWhillMsgtypeSubs;
                qos = obj.gqos_profile;
            else
                subs = obj.WhillTopicSubs;
                msgs = obj.WhillMsgtypeSubs;
                qos = obj.qos_profile;
            end
            whillSubs = ros2subscriber(obj.node, ...
                subs{vtype}, ...
                msgs{vtype}, ...
                "Reliability",qos.Reliability, ...
                "Durability",qos.Durability, ...
                "History",qos.History, ...
                "Depth",qos.Depth);
        end

        function [whillPubs,msg] = genWhillPubs(obj, vtype)
            if obj.mode == 2
                pubs = obj.gWhillTopicPubs;
                msgs = obj.gWhillMsgtypePubs;
                qos = obj.gqos_profile;
            else
                pubs = obj.WhillTopicPubs;
                msgs = obj.WhillMsgtypePubs;
                qos = obj.qos_profile;
            end
            whillPubs = ros2publisher(obj.node, ...
                pubs{vtype}, ...
                msgs{vtype}, ...
                "Reliability",qos.Reliability, ...
                "Durability",qos.Durability, ...
                "History",qos.History, ...
                "Depth",qos.Depth);
            msg = ros2message(whillPubs);
        end

        function send_msgs_toCtrl(obj,cnt,EstData,EstVarName,pubs,msgs)
            if ~isempty(EstData)
                for k = 1:length(pubs)
                    if ~isa(EstData.(EstVarName{k}), 'string') && ~isa(EstData.(EstVarName{k}), 'char')
                        msgs{k}.layout.dim.label = char(join(string(size(EstData.(EstVarName{k}))),","));
                        msgs{k}.layout.data_offset = uint32(cnt); % Sequence
                    else
                        EstData.(EstVarName{k}) = char(join([string(cnt),EstData.(EstVarName{k})],","));
                    end
                    msgs{k}.data = EstData.(EstVarName{k});
                    % send(pubs{k}, msgs{k});
                end
                for k = 1:length(pubs)
                    send(pubs{k}, msgs{k});
                end
            end
        end

        function send_msgs_toWhill(obj,pubs,msgs,V)
            msgs.twist.linear.x = double(V(1));
            msgs.twist.angular.z = double(V(2));
            send(pubs,msgs)
        end

        function stopController(obj)
            obj.varmsg.data = 'stop';
            send(obj.varpub,obj.varmsg);


        end

    end
    
end