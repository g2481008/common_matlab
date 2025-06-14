classdef Localization_func_test< handle
    properties
        Current_pose
        Pose_tf
        params
        Est_State
        Current_dt
        Count
        buffer
        bufferNum
        Matching_pose
    end
    methods
        function obj = Localization_func_test(NDTparams,init_pose)
            obj.params.Voxelsize = NDTparams.Voxelsize;
            obj.params.InitialTransform = init_pose;
            obj.params.OutlierRatio = NDTparams.OutlierRatio;
            obj.params.MaxIterations = NDTparams.MaxIterations;
            obj.params.Tolerance = NDTparams.Tolerance;
            obj.Est_State.nowTime = 0;
            obj.Est_State.oldTime = 0;
            obj.bufferNum = NDTparams.bufferNum;
            obj.Count = 1;
        end

        function obj = main(obj,points_raw)
            if obj.Count == 1
                %%%%% Pre-Processing %%%%%
                obj.Est_State.nowTime = tic;
                obj.buffer.pointcloud = pointCloud.empty(obj.bufferNum,0);
                obj.buffer.transform = nan(obj.bufferNum,12);
                obj.buffer.pointcloud(obj.Count) = pointCloud(points_raw);
                obj.buffer.transform(obj.Count,:) = obj.params.InitialTransform;
                obj.Matching_pose = rigidtform3d();
            else
                %%%%% Pre-Processing %%%%%
                obj.Current_dt = toc(obj.Est_State.nowTime) - obj.Est_State.oldTime;
                obj.Est_State.oldTime = toc(obj.Est_State.nowTime);

                %%%%% Prediction (EKF) %%%%%
                % Predict_pose = EKF_predict_fun();

                %%%%% NDT_matching %%%%%
                targetidx = min(obj.Count,obj.bufferNum);
                TF_PointCloud = arrayfun(@(i) pctransform(obj.buffer.pointcloud(i),...
                                   rigidtform3d(obj.buffer.transform(i,4:6), obj.buffer.transform(i,1:3))), 1:targetidx-1);
                fixedpointcloud = cat(1,TF_PointCloud(1:targetidx-1).Location);
                obj.Matching_pose = pcregisterndt(pointCloud(points_raw),pointCloud(fixedpointcloud),obj.params.Voxelsize,'InitialTransform',rigidtform3d(obj.buffer.transform(targetidx-1,4:6), obj.buffer.transform(targetidx-1,1:3)),...
                    'OutlierRatio',obj.params.OutlierRatio,'MaxIterations',obj.params.MaxIterations,'Tolerance',obj.params.Tolerance,'Verbose',false);
                if obj.bufferNum >= obj.Count
                    obj.buffer.pointcloud(targetidx) = pointCloud(points_raw);
                    obj.buffer.transform(targetidx,:) = [obj.Matching_pose.Translation, rotm2eul(obj.Matching_pose.R,"ZYX"), zeros(1,6)];
                else
                    %%% X m毎に地図の更新を行うように地図を更新 & 固定点群のダウンサンプリング
                    obj.buffer.pointcloud = circshift(obj.buffer.pointcloud,-1,2);
                    obj.buffer.transform = circshift(obj.buffer.transform,-1,1);
                    obj.buffer.pointcloud(targetidx) = pointCloud(points_raw);
                    obj.buffer.transform(targetidx,:) = [obj.Matching_pose.Translation, rotm2eul(obj.Matching_pose.R,"ZYX"), zeros(1,6)];
                end
            end
            obj.Count = obj.Count+1;
        end

        function [Prediction_pose, Prediction_cov] = EKF_predict_fun(dt, prior_pose)
            [F, dFdx] = predict_model(dt,prior_pose);
            Prediction_pose = prior_pose + F;
            d
        end

        function [F, dFdx] = predict_model(dt,prior_pose)
            dF = [1, sin(prior_pose(4))*sin(prior_pose(5))/cos(prior_pose(5)), cos(prior_pose(4))*sin(prior_pose(5))/cos(prior_pose(5))
                  0, cos(prior_pose(4)), -sin(prior_pose(4))
                  0, sin(prior_pose(4))/cos(prior_pose(5)), cos(prior_pose(4))/cos(prior_pose(5))];
            F = [prior_pose(7)*dt, prior_pose(8)*dt, prior_pose(9)*dt, dF*[prior_pose(10)*dt, prior_pose(11)*dt, prior_pose(12)*dt], 0, 0, 0, 0, 0, 0];
            dFdx1_1 = 1; dFdx1_2 = 0; dFdx1_3 = 0; dFdx1_4 = 0; dFdx1_5 = 0; dFdx1_6 = 0; dFdx1_7 = dt; dFdx1_8 = 0; dFdx1_9 = 0; dFdx1_10 = 0; dFdx1_11 = 0; dFdx1_12 = 0;
            dFdx2_1 = 0; dFdx2_2 = 1; dFdx2_3 = 0; dFdx2_4 = 0; dFdx2_5 = 0; dFdx2_6 = 0; dFdx2_7 = 0; dFdx2_8 = dt; dFdx2_9 = 0; dFdx2_10 = 0; dFdx2_11 = 0; dFdx2_12 = 0;
            dFdx3_1 = 0; dFdx3_2 = 0; dFdx3_3 = 1; dFdx3_4 = 0; dFdx3_5 = 0; dFdx3_6 = 0; dFdx3_7 = 0; dFdx3_8 = 0; dFdx3_9 = dt; dFdx3_10 = 0; dFdx3_11 = 0; dFdx3_12 = 0; 
            dFdx4_1 = 0; dFdx4_2 = 0; dFdx4_3 = 0;
            dFdx4_4 = 1+prior_pose(8)*(cos(prior_pose(7))*sin(prior_pose(8))/cos(prior_pose(8)))-prior_pose(9)*(sin(prior_pose(7))*sin(prior_pose(8))/cos(prior_pose(8)));
            dFdx4_5 = prior_pose(8) * sin(prior_pose(7))/cos(prior_pose(8))^2;
            dFdx4_6 = 0; dEdx4_7 = 0;dFdx4_8 = 0; dFdx4_9 = 0;
            dFdx4_10 = 1; dFdx4_11 = sin(prior_pose(7))*sin(prior_pose(8))/cos(prior_pose(8)); dFdx4_12 = cos(prior_pose(7))*sin(prior_pose(8))/cos(prior_pose(8));
            dFdx5_1 = 0;
            dFdx = [dFdx1_1, dFdx1_2, dFdx1_3, dFdx1_4, dFdx1_5, dFdx1_6, dFdx1_7, dFdx1_8, dFdx1_9, dFdx1_10, dFdx1_11, dFdx1_12;
                    dFdx2_1, dFdx2_2];
        end
    end
end