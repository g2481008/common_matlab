classdef Control2 < handle
    %UNTITLED4 このクラスの概要をここに記述
    %   詳細説明をここに記述
    properties(Constant)

    end

    properties
        vehicleType
        isMultiPC
        controller
    end

    methods
        function obj = Control2()
            %===========DO NOT DELETE==========
            % System configuration
            obj.vehicleType = 1; % 1:CR, 2:CR2
            obj.isMultiPC = false;
            %==================================
            % obj.controller = controllerPurePursuit(DesiredLinearVelocity=0.02,LookaheadDistance=0.3,Waypoints=[1,0;2,0]);
        end

        function result = main(obj,Estimator)
            % data.RawData.SelfPos
            % [U,Omega,~] = obj.controller([data.selfpos(1:2),0]); % [x y theta]


            result = Estimator;
            result.V = [0;0]; %[U;Omega]; % wheelchair cmd_vel
            result.Q = magic(5);
            
        end
    end
end