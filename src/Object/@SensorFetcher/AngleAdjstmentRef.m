function ret = AngleAdjstmentRef(ret, obj)
    while ret.Autowareroll - obj.Data.roll(1,obj.Count-1) > pi
        ret.Autowareroll = ret.Autowareroll - 2 * pi;
    end

    while ret.Autowareroll - obj.Data.roll(1,obj.Count-1) <= -pi
        ret.Autowareroll = ret.Autowareroll + 2 * pi;
    end
    while ret.Autowarepitch - obj.Data.pitch(1,obj.Count-1) > pi
        ret.Autowarepitch = ret.Autowarepitch - 2 * pi;
    end

    while ret.Autowarepitch - obj.Data.pitch(1,obj.Count-1) <= -pi
        ret.Autowarepitch= ret.Autowarepitch+ 2 * pi;
    end
    while ret.Autowareyaw - obj.Data.yaw(1,obj.Count-1) > pi
        ret.Autowareyaw = ret.Autowareyaw - 2 * pi;
    end

    while ret.Autowareyaw - obj.Data.yaw(1,obj.Count-1) <= -pi
        ret.Autowareyaw = ret.Autowareyaw + 2 * pi;
    end
end