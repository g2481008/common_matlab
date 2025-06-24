function ret = AngleAdjstmentRef_prime(ret, obj)
    while ret.Primeroll - obj.Data.roll(1,obj.Count-1) > pi
        ret.Primeroll = ret.Primeroll - 2 * pi;
    end

    while ret.Primeroll - obj.Data.roll(1,obj.Count-1) <= -pi
        ret.Primeroll = ret.Primeroll + 2 * pi;
    end
    while ret.Primepitch - obj.Data.pitch(1,obj.Count-1) > pi
        ret.Primepitch = ret.Primepitch - 2 * pi;
    end

    while ret.Primepitch - obj.Data.pitch(1,obj.Count-1) <= -pi
        ret.Primepitch= ret.Primepitch+ 2 * pi;
    end
    while ret.Primeyaw - obj.Data.yaw(1,obj.Count-1) > pi
        ret.Primeyaw = ret.Primeyaw - 2 * pi;
    end

    while ret.Primeyaw - obj.Data.yaw(1,obj.Count-1) <= -pi
        ret.Primeyaw = ret.Primeyaw + 2 * pi;
    end
end