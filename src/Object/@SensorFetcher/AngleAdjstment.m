function ret = AngleAdjstment(ret, Data)
    while ret.Roll - Data.Roll > pi
       ret.Roll = ret.Roll - 2 * pi;
    end

    while ret.Roll - Data.Roll <= -pi
       ret.Roll = ret.Roll + 2 * pi;
    end

    while ret.Pitch - Data.Pitch > pi
       ret.Pitch = ret.Pitch - 2 * pi;
    end

    while ret.Pitch - Data.Pitch <= -pi
       ret.Pitch = ret.Pitch + 2 * pi;
    end

    while ret.Yaw - Data.Yaw > pi
       ret.Yaw = ret.Yaw - 2 * pi;
    end

    while ret.Yaw - Data.Yaw <= -pi
       ret.Yaw = ret.Yaw + 2 * pi;
    end
end