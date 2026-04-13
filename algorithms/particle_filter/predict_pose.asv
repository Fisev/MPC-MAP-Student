function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
    x  = old_pose(1);
    y  = old_pose(2);
    th = old_pose(3);
    
    vR = motion_vector(1);
    vL = motion_vector(2);
    dt = read_only_vars.sampling_period;
    read_only_vars.agent_drive.interwheel_dist;
    
    sigmaV = 0.8;
    sigmaW = 0.8;
    vWithNoise = ((vR + vL) / 2) + sigmaV * randn();
    wWithNoise = ((vR - vL) / read_only_vars.agent_drive.interwheel_dist) + sigmaW * randn();
    
    if abs(wWithNoise) < 1e-6
        xNew  = x + vWithNoise * dt * cos(th);
        yNew  = y + vWithNoise * dt * sin(th);
        thNew = th;
    else
        xNew  = x + (vWithNoise / wWithNoise) * (sin(th + wWithNoise*dt) - sin(th));
        yNew  = y - (vWithNoise / wWithNoise) * (cos(th + wWithNoise*dt) - cos(th));
        thNew = th + wWithNoise * dt;
     end
    
    thNew = wrapToPi(thNew);
    new_pose = [xNew; yNew; thNew];
end

