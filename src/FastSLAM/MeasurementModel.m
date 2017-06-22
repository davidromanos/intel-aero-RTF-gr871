function z = MeasurementModel(pose, l, intrin, CamOffset)
    c_psi = cos(pose(6));
    s_psi = sin(pose(6));
    c_theta = cos(pose(5));
    s_theta = sin(pose(5));
    c_phi = cos(pose(4));
    s_phi = sin(pose(4));        

    xl = l(1);
    yl = l(2);
    zl = l(3);

    % EB_R = R_yaw * R_pitch * R_roll;
    EB_R =  [ (c_theta*c_psi), (c_psi*s_theta*s_phi - c_phi*s_psi), (s_phi*s_psi + c_phi*c_psi*s_theta);
              (c_theta*s_psi), (c_phi*c_psi + s_theta*s_phi*s_psi), (c_phi*s_theta*s_psi - c_psi*s_phi);
              (-s_theta),      (c_theta*s_phi),                     (c_theta*c_phi)];            
    
    CamCenter = pose(1:3) + EB_R * CamOffset;

    c_xl = (-c_psi*s_theta*s_phi + s_psi*c_phi)*(xl - CamCenter(1)) + (-s_psi*s_theta*s_phi-c_psi*c_phi)*(yl - CamCenter(2)) - (c_theta*s_phi)*(zl - CamCenter(3));
    c_yl = (-c_psi*s_theta*c_phi-s_psi*s_phi)*(xl - CamCenter(1)) + (-s_psi*s_theta*c_phi+c_psi*s_phi)*(yl - CamCenter(2)) - (c_theta*c_phi)*(zl - CamCenter(3));
    c_zl = (c_psi*c_theta)*(xl - CamCenter(1)) + (s_psi*c_theta)*(yl - CamCenter(2)) - s_theta*(zl - CamCenter(3));
    %if (c_zl < 0) c_zl = -c_zl; end

    xi = (intrin.fx*c_xl + intrin.ppx*c_zl) / c_zl;
    yi = (intrin.fy*c_yl + intrin.ppy*c_zl) / c_zl;
    zc = c_zl;

    z = [xi; yi; zc];