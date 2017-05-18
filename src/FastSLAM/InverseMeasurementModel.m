function WorldLandmark = InverseMeasurementModel(pose, z, intrin)
    c_psi = cos(pose(6));
    s_psi = sin(pose(6));
    c_theta = cos(pose(5));
    s_theta = sin(pose(5));
    c_phi = cos(pose(4));
    s_phi = sin(pose(4));

    % Rotation matrix corresponding to: BC_R' * EB_R'
    R = [(-c_psi*s_theta*s_phi + s_psi*c_phi), (-s_psi*s_theta*s_phi-c_psi*c_phi), -(c_theta*s_phi);
            (-c_psi*s_theta*c_phi-s_psi*s_phi),  (-s_psi*s_theta*c_phi+c_psi*s_phi), -(c_theta*c_phi);
            (c_psi*c_theta),                     (s_psi*c_theta),                    -(s_theta)];

    c_zl = z(3);
    c_xl = (z(1)*c_zl - intrin.ppx*c_zl) / intrin.fx;
    c_yl = (z(2)*c_zl - intrin.ppy*c_zl) / intrin.fy;
    
    CamLandmark = [c_xl, c_yl, c_zl]';
    
    pose_xyz = [pose(1), pose(2), pose(3)]';

    TempLandmark = CamLandmark + R*pose_xyz;

    WorldLandmark = R' * TempLandmark; % rot.transpose() corresponds to EB_R * BC_R