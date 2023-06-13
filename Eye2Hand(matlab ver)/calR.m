function R = calR(Ps_cam)
%calR calculate rotation matrix between BASE coordinate system and cAMERA
%coordinate system
%   input: Ps_cam, (3x6) x M, 6组标识球数据
    % x axis motion
    Px_1 = Ps_cam(1:4,:)'; 
    Px_2 = Ps_cam(5:8,:)';  
    r_1 = mean(Px_1-Px_2,2);  
    r_1 = r_1/norm(r_1)
    % y axis motion
    Py_1 = Ps_cam(9:12,:)';
    Py_2 = Ps_cam(13:16,:)';
    r_2 = mean(Py_1-Py_2,2);
    r_2 = r_2/norm(r_2)
    % z axis motion
    Pz = Ps_cam(17:20,:)';
    P_z = Ps_cam(21:24,:)';
    r_3 = mean(Pz-P_z,2);
    r_3 = r_3/norm(r_3)
    [R t]= Kabsch([1 0 0; 0 1 0; 0 0 1], [r_1, r_2, r_3]);
end

