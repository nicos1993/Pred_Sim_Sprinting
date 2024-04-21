function createExternalForces_visualization(filename,data,timeNodes)

header = [filename '.mot'];
% time = linspace(0,1,100);
% data = ones(100,18);
% data(:,[1:3,10:12]) = 500.*rand(100,6);
% data(:,[5,14]) = 0.0;
% data(:,[7:9,16:18]) = 0.0;
% data(:,13) = 0.2;

new_data_r = zeros(length(timeNodes),7*9);
new_data_r(:,1:3) = data.r_sph_F_1_G';
new_data_r(:,4:6) = data.r_sph_1_G';
new_data_r(:,10:12) = data.r_sph_F_2_G';
new_data_r(:,13:15) = data.r_sph_2_G';
new_data_r(:,19:21) = data.r_sph_F_3_G';
new_data_r(:,22:24) = data.r_sph_3_G';
new_data_r(:,28:30) = data.r_sph_F_4_G';
new_data_r(:,31:33) = data.r_sph_4_G';
new_data_r(:,37:39) = data.r_sph_F_5_G';
new_data_r(:,40:42) = data.r_sph_5_G';
new_data_r(:,46:48) = data.r_sph_F_6_G';
new_data_r(:,49:51) = data.r_sph_6_G';
new_data_r(:,55:57) = data.r_sph_F_7_G';
new_data_r(:,58:60) = data.r_sph_7_G';

new_data_l = zeros(length(timeNodes),7*9);
new_data_l(:,1:3) = data.l_sph_F_1_G';
new_data_l(:,4:6) = data.l_sph_1_G';
new_data_l(:,10:12) = data.l_sph_F_2_G';
new_data_l(:,13:15) = data.l_sph_2_G';
new_data_l(:,19:21) = data.l_sph_F_3_G';
new_data_l(:,22:24) = data.l_sph_3_G';
new_data_l(:,28:30) = data.l_sph_F_4_G';
new_data_l(:,31:33) = data.l_sph_4_G';
new_data_l(:,37:39) = data.l_sph_F_5_G';
new_data_l(:,40:42) = data.l_sph_5_G';
new_data_l(:,46:48) = data.l_sph_F_6_G';
new_data_l(:,49:51) = data.l_sph_6_G';
new_data_l(:,55:57) = data.l_sph_F_7_G';
new_data_l(:,58:60) = data.l_sph_7_G';

new_data = [new_data_r new_data_l];

labels = {'ground_force_vx','ground_force_vy','ground_force_vz',... % Spheres on the right foot
    'ground_force_px','ground_force_py','ground_force_pz',...
    'ground_torque_x','ground_torque_y','ground_torque_z',...
    '1_ground_force_vx','1_ground_force_vy','1_ground_force_vz',...
    '1_ground_force_px','1_ground_force_py','1_ground_force_pz',...
    '1_ground_torque_x','1_ground_torque_y','1_ground_torque_z',...
    '2_ground_force_vx','2_ground_force_vy','2_ground_force_vz',...
    '2_ground_force_px','2_ground_force_py','2_ground_force_pz',...
    '2_ground_torque_x','2_ground_torque_y','2_ground_torque_z',...
    '3_ground_force_vx','3_ground_force_vy','3_ground_force_vz',...
    '3_ground_force_px','3_ground_force_py','3_ground_force_pz',...
    '3_ground_torque_x','3_ground_torque_y','3_ground_torque_z',...
    '4_ground_force_vx','4_ground_force_vy','4_ground_force_vz',...
    '4_ground_force_px','4_ground_force_py','4_ground_force_pz',...
    '4_ground_torque_x','4_ground_torque_y','4_ground_torque_z',...
    '5_ground_force_vx','5_ground_force_vy','5_ground_force_vz',...
    '5_ground_force_px','5_ground_force_py','5_ground_force_pz',...
    '5_ground_torque_x','5_ground_torque_y','5_ground_torque_z',...
    '6_ground_force_vx','6_ground_force_vy','6_ground_force_vz',...
    '6_ground_force_px','6_ground_force_py','6_ground_force_pz',...
    '6_ground_torque_x','6_ground_torque_y','6_ground_torque_z',...
    '7_ground_force_vx','7_ground_force_vy','7_ground_force_vz',...
    '7_ground_force_px','7_ground_force_py','7_ground_force_pz',...
    '7_ground_torque_x','7_ground_torque_y','7_ground_torque_z',...
    '8_ground_force_vx','8_ground_force_vy','8_ground_force_vz',...
    '8_ground_force_px','8_ground_force_py','8_ground_force_pz',...
    '8_ground_torque_x','8_ground_torque_y','8_ground_torque_z',...
    '9_ground_force_vx','9_ground_force_vy','9_ground_force_vz',... % Spheres on the left foot
    '9_ground_force_px','9_ground_force_py','9_ground_force_pz',...
    '9_ground_torque_x','9_ground_torque_y','9_ground_torque_z',...
    '10_ground_force_vx','10_ground_force_vy','10_ground_force_vz',...
    '10_ground_force_px','10_ground_force_py','10_ground_force_pz',...
    '10_ground_torque_x','10_ground_torque_y','10_ground_torque_z',...
    '11_ground_force_vx','11_ground_force_vy','11_ground_force_vz',...
    '11_ground_force_px','11_ground_force_py','11_ground_force_pz',...
    '11_ground_torque_x','11_ground_torque_y','11_ground_torque_z',...
    '12_ground_force_vx','12_ground_force_vy','12_ground_force_vz',...
    '12_ground_force_px','12_ground_force_py','12_ground_force_pz',...
    '12_ground_torque_x','12_ground_torque_y','12_ground_torque_z',...
    '13_ground_force_vx','13_ground_force_vy','13_ground_force_vz',...
    '13_ground_force_px','13_ground_force_py','13_ground_force_pz',...
    '13_ground_torque_x','13_ground_torque_y','13_ground_torque_z'};

fid = fopen([filename '.mot'],'w');
% Write the header
fprintf(fid, '%s\n', header);
fprintf(fid, '%s\n', 'version=1');
fprintf(fid, '%s\n', ['nRows=' num2str(length(timeNodes))]);
fprintf(fid, '%s\n', ['nColumns=' num2str(length(labels)+1)]);
fprintf(fid, '%s\n', 'inDegrees=yes');
fprintf(fid, '%s\n', 'endheader');

% Write column names
fprintf(fid,'time');
for i = 1:length(labels)
    fprintf(fid,'\t%s', labels{i});
end
fprintf(fid,'\n');

for i = 1:length(timeNodes)
    fprintf(fid, '%f', timeNodes(i));
    for j = 1:length(labels)
        fprintf(fid, '\t%f', new_data(i,j));
    end
    fprintf(fid,'\n');
end

% Close the file
fclose(fid);

% The polarity of the force is switched here; the force applied by the
% human to the ground - action force
summed_new_data = - [new_data_r(:,1:3) + new_data_r(:,10:12) + new_data_r(:,19:21) + new_data_r(:,28:30) + new_data_r(:,37:39) + new_data_r(:,46:48) + new_data_r(:,55:57)];

summed_moments_about_origin = zeros(length(timeNodes),3);
COP_from_origin = zeros(length(timeNodes),3);
for i = 1:length(timeNodes)
    summed_moments_about_origin(i,:) = summed_moments_about_origin(i,:) + cross(new_data_r(i,4:6),new_data_r(i,1:3));
    summed_moments_about_origin(i,:) = summed_moments_about_origin(i,:) + cross(new_data_r(i,13:15),new_data_r(i,10:12));
    summed_moments_about_origin(i,:) = summed_moments_about_origin(i,:) + cross(new_data_r(i,22:24),new_data_r(i,19:21));
    summed_moments_about_origin(i,:) = summed_moments_about_origin(i,:) + cross(new_data_r(i,31:33),new_data_r(i,28:30));
    summed_moments_about_origin(i,:) = summed_moments_about_origin(i,:) + cross(new_data_r(i,40:42),new_data_r(i,37:39));
    summed_moments_about_origin(i,:) = summed_moments_about_origin(i,:) + cross(new_data_r(i,49:51),new_data_r(i,46:48));
    summed_moments_about_origin(i,:) = summed_moments_about_origin(i,:) + cross(new_data_r(i,58:60),new_data_r(i,55:57));

    COP_from_origin(i,1) = -summed_moments_about_origin(i,3) / summed_new_data(i,2);
    COP_from_origin(i,3) = summed_moments_about_origin(i,1) / summed_new_data(i,2);
end

summed_new_data = [-summed_new_data COP_from_origin summed_moments_about_origin]; % the summed_moments_about_origin is incorrect information

labels_new = {'ground_force_vx','ground_force_vy','ground_force_vz',... 
    'ground_force_px','ground_force_py','ground_force_pz',...
    'ground_torque_x','ground_torque_y','ground_torque_z'};

fid_2 = fopen([filename '_Single.mot'],'w');
% Write the header
fprintf(fid_2, '%s\n', header);
fprintf(fid_2, '%s\n', 'version=1');
fprintf(fid_2, '%s\n', ['nRows=' num2str(length(timeNodes))]);
fprintf(fid_2, '%s\n', ['nColumns=' num2str(length(labels_new)+1)]);
fprintf(fid_2, '%s\n', 'inDegrees=yes');
fprintf(fid_2, '%s\n', 'endheader');

% Write column names
fprintf(fid_2,'time');
for i = 1:length(labels_new)
    fprintf(fid_2,'\t%s', labels_new{i});
end
fprintf(fid_2,'\n');

for i = 1:length(timeNodes)
    fprintf(fid_2, '%f', timeNodes(i));
    for j = 1:length(labels_new)
        fprintf(fid_2, '\t%f', summed_new_data(i,j));
    end
    fprintf(fid_2,'\n');
end

% Close the file
fclose(fid_2);

%