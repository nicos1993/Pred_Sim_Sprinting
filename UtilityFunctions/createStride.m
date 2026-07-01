function [] = createStride(output,fileName_mot,fileName_act,n_strides)

labels = {'time','/jointset/ground_pelvis/pelvis_tilt/value','/jointset/ground_pelvis/pelvis_list/value','/jointset/ground_pelvis/pelvis_rotation/value',...
    '/jointset/ground_pelvis/pelvis_tx/value','/jointset/ground_pelvis/pelvis_ty/value','/jointset/ground_pelvis/pelvis_tz/value',...
    '/jointset/hip_r/hip_flexion_r/value','/jointset/hip_r/hip_adduction_r/value','/jointset/hip_r/hip_rotation_r/value','/jointset/knee_r/knee_angle_r/value',...
    '/jointset/ankle_r/ankle_angle_r/value','/jointset/subtalar_r/subtalar_angle_r/value','/jointset/mtp_r/mtp_angle_r/value',...
    '/jointset/hip_l/hip_flexion_l/value','/jointset/hip_l/hip_adduction_l/value','/jointset/hip_l/hip_rotation_l/value','/jointset/knee_l/knee_angle_l/value',...
    '/jointset/ankle_l/ankle_angle_l/value','/jointset/subtalar_l/subtalar_angle_l/value','/jointset/mtp_l/mtp_angle_l/value',...
    '/jointset/back/lumbar_extension/value','/jointset/back/lumbar_bending/value','/jointset/back/lumbar_rotation/value',...
    '/jointset/acromial_r/arm_flex_r/value','/jointset/acromial_r/arm_add_r/value','/jointset/acromial_r/arm_rot_r/value','/jointset/elbow_r/elbow_flex_r/value',...
    '/jointset/radioulnar_r/pro_sup_r/value','/jointset/radius_hand_r/wrist_flex_r/value','/jointset/radius_hand_r/wrist_dev_r/value',...
    '/jointset/acromial_l/arm_flex_l/value','/jointset/acromial_l/arm_add_l/value','/jointset/acromial_l/arm_rot_l/value','/jointset/elbow_l/elbow_flex_l/value',...
    '/jointset/radioulnar_l/pro_sup_l/value','/jointset/radius_hand_l/wrist_flex_l/value','/jointset/radius_hand_l/wrist_dev_l/value'};

timeGrid = output.optimumOutput.timeGrid - output.optimumOutput.timeGrid(1);

% Remove shared collocation boundaries from exported stride files only.
timeTol = 1e-10;
uniqueTimeIdx = 1;
for iTime = 2:length(timeGrid)
    if timeGrid(iTime) > timeGrid(uniqueTimeIdx(end)) + timeTol
        uniqueTimeIdx(end+1,1) = iTime;
    end
end
timeGrid = timeGrid(uniqueTimeIdx);

q = output.optimumOutput.optVars_nsc.q';
q_orig = q(uniqueTimeIdx,:);
q_sym = q_orig;
    
q_sym(:,7:13) = q_orig(:,14:20);
q_sym(:,14:20) = q_orig(:,7:13);

q_sym(:,5:6) = q_orig(:,5:6);
q_sym(:,[1,21]) = q_orig(:,[1,21]);

q_sym(:,24:30) = q_orig(:,31:37);
q_sym(:,31:37) = q_orig(:,24:30);

q_sym(:,[2:3,22:23]) = -q_orig(:,[2:3,22:23]);
q_sym(:,4) = q_sym(:,4) + abs(q_orig(1,4)) + abs(q_orig(end,4));

stride_timeGrid = [timeGrid; timeGrid(2:end) + timeGrid(end)];
stride_q = [q_orig; q_sym(2:end,:)];


stride_q(:,[1:3,7:37]) = rad2deg(stride_q(:,[1:3,7:37]));

load('activations_labels.mat');

act = output.optimumOutput.optVars_nsc.act';
act_orig = act(uniqueTimeIdx,:);
    
act_sym = act_orig;
    
act_sym(:,1:46) = act_orig(:,47:end);
act_sym(:,47:end) = act_orig(:,1:46);

stride_act = [act_orig; act_sym(2:end,:)];

t = stride_timeGrid;
q_mat = stride_q;
stride_act_upd = stride_act;

for i = 2:n_strides
    t = [t; stride_timeGrid(2:end) + t(end)];

    q_next = stride_q(2:end,:);
    q_next(:,4) = q_next(:,4) + q_mat(end,4) - stride_q(1,4);
    q_mat = [q_mat; q_next];

    stride_act_upd = [stride_act_upd; stride_act(2:end,:)];
end

motData.labels = labels;
motData.data   = [t q_mat];
write_motionFile(motData,['symmetric_stride_' fileName_mot]);
    
actData.labels = labels_act;
actData.data   = [t stride_act_upd];
    
write_storageFile(actData,['symmetric_stride_' fileName_act]);

end
