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

timeGrid_dummy = timeGrid + timeGrid(end);

stride_timeGrid = [timeGrid; timeGrid_dummy];

for i = 1:n_strides
    
    if i == 1
        t(:,i) = stride_timeGrid;
    else
        t(:,i) = stride_timeGrid;
        t(:,i) = t(:,i) + t(end,i-1);
    end
    
end

t = t(:);

q = output.optimumOutput.optVars_nsc.q';
q_orig = q;
q_sym = q;
    
stride_q = [q;q_sym];

stride_q(length(timeGrid)+1:end,7:13) = q_orig(1:end,14:20);
stride_q(length(timeGrid)+1:end,14:20) = q_orig(1:end,7:13);

stride_q(length(timeGrid)+1:end,5:6) = q_orig(1:end,5:6);
stride_q(length(timeGrid)+1:end,[1,21]) = q_orig(1:end,[1,21]);

stride_q(length(timeGrid)+1:end,24:30) = q_orig(1:end,31:37);
stride_q(length(timeGrid)+1:end,31:37) = q_orig(1:end,24:30);

stride_q(length(timeGrid)+1:end,[2:3,22:23]) = -q_orig(1:end,[2:3,22:23]);    
stride_q(length(timeGrid)+1:end,4) = stride_q(length(timeGrid)+1:end,4) + abs(q_orig(1,4)) + abs(q_orig(end,4));


stride_q(:,[1:3,7:37]) = rad2deg(stride_q(:,[1:3,7:37]));

for k = 1:37
    q_vec = stride_q(:,k);
    for i = 1:n_strides
        if k ~= 4
            q_t(:,i) = q_vec;
        else
            if i == 1 
                q_t(:,i) = q_vec;
            else
                q_t(:,i) = q_vec;
                q_t(:,i) = q_t(:,i) + q_t(end,i-1);
            end
        end
    end
    q_t = q_t(:);
    q_mat(:,k) = q_t;
    clear q_t;
end

if n_strides == 1
    motData.labels = labels;
    motData.data   = [stride_timeGrid stride_q];
else
    motData.labels = labels;
    motData.data   = [t q_mat];
end
write_motionFile(motData,['symmetric_stride_' fileName_mot]);

load('activations_labels.mat');

act = output.optimumOutput.optVars_nsc.act';
act_orig = act;
    
act_sym = act;
    
stride_act = [act; act_sym];
    
stride_act(length(timeGrid)+1:end,1:46) = act_orig(1:end,47:end);
stride_act(length(timeGrid)+1:end,47:end) = act_orig(1:end,1:46);

stride_act_upd = [];

for n = 1:n_strides
    stride_act_upd = [stride_act_upd; stride_act];
end
    
actData.labels = labels_act;
actData.data   = [t stride_act_upd];
    
write_storageFile(actData,['symmetric_stride_' fileName_act]);

end