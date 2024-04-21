% To run code efficiently:
% be within 'Polynomials' folder



% Function generates polynomials to approximate muscle-tendon lengths (&
% velocities) and moment arms. This code is adapted from:
% https://simtk.org/projects/3dpredictsim

%% User inputs
runPolynomialfit =1;
saveQdot = 1;
savePolynomials = 1;

%% Extract time and angles from dummy motion

% Dummy motion was created using the ROMs of each DOF and rand function
% e.g., q = (-1+(1+1)*rand(5000,1))*(90); 
% Creates random angles bounded between -90:90

pathmain = pwd; % print working directory.

% MA results and dummy motion file stored in this folder
path_resultsMA = [pathmain,'\MAresults\test_movingKnee_further_moreKneeROM_modKneeFlex_unlockedMTP\'];
addpath(genpath(path_resultsMA));
name_dummymotion = 'dummy_motion_37DOF_further_moreKneeROM.mot';

dummy_motion = importdata([path_resultsMA,name_dummymotion]);

% Model has 37 DOF
% 17 DOF are necessary here; Ones which impact upon muscle behaviour
% But assume symmetry; thus 10 DOF required
% Order can be seen from: dummy_motion.colheaders
% Order:
% hip_flex_r,hip_add_r,hip_rot_r,knee_r,ankle_r,subtalar_r,mtp_r,lumbar_ext,lumbar_bend_lumbar_rot
q = deg2rad(dummy_motion.data(:,[8:14,22:24]));

% Generate random numbers between -1000 & 1000 (°/s)
% This code is not imperative, but it is used to purely evaluate the
% velocity of muscle-tendon function
if saveQdot == 1
    a = -1000;
    b = 1000;
    r1 = (b-a).*rand(5000,1) + a;
    r2 = (b-a).*rand(size(q,1),1) + a;
    r3 = (b-a).*rand(size(q,1),1) + a;
    r4 = (b-a).*rand(size(q,1),1) + a;
    r5 = (b-a).*rand(size(q,1),1) + a;
    r6 = (b-a).*rand(size(q,1),1) + a;
    r7 = (b-a).*rand(size(q,1),1) + a;
    r8 = (b-a).*rand(size(q,1),1) + a;
    r9 = (b-a).*rand(size(q,1),1) + a;
    r10 = (b-a).*rand(size(q,1),1) + a;
    r = [r1,r2,r3,r4,r5,r6,r7,r8,r9,r10];
    %qdot = zeros(size(q));
    qdot = deg2rad(r);
    dummy_qdot = qdot;
    save([path_resultsMA,'dummy_qdot_37DOF.mat'],'dummy_qdot');
end

load([path_resultsMA,'dummy_qdot_37DOF.mat']);
qdot = dummy_qdot(:,:);

%% Import data
% lMT - length of muscle-tendon
lMT = importdata([path_resultsMA,'subject01_MuscleAnalysis_Length.sto']);
% hip flexion r
MA.hip.flex = importdata([path_resultsMA,'subject01_MuscleAnalysis_MomentArm_hip_flexion_r.sto']);
% hip adduction r
MA.hip.add = importdata([path_resultsMA,'subject01_MuscleAnalysis_MomentArm_hip_adduction_r.sto']);
% hip rotation r
MA.hip.rot = importdata([path_resultsMA,'subject01_MuscleAnalysis_MomentArm_hip_rotation_r.sto']);
% knee flexion r 
MA.knee.flex = importdata([path_resultsMA,'subject01_MuscleAnalysis_MomentArm_knee_angle_r.sto']);
% ankle flexion r
MA.ankle.flex = importdata([path_resultsMA,'subject01_MuscleAnalysis_MomentArm_ankle_angle_r.sto']);
% subtalar r
MA.sub = importdata([path_resultsMA,'subject01_MuscleAnalysis_MomentArm_subtalar_angle_r.sto']);
% MTP flex r
MA.mtp.flex = importdata([path_resultsMA,'subject01_MuscleAnalysis_MomentArm_mtp_angle_r.sto']);
% lumbar extension
MA.trunk.ext = importdata([path_resultsMA,'subject01_MuscleAnalysis_MomentArm_lumbar_extension.sto']);
% lumbar bending
MA.trunk.ben = importdata([path_resultsMA,'subject01_MuscleAnalysis_MomentArm_lumbar_bending.sto']);
% lumbar rotation
MA.trunk.rot = importdata([path_resultsMA,'subject01_MuscleAnalysis_MomentArm_lumbar_rotation.sto']);

%% Organize MuscleData
if runPolynomialfit == 1
    MuscleData.dof_names = dummy_motion.colheaders([[8:14,22:24]]);
    muscleNames = {'glut_med1_r','glut_med2_r','glut_med3_r',...
        'glut_min1_r','glut_min2_r','glut_min3_r','semimem_r',...
        'semiten_r','bifemlh_r','bifemsh_r','sar_r','add_long_r',...
        'add_brev_r','add_mag1_r','add_mag2_r','add_mag3_r','tfl_r',...
        'pect_r','grac_r','glut_max1_r','glut_max2_r','glut_max3_r',......
        'iliacus_r','psoas_r','quad_fem_r','gem_r','peri_r',...
        'rect_fem_r','vas_med_r','vas_int_r','vas_lat_r','med_gas_r',...
        'lat_gas_r','soleus_r','tib_post_r','flex_dig_r','flex_hal_r',...
        'tib_ant_r','per_brev_r','per_long_r','per_tert_r','ext_dig_r',...
        'ext_hal_r','ercspn_r','intobl_r','extobl_r','ercspn_l','intobl_l','extobl_l'};
    MuscleData.muscle_names = muscleNames;
    for m = 1:length(muscleNames)
        % Muscle-tendon length
        MuscleData.lMT(:,m)    = lMT.data(:,strcmp(lMT.colheaders,muscleNames{m}));
        
        % Moment Arm
        MuscleData.dM(:,m,1)   = MA.hip.flex.data(:,strcmp(lMT.colheaders,muscleNames{m}));
        MuscleData.dM(:,m,2)   = MA.hip.add.data(:,strcmp(lMT.colheaders,muscleNames{m}));
        MuscleData.dM(:,m,3)   = MA.hip.rot.data(:,strcmp(lMT.colheaders,muscleNames{m}));
        MuscleData.dM(:,m,4)   = MA.knee.flex.data(:,strcmp(lMT.colheaders,muscleNames{m}));
        MuscleData.dM(:,m,5)   = MA.ankle.flex.data(:,strcmp(lMT.colheaders,muscleNames{m}));
        MuscleData.dM(:,m,6)   = MA.sub.data(:,strcmp(lMT.colheaders,muscleNames{m}));
        MuscleData.dM(:,m,7)   = MA.mtp.flex.data(:,strcmp(lMT.colheaders,muscleNames{m}));
        MuscleData.dM(:,m,8)   = MA.trunk.ext.data(:,strcmp(lMT.colheaders,muscleNames{m}));
        MuscleData.dM(:,m,9)   = MA.trunk.ben.data(:,strcmp(lMT.colheaders,muscleNames{m}));
        MuscleData.dM(:,m,10)  = MA.trunk.rot.data(:,strcmp(lMT.colheaders,muscleNames{m}));
    end
    MuscleData.q = q;
    MuscleData.qdot = qdot;
end

%% Call PolynomialFit
if runPolynomialfit == 1
    [muscle_spanning_joint_INFO,MuscleInfo] = ...
        PolynomialFit(MuscleData);
    if savePolynomials == 1
        save MuscleData_subject9 MuscleData
        save muscle_spanning_joint_INFO_subject9 muscle_spanning_joint_INFO
        save MuscleInfo_subject9 MuscleInfo
    end
end

%% Create CasADi functions
import casadi.*
% Order:
% hip_flex_r,hip_add_r,hip_rot_r,knee_r,ankle_r,subtalar_r,mtp_r,lumbar_ext,lumbar_bend_lumbar_rot
load muscle_spanning_joint_INFO_subject9.mat
load MuscleInfo_subject9.mat
NMuscle = length(MuscleInfo.muscle);
q_leg_trunk = 10;
qin     = SX.sym('qin',1,q_leg_trunk);
qdotin  = SX.sym('qdotin',1,q_leg_trunk);
lMT     = SX(NMuscle,1);
vMT     = SX(NMuscle,1);
dM      = SX(NMuscle,q_leg_trunk);
for i=1:NMuscle     
    index_dof_crossing  = find(muscle_spanning_joint_INFO(i,:)==1);
    order               = MuscleInfo.muscle(i).order;
    [mat,diff_mat_q]    = n_art_mat_3_cas_SX(qin(1,index_dof_crossing),order);
    lMT(i,1)            = mat*MuscleInfo.muscle(i).coeff;
    vMT(i,1)            = 0;
    dM(i,1:q_leg_trunk) = 0;
    nr_dof_crossing     = length(index_dof_crossing); 
    for dof_nr = 1:nr_dof_crossing
        dM(i,index_dof_crossing(dof_nr)) = (-(diff_mat_q(:,dof_nr)))'*MuscleInfo.muscle(i).coeff;
        vMT(i,1) = vMT(i,1) + (-dM(i,index_dof_crossing(dof_nr))*qdotin(1,index_dof_crossing(dof_nr)));
    end 
end
f_lMT_vMT_dM = Function('f_lMT_vMT_dM',{qin,qdotin},{lMT,vMT,dM});

%% Check results
load MuscleData_subject9.mat
lMT_out_r = zeros(size(q,1),NMuscle);
vMT_out_r = zeros(size(q,1),NMuscle);
dM_out_r = zeros(size(q,1),NMuscle,q_leg_trunk);
for i = 1:size(q,1)
    [out1_r,out2_r,out3_r] = f_lMT_vMT_dM(MuscleData.q(i,:),MuscleData.qdot(i,:));
    lMT_out_r(i,:) = full(out1_r);
    vMT_out_r(i,:) = full(out2_r);
    dM_out_r(i,:,1) = full(out3_r(:,1));
    dM_out_r(i,:,2) = full(out3_r(:,2));
    dM_out_r(i,:,3) = full(out3_r(:,3));
    dM_out_r(i,:,4) = full(out3_r(:,4));
    dM_out_r(i,:,5) = full(out3_r(:,5));   
    dM_out_r(i,:,6) = full(out3_r(:,6));
    dM_out_r(i,:,7) = full(out3_r(:,7));
    dM_out_r(i,:,8) = full(out3_r(:,8));
    dM_out_r(i,:,9) = full(out3_r(:,9)); 
    dM_out_r(i,:,10) = full(out3_r(:,10)); 
end

% Testing/Inspecting polynomials for knee flexion behaviour
test_q = zeros(400,10);
test_qdot = zeros(400,10);
test_q(:,4) = flip(linspace(-2.55,0,400)');

lMT = zeros(size(test_q,1),NMuscle);
vMT = zeros(size(test_q,1),NMuscle);
dM = zeros(size(test_q,1),NMuscle);
for i = 1:size(test_q,1)
   [out1,out2,out3] = f_lMT_vMT_dM(test_q(i,:),test_qdot(i,:));
   lMT(i,:) = full(out1);
   vMT(i,:) = full(out2);
   dM(i,:)  = full(out3(:,4))';
end

test_q(:,5) = flip(linspace(-1.57,0,400)');

for i = 1:size(test_q,1)
   [out1,out2,out3] = f_lMT_vMT_dM(test_q(i,:),test_qdot(i,:));
   lMT(i,:) = full(out1);
   vMT(i,:) = full(out2);
   dM(i,:)  = full(out3(:,4))';
end

test_q(:,5) = (linspace(0,1.57,400)');

for i = 1:size(test_q,1)
   [out1,out2,out3] = f_lMT_vMT_dM(test_q(i,:),test_qdot(i,:));
   lMT(i,:) = full(out1);
   vMT(i,:) = full(out2);
   dM(i,:)  = full(out3(:,4))';
end

save inspect_lMT lMT
save inspect_vMT vMT
save inspect_dM dM

%% lMT Plotting
% right
% for m = 1:NMuscle
%     for coord = 1:q_leg_trunk
%         if muscle_spanning_joint_INFO(m,coord) == 1
%             if m == 29
%             scatter(MuscleData.q(:,coord),lMT_out_r(:,m));
%             hold on
%             scatter(MuscleData.q(:,coord),MuscleData.lMT(:,m));
%             title(MuscleData.muscle_names(m));
%             legend('Polynomial','Model');
%             suptitle('lMT right');
%             hold off
%             end
%         end
%     end
% end
figure()
subplot(4,4,1)
scatter(MuscleData.q(:,4),lMT_out_r(:,10)); hold on;
scatter(MuscleData.q(:,4),MuscleData.lMT(:,10));
xlabel('q knee');
title('BFSH');
subplot(4,4,2)
scatter(MuscleData.q(:,4),lMT_out_r(:,29)); hold on;
scatter(MuscleData.q(:,4),MuscleData.lMT(:,29));
xlabel('q knee');
title('VM');
subplot(4,4,3)
scatter(MuscleData.q(:,4),lMT_out_r(:,30)); hold on;
scatter(MuscleData.q(:,4),MuscleData.lMT(:,30));
xlabel('q knee');
title('VI');
subplot(4,4,4)
scatter(MuscleData.q(:,4),lMT_out_r(:,31)); hold on;
scatter(MuscleData.q(:,4),MuscleData.lMT(:,31));
xlabel('q knee');
title('VL');
subplot(4,4,5)
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),lMT_out_r(:,34)); hold on;
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),MuscleData.lMT(:,34));
xlabel('q knee');
ylabel('q ankle');
title('GM');
subplot(4,4,6)
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),lMT_out_r(:,35)); hold on;
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),MuscleData.lMT(:,35));
xlabel('q knee');
ylabel('q ankle');
title('GL');
subplot(4,4,7)
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),lMT_out_r(:,36)); hold on;
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),MuscleData.lMT(:,36));
xlabel('q knee');
ylabel('q ankle');
title('GM');
subplot(4,4,8)
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),lMT_out_r(:,37)); hold on;
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),MuscleData.lMT(:,37));
xlabel('q knee');
ylabel('q ankle');
title('GL');
subplot(4,4,9)
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),lMT_out_r(:,38)); hold on;
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),MuscleData.lMT(:,38));
xlabel('q knee');
ylabel('q ankle');
title('GM');
subplot(4,4,10)
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),lMT_out_r(:,39)); hold on;
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),MuscleData.lMT(:,39));
xlabel('q knee');
ylabel('q ankle');
title('GL');
subplot(4,4,11)
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),lMT_out_r(:,40)); hold on;
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),MuscleData.lMT(:,40));
xlabel('q knee');
ylabel('q ankle');
title('GM');
subplot(4,4,12)
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),lMT_out_r(:,41)); hold on;
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),MuscleData.lMT(:,41));
xlabel('q knee');
ylabel('q ankle');
title('GL');
subplot(4,4,13)
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),lMT_out_r(:,42)); hold on;
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),MuscleData.lMT(:,42));
xlabel('q knee');
ylabel('q ankle');
title('GM');
subplot(4,4,14)
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),lMT_out_r(:,43)); hold on;
scatter3(MuscleData.q(:,5),MuscleData.q(:,6),MuscleData.lMT(:,43));
xlabel('q knee');
ylabel('q ankle');
title('GL');
legend('Polynomial','Model');
suptitle('lMT right');

%% Assert results
for i = 1:NMuscle  
    assertLMT(:,i) = abs(lMT_out_r(:,i) - MuscleData.lMT(:,i));
    assertdM.hip.flex(:,i) = abs(dM_out_r(:,i,1) - MuscleData.dM(:,i,1));
    assertdM.hip.add(:,i) = abs(dM_out_r(:,i,2) - MuscleData.dM(:,i,2));
    assertdM.hip.rot(:,i) = abs(dM_out_r(:,i,3) - MuscleData.dM(:,i,3));
    assertdM.knee(:,i) = abs(dM_out_r(:,i,4) - MuscleData.dM(:,i,4));
    assertdM.ankle(:,i) = abs(dM_out_r(:,i,5) - MuscleData.dM(:,i,5));
    assertdM.sub(:,i) = abs(dM_out_r(:,i,6) - MuscleData.dM(:,i,6));
    assertdM.mtp(:,i) = abs(dM_out_r(:,i,7) - MuscleData.dM(:,i,7));
    assertdM.lumb.ext(:,i) = abs(dM_out_r(:,i,8) - MuscleData.dM(:,i,8));
    assertdM.lumb.bend(:,i) = abs(dM_out_r(:,i,9) - MuscleData.dM(:,i,9));
    assertdM.lumb.rot(:,i) = abs(dM_out_r(:,i,10) - MuscleData.dM(:,i,10));
end

assertLMTmax_r = max(max(assertLMT));
assertdM.hip.flexmax = max(max(assertdM.hip.flex));
assertdM.hip.addmax = max(max(assertdM.hip.add));
assertdM.hip.rotmax = max(max(assertdM.hip.rot));
assertdM.kneemax = max(max(assertdM.knee));
assertdM.anklemax = max(max(assertdM.ankle));
assertdM.mtpmax = max(max(assertdM.mtp));
assertdM.lumb.extmax = max(max(assertdM.lumb.ext));
assertdM.lumb.bendmax = max(max(assertdM.lumb.bend));
assertdM.lumb.rotmax = max(max(assertdM.lumb.rot));