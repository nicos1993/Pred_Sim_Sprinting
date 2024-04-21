% Script that performs necessary calculations on simulation results

clear; clc;
close all

pred_15 = load('IC_pred_Sprinting_optimum_p02_maxVel_01_26-October-2023__10-52-28___Nominal_CONS_CHECK.mat');

pred_15_006_td_p = load('IC_pred_Sprinting_optimum_p02_maxVel_01_28-October-2023__10-02-52___HTD_Plus_6_CHECK2_1.mat');
pred_15_004_td_p = load('IC_pred_Sprinting_optimum_p02_maxVel_01_26-October-2023__22-16-25___HTD_Plus_4.mat');
pred_15_002_td_p = load('IC_pred_Sprinting_optimum_p02_maxVel_01_27-October-2023__18-13-56___HTD_Plus_2_CHECK2_2.mat');
pred_15_002_td_m = load('IC_pred_Sprinting_optimum_p02_maxVel_01_28-October-2023__01-01-40___HTD_Minus_2.mat');
pred_15_004_td_m = load('IC_pred_Sprinting_optimum_p02_maxVel_01_28-October-2023__10-08-04___HTD_Minus_4_CHECK2_1.mat'); 
pred_15_006_td_m = load('IC_pred_Sprinting_optimum_p02_maxVel_01_28-October-2023__09-53-48___HTD_Minus_6_CHECK2_1.mat');

pred_15_006_ik_p = load('IC_pred_Sprinting_optimum_p02_maxVel_01_26-October-2023__20-13-02___IKTD_Plus_6.mat');
pred_15_004_ik_p = load('IC_pred_Sprinting_optimum_p02_maxVel_01_28-October-2023__16-43-06___IKTD_Plus_4_CHECK2_1.mat');
pred_15_002_ik_p = load('IC_pred_Sprinting_optimum_p02_maxVel_01_27-October-2023__11-17-52___IKTD_Plus_2.mat');
pred_15_006_ik_m = load('IC_pred_Sprinting_optimum_p02_maxVel_01_28-October-2023__13-58-45___IKTD_Minus_6_CHECK2_1.mat');
pred_15_004_ik_m = load('IC_pred_Sprinting_optimum_p02_maxVel_01_26-October-2023__23-37-47___IKTD_Minus_4.mat');
pred_15_002_ik_m = load('IC_pred_Sprinting_optimum_p02_maxVel_01_27-October-2023__11-55-38___IKTD_Minus_2.mat');



% Modified qdot's for Power calculations
pred_15_qdot = pred_15.optimumOutput.optVars_nsc.qdot(:,mod(1:length(pred_15.optimumOutput.optVars_nsc.qdot(:,:)),4)~=0);
pred_15_qdot(:,end+1) = pred_15.optimumOutput.optVars_nsc.qdot(:,end);

pred_15_q = pred_15.optimumOutput.optVars_nsc.q(:,mod(1:length(pred_15.optimumOutput.optVars_nsc.q(:,:)),4)~=0);
pred_15_q(:,end+1) = pred_15.optimumOutput.optVars_nsc.q(:,end);

% Joint Powers
pred_15_power = pred_15_qdot.*pred_15.optimumOutput.muscleMoments.joints;

% Determine contact times
pred_15_to  = find(pred_15.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;

pred_15_006_td_p_to = find(pred_15_006_td_p.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_15_004_td_p_to = find(pred_15_004_td_p.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_15_002_td_p_to = find(pred_15_002_td_p.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_15_006_td_m_to = find(pred_15_006_td_m.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_15_004_td_m_to = find(pred_15_004_td_m.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_15_002_td_m_to = find(pred_15_002_td_m.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;

pred_15_006_ik_p_to = find(pred_15_006_ik_p.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_15_004_ik_p_to = find(pred_15_004_ik_p.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_15_002_ik_p_to = find(pred_15_002_ik_p.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_15_006_ik_m_to = find(pred_15_006_ik_m.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_15_004_ik_m_to = find(pred_15_004_ik_m.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_15_002_ik_m_to = find(pred_15_002_ik_m.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;

pred_15_to_i = pred_15.optimumOutput.timeNodes(pred_15_to);

pred_15_006_td_p_to_i = pred_15_006_td_p.optimumOutput.timeNodes(pred_15_006_td_p_to);
pred_15_004_td_p_to_i = pred_15_004_td_p.optimumOutput.timeNodes(pred_15_004_td_p_to);
pred_15_002_td_p_to_i = pred_15_002_td_p.optimumOutput.timeNodes(pred_15_002_td_p_to);
pred_15_006_td_m_to_i = pred_15_006_td_m.optimumOutput.timeNodes(pred_15_006_td_m_to);
pred_15_004_td_m_to_i = pred_15_004_td_m.optimumOutput.timeNodes(pred_15_004_td_m_to);
pred_15_002_td_m_to_i = pred_15_002_td_m.optimumOutput.timeNodes(pred_15_002_td_m_to);

pred_15_006_ik_p_to_i = pred_15_006_ik_p.optimumOutput.timeNodes(pred_15_006_ik_p_to);
pred_15_004_ik_p_to_i = pred_15_004_ik_p.optimumOutput.timeNodes(pred_15_004_ik_p_to);
pred_15_002_ik_p_to_i = pred_15_002_ik_p.optimumOutput.timeNodes(pred_15_002_ik_p_to);
pred_15_006_ik_m_to_i = pred_15_006_ik_m.optimumOutput.timeNodes(pred_15_006_ik_m_to);
pred_15_004_ik_m_to_i = pred_15_004_ik_m.optimumOutput.timeNodes(pred_15_004_ik_m_to);
pred_15_002_ik_m_to_i = pred_15_002_ik_m.optimumOutput.timeNodes(pred_15_002_ik_m_to);

% Determine the take-off frame in kinematics timeline
pred_15_to_i_kin = find(pred_15.optimumOutput.timeGrid == pred_15_to_i);

pred_15_006_td_p_to_i_kin = find(pred_15_006_td_p.optimumOutput.timeGrid == pred_15_006_td_p_to_i);

% Average hip extension angular velocity during stance
pred_15_hip_vel = mean(pred_15.optimumOutput.optVars_nsc.qdot(7,1:pred_15_to_i_kin));

pred_15_006_td_p_hip_vel = mean(pred_15_006_td_p.optimumOutput.optVars_nsc.qdot(7,1:pred_15_006_td_p_to_i_kin));


pred_15_ct = pred_15.optimumOutput.timeNodes(pred_15_to) - pred_15.optimumOutput.timeNodes(1);

pred_15_006_td_p_ct = pred_15_006_td_p.optimumOutput.timeNodes(pred_15_006_td_p_to) - pred_15_006_td_p.optimumOutput.timeNodes(1);
pred_15_004_td_p_ct = pred_15_004_td_p.optimumOutput.timeNodes(pred_15_004_td_p_to) - pred_15_004_td_p.optimumOutput.timeNodes(1);
pred_15_002_td_p_ct = pred_15_002_td_p.optimumOutput.timeNodes(pred_15_002_td_p_to) - pred_15_002_td_p.optimumOutput.timeNodes(1);
pred_15_006_td_m_ct = pred_15_006_td_m.optimumOutput.timeNodes(pred_15_006_td_m_to) - pred_15_006_td_m.optimumOutput.timeNodes(1);
pred_15_004_td_m_ct = pred_15_004_td_m.optimumOutput.timeNodes(pred_15_004_td_m_to) - pred_15_004_td_m.optimumOutput.timeNodes(1);
pred_15_002_td_m_ct = pred_15_002_td_m.optimumOutput.timeNodes(pred_15_002_td_m_to) - pred_15_002_td_m.optimumOutput.timeNodes(1);

pred_15_006_ik_p_ct = pred_15_006_ik_p.optimumOutput.timeNodes(pred_15_006_ik_p_to) - pred_15_006_ik_p.optimumOutput.timeNodes(1);
pred_15_004_ik_p_ct = pred_15_004_ik_p.optimumOutput.timeNodes(pred_15_004_ik_p_to) - pred_15_004_ik_p.optimumOutput.timeNodes(1);
pred_15_002_ik_p_ct = pred_15_002_ik_p.optimumOutput.timeNodes(pred_15_002_ik_p_to) - pred_15_002_ik_p.optimumOutput.timeNodes(1);
pred_15_006_ik_m_ct = pred_15_006_ik_m.optimumOutput.timeNodes(pred_15_006_ik_m_to) - pred_15_006_ik_m.optimumOutput.timeNodes(1);
pred_15_004_ik_m_ct = pred_15_004_ik_m.optimumOutput.timeNodes(pred_15_004_ik_m_to) - pred_15_004_ik_m.optimumOutput.timeNodes(1);
pred_15_002_ik_m_ct = pred_15_002_ik_m.optimumOutput.timeNodes(pred_15_002_ik_m_to) - pred_15_002_ik_m.optimumOutput.timeNodes(1);




% Step duration
pred_15_sd = pred_15.optimumOutput.optVars_nsc.totalTime;

pred_15_006_td_p_sd = pred_15_006_td_p.optimumOutput.optVars_nsc.totalTime;
pred_15_004_td_p_sd = pred_15_004_td_p.optimumOutput.optVars_nsc.totalTime;
pred_15_002_td_p_sd = pred_15_002_td_p.optimumOutput.optVars_nsc.totalTime;
pred_15_006_td_m_sd = pred_15_006_td_m.optimumOutput.optVars_nsc.totalTime;
pred_15_004_td_m_sd = pred_15_004_td_m.optimumOutput.optVars_nsc.totalTime;
pred_15_002_td_m_sd = pred_15_002_td_m.optimumOutput.optVars_nsc.totalTime;

pred_15_006_ik_p_sd = pred_15_006_ik_p.optimumOutput.optVars_nsc.totalTime;
pred_15_004_ik_p_sd = pred_15_004_ik_p.optimumOutput.optVars_nsc.totalTime;
pred_15_002_ik_p_sd = pred_15_002_ik_p.optimumOutput.optVars_nsc.totalTime;
pred_15_006_ik_m_sd = pred_15_006_ik_m.optimumOutput.optVars_nsc.totalTime;
pred_15_004_ik_m_sd = pred_15_004_ik_m.optimumOutput.optVars_nsc.totalTime;
pred_15_002_ik_m_sd = pred_15_002_ik_m.optimumOutput.optVars_nsc.totalTime;


% Step frequency
pred_15_sf = 1/pred_15_sd;

pred_15_006_td_p_sf = 1/pred_15_006_td_p_sd;
pred_15_004_td_p_sf = 1/pred_15_004_td_p_sd;
pred_15_002_td_p_sf = 1/pred_15_002_td_p_sd;
pred_15_006_td_m_sf = 1/pred_15_006_td_m_sd;
pred_15_004_td_m_sf = 1/pred_15_004_td_m_sd;
pred_15_002_td_m_sf = 1/pred_15_002_td_m_sd;

pred_15_006_ik_p_sf = 1/pred_15_006_ik_p_sd;
pred_15_004_ik_p_sf = 1/pred_15_004_ik_p_sd;
pred_15_002_ik_p_sf = 1/pred_15_002_ik_p_sd;
pred_15_006_ik_m_sf = 1/pred_15_006_ik_m_sd;
pred_15_004_ik_m_sf = 1/pred_15_004_ik_m_sd;
pred_15_002_ik_m_sf = 1/pred_15_002_ik_m_sd;


% Swing time
pred_15_st = (2*(pred_15_sd - pred_15_ct)) + pred_15_ct;

pred_15_006_td_p_st = (2*(pred_15_006_td_p_sd - pred_15_006_td_p_ct)) + pred_15_006_td_p_ct;
pred_15_004_td_p_st = (2*(pred_15_004_td_p_sd - pred_15_004_td_p_ct)) + pred_15_004_td_p_ct;
pred_15_002_td_p_st = (2*(pred_15_002_td_p_sd - pred_15_002_td_p_ct)) + pred_15_002_td_p_ct;
pred_15_006_td_m_st = (2*(pred_15_006_td_m_sd - pred_15_006_td_m_ct)) + pred_15_006_td_m_ct;
pred_15_004_td_m_st = (2*(pred_15_004_td_m_sd - pred_15_004_td_m_ct)) + pred_15_004_td_m_ct;
pred_15_002_td_m_st = (2*(pred_15_002_td_m_sd - pred_15_002_td_m_ct)) + pred_15_002_td_m_ct;

pred_15_006_ik_p_st = (2*(pred_15_006_ik_p_sd - pred_15_006_ik_p_ct)) + pred_15_006_ik_p_ct;
pred_15_004_ik_p_st = (2*(pred_15_004_ik_p_sd - pred_15_004_ik_p_ct)) + pred_15_004_ik_p_ct;
pred_15_002_ik_p_st = (2*(pred_15_002_ik_p_sd - pred_15_002_ik_p_ct)) + pred_15_002_ik_p_ct;
pred_15_006_ik_m_st = (2*(pred_15_006_ik_m_sd - pred_15_006_ik_m_ct)) + pred_15_006_ik_m_ct;
pred_15_004_ik_m_st = (2*(pred_15_004_ik_m_sd - pred_15_004_ik_m_ct)) + pred_15_004_ik_m_ct;
pred_15_002_ik_m_st = (2*(pred_15_002_ik_m_sd - pred_15_002_ik_m_ct)) + pred_15_002_ik_m_ct;


% Aerial time
pred_15_at = pred_15_sd - pred_15_ct;

pred_15_006_td_p_at = pred_15_006_td_p_sd - pred_15_006_td_p_ct;
pred_15_004_td_p_at = pred_15_004_td_p_sd - pred_15_004_td_p_ct;
pred_15_002_td_p_at = pred_15_002_td_p_sd - pred_15_002_td_p_ct;
pred_15_006_td_m_at = pred_15_006_td_m_sd - pred_15_006_td_m_ct;
pred_15_004_td_m_at = pred_15_004_td_m_sd - pred_15_004_td_m_ct;
pred_15_002_td_m_at = pred_15_002_td_m_sd - pred_15_002_td_m_ct;

pred_15_006_ik_p_at = pred_15_006_ik_p_sd - pred_15_006_ik_p_ct;
pred_15_004_ik_p_at = pred_15_004_ik_p_sd - pred_15_004_ik_p_ct;
pred_15_002_ik_p_at = pred_15_002_ik_p_sd - pred_15_002_ik_p_ct;
pred_15_006_ik_m_at = pred_15_006_ik_m_sd - pred_15_006_ik_m_ct;
pred_15_004_ik_m_at = pred_15_004_ik_m_sd - pred_15_004_ik_m_ct;
pred_15_002_ik_m_at = pred_15_002_ik_m_sd - pred_15_002_ik_m_ct;


% Vertical COM velocity at touchdown
pred_15_COM_v_td = pred_15.optimumOutput.modelCOM.vel(2,1);

pred_15_006_td_p_COM_v_td = pred_15_006_td_p.optimumOutput.modelCOM.vel(2,1);
pred_15_004_td_p_COM_v_td = pred_15_004_td_p.optimumOutput.modelCOM.vel(2,1);
pred_15_002_td_p_COM_v_td = pred_15_002_td_p.optimumOutput.modelCOM.vel(2,1);
pred_15_006_td_m_COM_v_td = pred_15_006_td_m.optimumOutput.modelCOM.vel(2,1);
pred_15_004_td_m_COM_v_td = pred_15_004_td_m.optimumOutput.modelCOM.vel(2,1);
pred_15_002_td_m_COM_v_td = pred_15_002_td_m.optimumOutput.modelCOM.vel(2,1);

pred_15_006_ik_p_COM_v_td = pred_15_006_ik_p.optimumOutput.modelCOM.vel(2,1);
pred_15_004_ik_p_COM_v_td = pred_15_004_ik_p.optimumOutput.modelCOM.vel(2,1);
pred_15_002_ik_p_COM_v_td = pred_15_002_ik_p.optimumOutput.modelCOM.vel(2,1);
pred_15_006_ik_m_COM_v_td = pred_15_006_ik_m.optimumOutput.modelCOM.vel(2,1);
pred_15_004_ik_m_COM_v_td = pred_15_004_ik_m.optimumOutput.modelCOM.vel(2,1);
pred_15_002_ik_m_COM_v_td = pred_15_002_ik_m.optimumOutput.modelCOM.vel(2,1);

% Vertical COM velocity at takeoff
pred_15_COM_v_to = pred_15.optimumOutput.modelCOM.vel(2,pred_15_to);

pred_15_006_td_p_COM_v_to = pred_15_006_td_p.optimumOutput.modelCOM.vel(2,pred_15_006_td_p_to);
pred_15_004_td_p_COM_v_to = pred_15_004_td_p.optimumOutput.modelCOM.vel(2,pred_15_004_td_p_to);
pred_15_002_td_p_COM_v_to = pred_15_002_td_p.optimumOutput.modelCOM.vel(2,pred_15_002_td_p_to);
pred_15_006_td_m_COM_v_to = pred_15_006_td_m.optimumOutput.modelCOM.vel(2,pred_15_006_td_m_to);
pred_15_004_td_m_COM_v_to = pred_15_004_td_m.optimumOutput.modelCOM.vel(2,pred_15_004_td_m_to);
pred_15_002_td_m_COM_v_to = pred_15_002_td_m.optimumOutput.modelCOM.vel(2,pred_15_002_td_m_to);

pred_15_006_ik_p_COM_v_to = pred_15_006_ik_p.optimumOutput.modelCOM.vel(2,pred_15_006_ik_p_to);
pred_15_004_ik_p_COM_v_to = pred_15_004_ik_p.optimumOutput.modelCOM.vel(2,pred_15_004_ik_p_to);
pred_15_002_ik_p_COM_v_to = pred_15_002_ik_p.optimumOutput.modelCOM.vel(2,pred_15_002_ik_p_to);
pred_15_006_ik_m_COM_v_to = pred_15_006_ik_m.optimumOutput.modelCOM.vel(2,pred_15_006_ik_m_to);
pred_15_004_ik_m_COM_v_to = pred_15_004_ik_m.optimumOutput.modelCOM.vel(2,pred_15_004_ik_m_to);
pred_15_002_ik_m_COM_v_to = pred_15_002_ik_m.optimumOutput.modelCOM.vel(2,pred_15_002_ik_m_to);

% Horizontal COM velocity at touchdown
pred_15_COM_h_td = pred_15.optimumOutput.modelCOM.vel(1,1);

pred_15_006_td_p_COM_h_td = pred_15_006_td_p.optimumOutput.modelCOM.vel(1,1);
pred_15_004_td_p_COM_h_td = pred_15_004_td_p.optimumOutput.modelCOM.vel(1,1);
pred_15_002_td_p_COM_h_td = pred_15_002_td_p.optimumOutput.modelCOM.vel(1,1);
pred_15_006_td_m_COM_h_td = pred_15_006_td_m.optimumOutput.modelCOM.vel(1,1);
pred_15_004_td_m_COM_h_td = pred_15_004_td_m.optimumOutput.modelCOM.vel(1,1);
pred_15_002_td_m_COM_h_td = pred_15_002_td_m.optimumOutput.modelCOM.vel(1,1);

pred_15_006_ik_p_COM_h_td = pred_15_006_ik_p.optimumOutput.modelCOM.vel(1,1);
pred_15_004_ik_p_COM_h_td = pred_15_004_ik_p.optimumOutput.modelCOM.vel(1,1);
pred_15_002_ik_p_COM_h_td = pred_15_002_ik_p.optimumOutput.modelCOM.vel(1,1);
pred_15_006_ik_m_COM_h_td = pred_15_006_ik_m.optimumOutput.modelCOM.vel(1,1);
pred_15_004_ik_m_COM_h_td = pred_15_004_ik_m.optimumOutput.modelCOM.vel(1,1);
pred_15_002_ik_m_COM_h_td = pred_15_002_ik_m.optimumOutput.modelCOM.vel(1,1);

% Horizontal COM velocity at takeoff
pred_15_COM_h_to = pred_15.optimumOutput.modelCOM.vel(1,pred_15_to);

pred_15_006_td_p_COM_h_to = pred_15_006_td_p.optimumOutput.modelCOM.vel(1,pred_15_006_td_p_to);
pred_15_004_td_p_COM_h_to = pred_15_004_td_p.optimumOutput.modelCOM.vel(1,pred_15_004_td_p_to);
pred_15_002_td_p_COM_h_to = pred_15_002_td_p.optimumOutput.modelCOM.vel(1,pred_15_002_td_p_to);
pred_15_006_td_m_COM_h_to = pred_15_006_td_m.optimumOutput.modelCOM.vel(1,pred_15_006_td_m_to);
pred_15_004_td_m_COM_h_to = pred_15_004_td_m.optimumOutput.modelCOM.vel(1,pred_15_004_td_m_to);
pred_15_002_td_m_COM_h_to = pred_15_002_td_m.optimumOutput.modelCOM.vel(1,pred_15_002_td_m_to);

pred_15_006_ik_p_COM_h_to = pred_15_006_ik_p.optimumOutput.modelCOM.vel(1,pred_15_006_ik_p_to);
pred_15_004_ik_p_COM_h_to = pred_15_004_ik_p.optimumOutput.modelCOM.vel(1,pred_15_004_ik_p_to);
pred_15_002_ik_p_COM_h_to = pred_15_002_ik_p.optimumOutput.modelCOM.vel(1,pred_15_002_ik_p_to);
pred_15_006_ik_m_COM_h_to = pred_15_006_ik_m.optimumOutput.modelCOM.vel(1,pred_15_006_ik_m_to);
pred_15_004_ik_m_COM_h_to = pred_15_004_ik_m.optimumOutput.modelCOM.vel(1,pred_15_004_ik_m_to);
pred_15_002_ik_m_COM_h_to = pred_15_002_ik_m.optimumOutput.modelCOM.vel(1,pred_15_002_ik_m_to);

% Step length
pred_15_sl = pred_15.optimumOutput.optVars_nsc.q(4,end);

pred_15_006_td_p_sl = pred_15_006_td_p.optimumOutput.optVars_nsc.q(4,end);
pred_15_004_td_p_sl = pred_15_004_td_p.optimumOutput.optVars_nsc.q(4,end);
pred_15_002_td_p_sl = pred_15_002_td_p.optimumOutput.optVars_nsc.q(4,end);
pred_15_006_td_m_sl = pred_15_006_td_m.optimumOutput.optVars_nsc.q(4,end);
pred_15_004_td_m_sl = pred_15_004_td_m.optimumOutput.optVars_nsc.q(4,end);
pred_15_002_td_m_sl = pred_15_002_td_m.optimumOutput.optVars_nsc.q(4,end);

pred_15_006_ik_p_sl = pred_15_006_ik_p.optimumOutput.optVars_nsc.q(4,end);
pred_15_004_ik_p_sl = pred_15_004_ik_p.optimumOutput.optVars_nsc.q(4,end);
pred_15_002_ik_p_sl = pred_15_002_ik_p.optimumOutput.optVars_nsc.q(4,end);
pred_15_006_ik_m_sl = pred_15_006_ik_m.optimumOutput.optVars_nsc.q(4,end);
pred_15_004_ik_m_sl = pred_15_004_ik_m.optimumOutput.optVars_nsc.q(4,end);
pred_15_002_ik_m_sl = pred_15_002_ik_m.optimumOutput.optVars_nsc.q(4,end);

% Speed
pred_15_spe = pred_15_sf*pred_15_sl;

pred_15_006_td_p_spe = pred_15_006_td_p_sf*pred_15_006_td_p_sl;
pred_15_004_td_p_spe = pred_15_004_td_p_sf*pred_15_004_td_p_sl;
pred_15_002_td_p_spe = pred_15_002_td_p_sf*pred_15_002_td_p_sl;
pred_15_006_td_m_spe = pred_15_006_td_m_sf*pred_15_006_td_m_sl;
pred_15_004_td_m_spe = pred_15_004_td_m_sf*pred_15_004_td_m_sl;
pred_15_002_td_m_spe = pred_15_002_td_m_sf*pred_15_002_td_m_sl;

pred_15_006_ik_p_spe = pred_15_006_ik_p_sf*pred_15_006_ik_p_sl;
pred_15_004_ik_p_spe = pred_15_004_ik_p_sf*pred_15_004_ik_p_sl;
pred_15_002_ik_p_spe = pred_15_002_ik_p_sf*pred_15_002_ik_p_sl;
pred_15_006_ik_m_spe = pred_15_006_ik_m_sf*pred_15_006_ik_m_sl;
pred_15_004_ik_m_spe = pred_15_004_ik_m_sf*pred_15_004_ik_m_sl;
pred_15_002_ik_m_spe = pred_15_002_ik_m_sf*pred_15_002_ik_m_sl;


% Calculate stance joint work

% Stance Ankle
pred_15_aw = trapz(pred_15.optimumOutput.timeNodes(1:pred_15_to),pred_15_power(11,1:pred_15_to));

% Stance Knee
pred_15_kw = trapz(pred_15.optimumOutput.timeNodes(1:pred_15_to),pred_15_power(10,1:pred_15_to));

% Stance Hip
pred_15_hw = trapz(pred_15.optimumOutput.timeNodes(1:pred_15_to),pred_15_power(7,1:pred_15_to));

% Calculate stance angular joint impulse

% Stance Ankle
[pred_15_aai,pred_15_aai_df,pred_15_aai_pf] = trapezoidalIntegration(pred_15.optimumOutput.timeNodes(1:pred_15_to),pred_15.optimumOutput.muscleMoments.joints(11,1:pred_15_to));

[pred_15_006_p_td_aai,pred_15_006_td_p_aai_df,pred_15_006_td_p_aai_pf] = trapezoidalIntegration(pred_15_006_td_p.optimumOutput.timeNodes(1:pred_15_006_td_p_to),pred_15_006_td_p.optimumOutput.muscleMoments.joints(11,1:pred_15_006_td_p_to));
[pred_15_004_p_td_aai,pred_15_004_td_p_aai_df,pred_15_004_td_p_aai_pf] = trapezoidalIntegration(pred_15_004_td_p.optimumOutput.timeNodes(1:pred_15_004_td_p_to),pred_15_004_td_p.optimumOutput.muscleMoments.joints(11,1:pred_15_004_td_p_to));
[pred_15_002_p_td_aai,pred_15_002_td_p_aai_df,pred_15_002_td_p_aai_pf] = trapezoidalIntegration(pred_15_002_td_p.optimumOutput.timeNodes(1:pred_15_002_td_p_to),pred_15_002_td_p.optimumOutput.muscleMoments.joints(11,1:pred_15_002_td_p_to));
[pred_15_006_m_td_aai,pred_15_006_td_m_aai_df,pred_15_006_td_m_aai_pf] = trapezoidalIntegration(pred_15_006_td_m.optimumOutput.timeNodes(1:pred_15_006_td_m_to),pred_15_006_td_m.optimumOutput.muscleMoments.joints(11,1:pred_15_006_td_m_to));
[pred_15_004_m_td_aai,pred_15_004_td_m_aai_df,pred_15_004_td_m_aai_pf] = trapezoidalIntegration(pred_15_004_td_m.optimumOutput.timeNodes(1:pred_15_004_td_m_to),pred_15_004_td_m.optimumOutput.muscleMoments.joints(11,1:pred_15_004_td_m_to));
[pred_15_002_m_td_aai,pred_15_002_td_m_aai_df,pred_15_002_td_m_aai_pf] = trapezoidalIntegration(pred_15_002_td_m.optimumOutput.timeNodes(1:pred_15_002_td_m_to),pred_15_002_td_m.optimumOutput.muscleMoments.joints(11,1:pred_15_002_td_m_to));

[pred_15_006_p_ik_aai,pred_15_006_ik_p_aai_df,pred_15_006_ik_p_aai_pf] = trapezoidalIntegration(pred_15_006_ik_p.optimumOutput.timeNodes(1:pred_15_006_ik_p_to),pred_15_006_ik_p.optimumOutput.muscleMoments.joints(11,1:pred_15_006_ik_p_to));
[pred_15_004_p_ik_aai,pred_15_004_ik_p_aai_df,pred_15_004_ik_p_aai_pf] = trapezoidalIntegration(pred_15_004_ik_p.optimumOutput.timeNodes(1:pred_15_004_ik_p_to),pred_15_004_ik_p.optimumOutput.muscleMoments.joints(11,1:pred_15_004_ik_p_to));
[pred_15_002_p_ik_aai,pred_15_002_ik_p_aai_df,pred_15_002_ik_p_aai_pf] = trapezoidalIntegration(pred_15_002_ik_p.optimumOutput.timeNodes(1:pred_15_002_ik_p_to),pred_15_002_ik_p.optimumOutput.muscleMoments.joints(11,1:pred_15_002_ik_p_to));
[pred_15_006_m_ik_aai,pred_15_006_ik_m_aai_df,pred_15_006_ik_m_aai_pf] = trapezoidalIntegration(pred_15_006_ik_m.optimumOutput.timeNodes(1:pred_15_006_ik_m_to),pred_15_006_ik_m.optimumOutput.muscleMoments.joints(11,1:pred_15_006_ik_m_to));
[pred_15_004_m_ik_aai,pred_15_004_ik_m_aai_df,pred_15_004_ik_m_aai_pf] = trapezoidalIntegration(pred_15_004_ik_m.optimumOutput.timeNodes(1:pred_15_004_ik_m_to),pred_15_004_ik_m.optimumOutput.muscleMoments.joints(11,1:pred_15_004_ik_m_to));
[pred_15_002_m_ik_aai,pred_15_002_ik_m_aai_df,pred_15_002_ik_m_aai_pf] = trapezoidalIntegration(pred_15_002_ik_m.optimumOutput.timeNodes(1:pred_15_002_ik_m_to),pred_15_002_ik_m.optimumOutput.muscleMoments.joints(11,1:pred_15_002_ik_m_to));


% Stance Knee
[pred_15_kai,pred_15_kai_ext,pred_15_kai_fle] = trapezoidalIntegration(pred_15.optimumOutput.timeNodes(1:pred_15_to),pred_15.optimumOutput.muscleMoments.joints(10,1:pred_15_to));

[pred_15_006_p_td_kai,pred_15_006_td_p_kai_ext,pred_15_006_td_p_kai_fle] = trapezoidalIntegration(pred_15_006_td_p.optimumOutput.timeNodes(1:pred_15_006_td_p_to),pred_15_006_td_p.optimumOutput.muscleMoments.joints(10,1:pred_15_006_td_p_to));
[pred_15_004_p_td_kai,pred_15_004_td_p_kai_ext,pred_15_004_td_p_kai_fle] = trapezoidalIntegration(pred_15_004_td_p.optimumOutput.timeNodes(1:pred_15_004_td_p_to),pred_15_004_td_p.optimumOutput.muscleMoments.joints(10,1:pred_15_004_td_p_to));
[pred_15_002_p_td_kai,pred_15_002_td_p_kai_ext,pred_15_002_td_p_kai_fle] = trapezoidalIntegration(pred_15_002_td_p.optimumOutput.timeNodes(1:pred_15_002_td_p_to),pred_15_002_td_p.optimumOutput.muscleMoments.joints(10,1:pred_15_002_td_p_to));
[pred_15_006_m_td_kai,pred_15_006_td_m_kai_ext,pred_15_006_td_m_kai_fle] = trapezoidalIntegration(pred_15_006_td_m.optimumOutput.timeNodes(1:pred_15_006_td_m_to),pred_15_006_td_m.optimumOutput.muscleMoments.joints(10,1:pred_15_006_td_m_to));
[pred_15_004_m_td_kai,pred_15_004_td_m_kai_ext,pred_15_004_td_m_kai_fle] = trapezoidalIntegration(pred_15_004_td_m.optimumOutput.timeNodes(1:pred_15_004_td_m_to),pred_15_004_td_m.optimumOutput.muscleMoments.joints(10,1:pred_15_004_td_m_to));
[pred_15_002_m_td_kai,pred_15_002_td_m_kai_ext,pred_15_002_td_m_kai_fle] = trapezoidalIntegration(pred_15_002_td_m.optimumOutput.timeNodes(1:pred_15_002_td_m_to),pred_15_002_td_m.optimumOutput.muscleMoments.joints(10,1:pred_15_002_td_m_to));

[pred_15_006_p_ik_kai,pred_15_006_ik_p_kai_ext,pred_15_006_ik_p_kai_fle] = trapezoidalIntegration(pred_15_006_ik_p.optimumOutput.timeNodes(1:pred_15_006_ik_p_to),pred_15_006_ik_p.optimumOutput.muscleMoments.joints(10,1:pred_15_006_ik_p_to));
[pred_15_004_p_ik_kai,pred_15_004_ik_p_kai_ext,pred_15_004_ik_p_kai_fle] = trapezoidalIntegration(pred_15_004_ik_p.optimumOutput.timeNodes(1:pred_15_004_ik_p_to),pred_15_004_ik_p.optimumOutput.muscleMoments.joints(10,1:pred_15_004_ik_p_to));
[pred_15_002_p_ik_kai,pred_15_002_ik_p_kai_ext,pred_15_002_ik_p_kai_fle] = trapezoidalIntegration(pred_15_002_ik_p.optimumOutput.timeNodes(1:pred_15_002_ik_p_to),pred_15_002_ik_p.optimumOutput.muscleMoments.joints(10,1:pred_15_002_ik_p_to));
[pred_15_006_m_ik_kai,pred_15_006_ik_m_kai_ext,pred_15_006_ik_m_kai_fle] = trapezoidalIntegration(pred_15_006_ik_m.optimumOutput.timeNodes(1:pred_15_006_ik_m_to),pred_15_006_ik_m.optimumOutput.muscleMoments.joints(10,1:pred_15_006_ik_m_to));
[pred_15_004_m_ik_kai,pred_15_004_ik_m_kai_ext,pred_15_004_ik_m_kai_fle] = trapezoidalIntegration(pred_15_004_ik_m.optimumOutput.timeNodes(1:pred_15_004_ik_m_to),pred_15_004_ik_m.optimumOutput.muscleMoments.joints(10,1:pred_15_004_ik_m_to));
[pred_15_002_m_ik_kai,pred_15_002_ik_m_kai_ext,pred_15_002_ik_m_kai_fle] = trapezoidalIntegration(pred_15_002_ik_m.optimumOutput.timeNodes(1:pred_15_002_ik_m_to),pred_15_002_ik_m.optimumOutput.muscleMoments.joints(10,1:pred_15_002_ik_m_to));


% Stance Hip
[pred_15_hai,pred_15_hai_fle,pred_15_hai_ext] = trapezoidalIntegration(pred_15.optimumOutput.timeNodes(1:pred_15_to),pred_15.optimumOutput.muscleMoments.joints(7,1:pred_15_to));

[pred_15_006_p_td_hai,pred_15_006_td_p_hai_fle,pred_15_006_td_p_hai_ext] = trapezoidalIntegration(pred_15_006_td_p.optimumOutput.timeNodes(1:pred_15_006_td_p_to),pred_15_006_td_p.optimumOutput.muscleMoments.joints(7,1:pred_15_006_td_p_to));
[pred_15_004_p_td_hai,pred_15_004_td_p_hai_fle,pred_15_004_td_p_hai_ext] = trapezoidalIntegration(pred_15_004_td_p.optimumOutput.timeNodes(1:pred_15_004_td_p_to),pred_15_004_td_p.optimumOutput.muscleMoments.joints(7,1:pred_15_004_td_p_to));
[pred_15_002_p_td_hai,pred_15_002_td_p_hai_fle,pred_15_002_td_p_hai_ext] = trapezoidalIntegration(pred_15_002_td_p.optimumOutput.timeNodes(1:pred_15_002_td_p_to),pred_15_002_td_p.optimumOutput.muscleMoments.joints(7,1:pred_15_002_td_p_to));
[pred_15_006_m_td_hai,pred_15_006_td_m_hai_fle,pred_15_006_td_m_hai_ext] = trapezoidalIntegration(pred_15_006_td_m.optimumOutput.timeNodes(1:pred_15_006_td_m_to),pred_15_006_td_m.optimumOutput.muscleMoments.joints(7,1:pred_15_006_td_m_to));
[pred_15_004_m_td_hai,pred_15_004_td_m_hai_fle,pred_15_004_td_m_hai_ext] = trapezoidalIntegration(pred_15_004_td_m.optimumOutput.timeNodes(1:pred_15_004_td_m_to),pred_15_004_td_m.optimumOutput.muscleMoments.joints(7,1:pred_15_004_td_m_to));
[pred_15_002_m_td_hai,pred_15_002_td_m_hai_fle,pred_15_002_td_m_hai_ext] = trapezoidalIntegration(pred_15_002_td_m.optimumOutput.timeNodes(1:pred_15_002_td_m_to),pred_15_002_td_m.optimumOutput.muscleMoments.joints(7,1:pred_15_002_td_m_to));

[pred_15_006_p_ik_hai,pred_15_006_ik_p_hai_fle,pred_15_006_ik_p_hai_ext] = trapezoidalIntegration(pred_15_006_ik_p.optimumOutput.timeNodes(1:pred_15_006_ik_p_to),pred_15_006_ik_p.optimumOutput.muscleMoments.joints(7,1:pred_15_006_ik_p_to));
[pred_15_004_p_ik_hai,pred_15_004_ik_p_hai_fle,pred_15_004_ik_p_hai_ext] = trapezoidalIntegration(pred_15_004_ik_p.optimumOutput.timeNodes(1:pred_15_004_ik_p_to),pred_15_004_ik_p.optimumOutput.muscleMoments.joints(7,1:pred_15_004_ik_p_to));
[pred_15_002_p_ik_hai,pred_15_002_ik_p_hai_fle,pred_15_002_ik_p_hai_ext] = trapezoidalIntegration(pred_15_002_ik_p.optimumOutput.timeNodes(1:pred_15_002_ik_p_to),pred_15_002_ik_p.optimumOutput.muscleMoments.joints(7,1:pred_15_002_ik_p_to));
[pred_15_006_m_ik_hai,pred_15_006_ik_m_hai_fle,pred_15_006_ik_m_hai_ext] = trapezoidalIntegration(pred_15_006_ik_m.optimumOutput.timeNodes(1:pred_15_006_ik_m_to),pred_15_006_ik_m.optimumOutput.muscleMoments.joints(7,1:pred_15_006_ik_m_to));
[pred_15_004_m_ik_hai,pred_15_004_ik_m_hai_fle,pred_15_004_ik_m_hai_ext] = trapezoidalIntegration(pred_15_004_ik_m.optimumOutput.timeNodes(1:pred_15_004_ik_m_to),pred_15_004_ik_m.optimumOutput.muscleMoments.joints(7,1:pred_15_004_ik_m_to));
[pred_15_002_m_ik_hai,pred_15_002_ik_m_hai_fle,pred_15_002_ik_m_hai_ext] = trapezoidalIntegration(pred_15_002_ik_m.optimumOutput.timeNodes(1:pred_15_002_ik_m_to),pred_15_002_ik_m.optimumOutput.muscleMoments.joints(7,1:pred_15_002_ik_m_to));

% Calculate impulses

% Vertical
pred_15_vImp = trapz(pred_15.optimumOutput.timeNodes(1:pred_15_to),pred_15.optimumOutput.GRFs.R(1:pred_15_to,2)-(72.2*9.81));

pred_15_006_td_p_vImp = trapz(pred_15_006_td_p.optimumOutput.timeNodes(1:pred_15_006_td_p_to),pred_15_006_td_p.optimumOutput.GRFs.R(1:pred_15_006_td_p_to,2)-(72.2*9.81));
pred_15_004_td_p_vImp = trapz(pred_15_004_td_p.optimumOutput.timeNodes(1:pred_15_004_td_p_to),pred_15_004_td_p.optimumOutput.GRFs.R(1:pred_15_004_td_p_to,2)-(72.2*9.81));
pred_15_002_td_p_vImp = trapz(pred_15_002_td_p.optimumOutput.timeNodes(1:pred_15_002_td_p_to),pred_15_002_td_p.optimumOutput.GRFs.R(1:pred_15_002_td_p_to,2)-(72.2*9.81));
pred_15_006_td_m_vImp = trapz(pred_15_006_td_m.optimumOutput.timeNodes(1:pred_15_006_td_m_to),pred_15_006_td_m.optimumOutput.GRFs.R(1:pred_15_006_td_m_to,2)-(72.2*9.81));
pred_15_004_td_m_vImp = trapz(pred_15_004_td_m.optimumOutput.timeNodes(1:pred_15_004_td_m_to),pred_15_004_td_m.optimumOutput.GRFs.R(1:pred_15_004_td_m_to,2)-(72.2*9.81));
pred_15_002_td_m_vImp = trapz(pred_15_002_td_m.optimumOutput.timeNodes(1:pred_15_002_td_m_to),pred_15_002_td_m.optimumOutput.GRFs.R(1:pred_15_002_td_m_to,2)-(72.2*9.81));

pred_15_006_ik_p_vImp = trapz(pred_15_006_ik_p.optimumOutput.timeNodes(1:pred_15_006_ik_p_to),pred_15_006_ik_p.optimumOutput.GRFs.R(1:pred_15_006_ik_p_to,2)-(72.2*9.81));
pred_15_004_ik_p_vImp = trapz(pred_15_004_ik_p.optimumOutput.timeNodes(1:pred_15_004_ik_p_to),pred_15_004_ik_p.optimumOutput.GRFs.R(1:pred_15_004_ik_p_to,2)-(72.2*9.81));
pred_15_002_ik_p_vImp = trapz(pred_15_002_ik_p.optimumOutput.timeNodes(1:pred_15_002_ik_p_to),pred_15_002_ik_p.optimumOutput.GRFs.R(1:pred_15_002_ik_p_to,2)-(72.2*9.81));
pred_15_006_ik_m_vImp = trapz(pred_15_006_ik_m.optimumOutput.timeNodes(1:pred_15_006_ik_m_to),pred_15_006_ik_m.optimumOutput.GRFs.R(1:pred_15_006_ik_m_to,2)-(72.2*9.81));
pred_15_004_ik_m_vImp = trapz(pred_15_004_ik_m.optimumOutput.timeNodes(1:pred_15_004_ik_m_to),pred_15_004_ik_m.optimumOutput.GRFs.R(1:pred_15_004_ik_m_to,2)-(72.2*9.81));
pred_15_002_ik_m_vImp = trapz(pred_15_002_ik_m.optimumOutput.timeNodes(1:pred_15_002_ik_m_to),pred_15_002_ik_m.optimumOutput.GRFs.R(1:pred_15_002_ik_m_to,2)-(72.2*9.81));

% Horizontal
pred_15_hImp = trapz(pred_15.optimumOutput.timeNodes(1:pred_15_to),pred_15.optimumOutput.GRFs.R(1:pred_15_to,1));

pred_15_006_td_p_hImp = trapz(pred_15_006_td_p.optimumOutput.timeNodes(1:pred_15_006_td_p_to),pred_15_006_td_p.optimumOutput.GRFs.R(1:pred_15_006_td_p_to,1));
pred_15_004_td_p_hImp = trapz(pred_15_004_td_p.optimumOutput.timeNodes(1:pred_15_004_td_p_to),pred_15_004_td_p.optimumOutput.GRFs.R(1:pred_15_004_td_p_to,1));
pred_15_002_td_p_hImp = trapz(pred_15_002_td_p.optimumOutput.timeNodes(1:pred_15_002_td_p_to),pred_15_002_td_p.optimumOutput.GRFs.R(1:pred_15_002_td_p_to,1));
pred_15_006_td_m_hImp = trapz(pred_15_006_td_m.optimumOutput.timeNodes(1:pred_15_006_td_m_to),pred_15_006_td_m.optimumOutput.GRFs.R(1:pred_15_006_td_m_to,1));
pred_15_004_td_m_hImp = trapz(pred_15_004_td_m.optimumOutput.timeNodes(1:pred_15_004_td_m_to),pred_15_004_td_m.optimumOutput.GRFs.R(1:pred_15_004_td_m_to,1));
pred_15_002_td_m_hImp = trapz(pred_15_002_td_m.optimumOutput.timeNodes(1:pred_15_002_td_m_to),pred_15_002_td_m.optimumOutput.GRFs.R(1:pred_15_002_td_m_to,1));

pred_15_006_ik_p_hImp = trapz(pred_15_006_ik_p.optimumOutput.timeNodes(1:pred_15_006_ik_p_to),pred_15_006_ik_p.optimumOutput.GRFs.R(1:pred_15_006_ik_p_to,1));
pred_15_004_ik_p_hImp = trapz(pred_15_004_ik_p.optimumOutput.timeNodes(1:pred_15_004_ik_p_to),pred_15_004_ik_p.optimumOutput.GRFs.R(1:pred_15_004_ik_p_to,1));
pred_15_002_ik_p_hImp = trapz(pred_15_002_ik_p.optimumOutput.timeNodes(1:pred_15_002_ik_p_to),pred_15_002_ik_p.optimumOutput.GRFs.R(1:pred_15_002_ik_p_to,1));
pred_15_006_ik_m_hImp = trapz(pred_15_006_ik_m.optimumOutput.timeNodes(1:pred_15_006_ik_m_to),pred_15_006_ik_m.optimumOutput.GRFs.R(1:pred_15_006_ik_m_to,1));
pred_15_004_ik_m_hImp = trapz(pred_15_004_ik_m.optimumOutput.timeNodes(1:pred_15_004_ik_m_to),pred_15_004_ik_m.optimumOutput.GRFs.R(1:pred_15_004_ik_m_to,1));
pred_15_002_ik_m_hImp = trapz(pred_15_002_ik_m.optimumOutput.timeNodes(1:pred_15_002_ik_m_to),pred_15_002_ik_m.optimumOutput.GRFs.R(1:pred_15_002_ik_m_to,1));

% Braking Impulse
[pred_15_hImp_brake,pred_15_hImp_frame] = min(cumtrapz(pred_15.optimumOutput.timeNodes(1:pred_15_to),pred_15.optimumOutput.GRFs.R(1:pred_15_to,1)));

[pred_15_006_td_p_hImp_brake,pred_15_006_td_p_hImp_frame] = min(cumtrapz(pred_15_006_td_p.optimumOutput.timeNodes(1:pred_15_006_td_p_to),pred_15_006_td_p.optimumOutput.GRFs.R(1:pred_15_006_td_p_to,1)));
[pred_15_004_td_p_hImp_brake,pred_15_004_td_p_hImp_frame] = min(cumtrapz(pred_15_004_td_p.optimumOutput.timeNodes(1:pred_15_004_td_p_to),pred_15_004_td_p.optimumOutput.GRFs.R(1:pred_15_004_td_p_to,1)));
[pred_15_002_td_p_hImp_brake,pred_15_002_td_p_hImp_frame] = min(cumtrapz(pred_15_002_td_p.optimumOutput.timeNodes(1:pred_15_002_td_p_to),pred_15_002_td_p.optimumOutput.GRFs.R(1:pred_15_002_td_p_to,1)));
[pred_15_006_td_m_hImp_brake,pred_15_006_td_m_hImp_frame] = min(cumtrapz(pred_15_006_td_m.optimumOutput.timeNodes(1:pred_15_006_td_m_to),pred_15_006_td_m.optimumOutput.GRFs.R(1:pred_15_006_td_m_to,1)));
[pred_15_004_td_m_hImp_brake,pred_15_004_td_m_hImp_frame] = min(cumtrapz(pred_15_004_td_m.optimumOutput.timeNodes(1:pred_15_004_td_m_to),pred_15_004_td_m.optimumOutput.GRFs.R(1:pred_15_004_td_m_to,1)));
[pred_15_002_td_m_hImp_brake,pred_15_002_td_m_hImp_frame] = min(cumtrapz(pred_15_002_td_m.optimumOutput.timeNodes(1:pred_15_002_td_m_to),pred_15_002_td_m.optimumOutput.GRFs.R(1:pred_15_002_td_m_to,1)));

[pred_15_006_ik_p_hImp_brake,pred_15_006_ik_p_hImp_frame] = min(cumtrapz(pred_15_006_ik_p.optimumOutput.timeNodes(1:pred_15_006_ik_p_to),pred_15_006_ik_p.optimumOutput.GRFs.R(1:pred_15_006_ik_p_to,1)));
[pred_15_004_ik_p_hImp_brake,pred_15_004_ik_p_hImp_frame] = min(cumtrapz(pred_15_004_ik_p.optimumOutput.timeNodes(1:pred_15_004_ik_p_to),pred_15_004_ik_p.optimumOutput.GRFs.R(1:pred_15_004_ik_p_to,1)));
[pred_15_002_ik_p_hImp_brake,pred_15_002_ik_p_hImp_frame] = min(cumtrapz(pred_15_002_ik_p.optimumOutput.timeNodes(1:pred_15_002_ik_p_to),pred_15_002_ik_p.optimumOutput.GRFs.R(1:pred_15_002_ik_p_to,1)));
[pred_15_006_ik_m_hImp_brake,pred_15_006_ik_m_hImp_frame] = min(cumtrapz(pred_15_006_ik_m.optimumOutput.timeNodes(1:pred_15_006_ik_m_to),pred_15_006_ik_m.optimumOutput.GRFs.R(1:pred_15_006_ik_m_to,1)));
[pred_15_004_ik_m_hImp_brake,pred_15_004_ik_m_hImp_frame] = min(cumtrapz(pred_15_004_ik_m.optimumOutput.timeNodes(1:pred_15_004_ik_m_to),pred_15_004_ik_m.optimumOutput.GRFs.R(1:pred_15_004_ik_m_to,1)));
[pred_15_002_ik_m_hImp_brake,pred_15_002_ik_m_hImp_frame] = min(cumtrapz(pred_15_002_ik_m.optimumOutput.timeNodes(1:pred_15_002_ik_m_to),pred_15_002_ik_m.optimumOutput.GRFs.R(1:pred_15_002_ik_m_to,1)));


% Percentage time spent braking in contact
pred_15_hImp_brake_time = ((pred_15.optimumOutput.timeNodes(pred_15_hImp_frame)-pred_15.optimumOutput.timeNodes(1))/pred_15_ct)*100;

% Propulsive Impulse
pred_15_hImp_prop = pred_15_hImp - pred_15_hImp_brake;

pred_15_006_td_p_hImp_prop = pred_15_006_td_p_hImp - pred_15_006_td_p_hImp_brake;
pred_15_004_td_p_hImp_prop = pred_15_004_td_p_hImp - pred_15_004_td_p_hImp_brake;
pred_15_002_td_p_hImp_prop = pred_15_002_td_p_hImp - pred_15_002_td_p_hImp_brake;
pred_15_006_td_m_hImp_prop = pred_15_006_td_m_hImp - pred_15_006_td_m_hImp_brake;
pred_15_004_td_m_hImp_prop = pred_15_004_td_m_hImp - pred_15_004_td_m_hImp_brake;
pred_15_002_td_m_hImp_prop = pred_15_002_td_m_hImp - pred_15_002_td_m_hImp_brake;

pred_15_006_ik_p_hImp_prop = pred_15_006_ik_p_hImp - pred_15_006_ik_p_hImp_brake;
pred_15_004_ik_p_hImp_prop = pred_15_004_ik_p_hImp - pred_15_004_ik_p_hImp_brake;
pred_15_002_ik_p_hImp_prop = pred_15_002_ik_p_hImp - pred_15_002_ik_p_hImp_brake;
pred_15_006_ik_m_hImp_prop = pred_15_006_ik_m_hImp - pred_15_006_ik_m_hImp_brake;
pred_15_004_ik_m_hImp_prop = pred_15_004_ik_m_hImp - pred_15_004_ik_m_hImp_brake;
pred_15_002_ik_m_hImp_prop = pred_15_002_ik_m_hImp - pred_15_002_ik_m_hImp_brake;


% Horizontal touchdown distance
pred_15_td = pred_15.optimumOutput.modelCOM.r_toes_pos(1,1) - pred_15.optimumOutput.modelCOM.pos(1,1);

pred_15_006_td_p_td = pred_15_006_td_p.optimumOutput.modelCOM.r_toes_pos(1,1) - pred_15_006_td_p.optimumOutput.modelCOM.pos(1,1);
pred_15_004_td_p_td = pred_15_004_td_p.optimumOutput.modelCOM.r_toes_pos(1,1) - pred_15_004_td_p.optimumOutput.modelCOM.pos(1,1);
pred_15_002_td_p_td = pred_15_002_td_p.optimumOutput.modelCOM.r_toes_pos(1,1) - pred_15_002_td_p.optimumOutput.modelCOM.pos(1,1);
pred_15_006_td_m_td = pred_15_006_td_m.optimumOutput.modelCOM.r_toes_pos(1,1) - pred_15_006_td_m.optimumOutput.modelCOM.pos(1,1);
pred_15_004_td_m_td = pred_15_004_td_m.optimumOutput.modelCOM.r_toes_pos(1,1) - pred_15_004_td_m.optimumOutput.modelCOM.pos(1,1);
pred_15_002_td_m_td = pred_15_002_td_m.optimumOutput.modelCOM.r_toes_pos(1,1) - pred_15_002_td_m.optimumOutput.modelCOM.pos(1,1);

% Inter-knee touchdown distance
pred_15_ik = pred_15.optimumOutput.modelCOM.r_knee_pos(1,1) - pred_15.optimumOutput.modelCOM.l_knee_pos(1,1);

pred_15_006_ik_p_ik = pred_15_006_ik_p.optimumOutput.modelCOM.r_knee_pos(1,1) - pred_15_006_ik_p.optimumOutput.modelCOM.l_knee_pos(1,1);
pred_15_004_ik_p_ik = pred_15_004_ik_p.optimumOutput.modelCOM.r_knee_pos(1,1) - pred_15_004_ik_p.optimumOutput.modelCOM.l_knee_pos(1,1);
pred_15_002_ik_p_ik = pred_15_002_ik_p.optimumOutput.modelCOM.r_knee_pos(1,1) - pred_15_002_ik_p.optimumOutput.modelCOM.l_knee_pos(1,1);
pred_15_006_ik_m_ik = pred_15_006_ik_m.optimumOutput.modelCOM.r_knee_pos(1,1) - pred_15_006_ik_m.optimumOutput.modelCOM.l_knee_pos(1,1);
pred_15_004_ik_m_ik = pred_15_004_ik_m.optimumOutput.modelCOM.r_knee_pos(1,1) - pred_15_004_ik_m.optimumOutput.modelCOM.l_knee_pos(1,1);
pred_15_002_ik_m_ik = pred_15_002_ik_m.optimumOutput.modelCOM.r_knee_pos(1,1) - pred_15_002_ik_m.optimumOutput.modelCOM.l_knee_pos(1,1);



