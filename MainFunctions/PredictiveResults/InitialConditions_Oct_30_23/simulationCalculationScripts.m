% Script that performs necessary calculations on simulation results

clear; clc;
close all

pred = load('pred_sprinting_data_26-October-2023__10-52-28___Nominal.mat');

pred_006_td_p = load('pred_sprinting_data_28-October-2023__10-02-52___HTD_Plus_6.mat');
pred_004_td_p = load('pred_sprinting_data_26-October-2023__22-16-25___HTD_Plus_4.mat');
pred_002_td_p = load('pred_sprinting_data_27-October-2023__18-13-56___HTD_Plus_2.mat');
pred_002_td_m = load('pred_sprinting_data_28-October-2023__01-01-40___HTD_Minus_2.mat');
pred_004_td_m = load('pred_sprinting_data_28-October-2023__10-08-04___HTD_Minus_4.mat'); 
pred_006_td_m = load('pred_sprinting_data_28-October-2023__09-53-48___HTD_Minus_6.mat');

pred_006_ik_p = load('pred_sprinting_data_26-October-2023__20-13-02___IKTD_Plus_6.mat');
pred_004_ik_p = load('pred_sprinting_data_28-October-2023__16-43-06___IKTD_Plus_4.mat');
pred_002_ik_p = load('pred_sprinting_data_27-October-2023__11-17-52___IKTD_Plus_2.mat');
pred_006_ik_m = load('pred_sprinting_data_28-October-2023__13-58-45___IKTD_Minus_6.mat');
pred_004_ik_m = load('pred_sprinting_data_26-October-2023__23-37-47___IKTD_Minus_4.mat');
pred_002_ik_m = load('pred_sprinting_data_27-October-2023__11-55-38___IKTD_Minus_2.mat');



% Modified qdot's for Power calculations
pred_qdot = pred.optimumOutput.optVars_nsc.qdot(:,mod(1:length(pred.optimumOutput.optVars_nsc.qdot(:,:)),4)~=0);
pred_qdot(:,end+1) = pred.optimumOutput.optVars_nsc.qdot(:,end);

pred_q = pred.optimumOutput.optVars_nsc.q(:,mod(1:length(pred.optimumOutput.optVars_nsc.q(:,:)),4)~=0);
pred_q(:,end+1) = pred.optimumOutput.optVars_nsc.q(:,end);

% Joint Powers
pred_power = pred_qdot.*pred.optimumOutput.muscleMoments.joints;

% Determine contact times
pred_to  = find(pred.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;

pred_006_td_p_to = find(pred_006_td_p.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_004_td_p_to = find(pred_004_td_p.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_002_td_p_to = find(pred_002_td_p.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_006_td_m_to = find(pred_006_td_m.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_004_td_m_to = find(pred_004_td_m.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_002_td_m_to = find(pred_002_td_m.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;

pred_006_ik_p_to = find(pred_006_ik_p.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_004_ik_p_to = find(pred_004_ik_p.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_002_ik_p_to = find(pred_002_ik_p.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_006_ik_m_to = find(pred_006_ik_m.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_004_ik_m_to = find(pred_004_ik_m.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;
pred_002_ik_m_to = find(pred_002_ik_m.optimumOutput.GRFs.R(2:end,2) < 20,1) + 1;

pred_to_i = pred.optimumOutput.timeNodes(pred_to);

pred_006_td_p_to_i = pred_006_td_p.optimumOutput.timeNodes(pred_006_td_p_to);
pred_004_td_p_to_i = pred_004_td_p.optimumOutput.timeNodes(pred_004_td_p_to);
pred_002_td_p_to_i = pred_002_td_p.optimumOutput.timeNodes(pred_002_td_p_to);
pred_006_td_m_to_i = pred_006_td_m.optimumOutput.timeNodes(pred_006_td_m_to);
pred_004_td_m_to_i = pred_004_td_m.optimumOutput.timeNodes(pred_004_td_m_to);
pred_002_td_m_to_i = pred_002_td_m.optimumOutput.timeNodes(pred_002_td_m_to);

pred_006_ik_p_to_i = pred_006_ik_p.optimumOutput.timeNodes(pred_006_ik_p_to);
pred_004_ik_p_to_i = pred_004_ik_p.optimumOutput.timeNodes(pred_004_ik_p_to);
pred_002_ik_p_to_i = pred_002_ik_p.optimumOutput.timeNodes(pred_002_ik_p_to);
pred_006_ik_m_to_i = pred_006_ik_m.optimumOutput.timeNodes(pred_006_ik_m_to);
pred_004_ik_m_to_i = pred_004_ik_m.optimumOutput.timeNodes(pred_004_ik_m_to);
pred_002_ik_m_to_i = pred_002_ik_m.optimumOutput.timeNodes(pred_002_ik_m_to);

% Determine the take-off frame in kinematics timeline
pred_to_i_kin = find(pred.optimumOutput.timeGrid == pred_to_i);

pred_006_td_p_to_i_kin = find(pred_006_td_p.optimumOutput.timeGrid == pred_006_td_p_to_i);

% Average hip extension angular velocity during stance
pred_hip_vel = mean(pred.optimumOutput.optVars_nsc.qdot(7,1:pred_to_i_kin));

pred_006_td_p_hip_vel = mean(pred_006_td_p.optimumOutput.optVars_nsc.qdot(7,1:pred_006_td_p_to_i_kin));


pred_ct = pred.optimumOutput.timeNodes(pred_to) - pred.optimumOutput.timeNodes(1);

pred_006_td_p_ct = pred_006_td_p.optimumOutput.timeNodes(pred_006_td_p_to) - pred_006_td_p.optimumOutput.timeNodes(1);
pred_004_td_p_ct = pred_004_td_p.optimumOutput.timeNodes(pred_004_td_p_to) - pred_004_td_p.optimumOutput.timeNodes(1);
pred_002_td_p_ct = pred_002_td_p.optimumOutput.timeNodes(pred_002_td_p_to) - pred_002_td_p.optimumOutput.timeNodes(1);
pred_006_td_m_ct = pred_006_td_m.optimumOutput.timeNodes(pred_006_td_m_to) - pred_006_td_m.optimumOutput.timeNodes(1);
pred_004_td_m_ct = pred_004_td_m.optimumOutput.timeNodes(pred_004_td_m_to) - pred_004_td_m.optimumOutput.timeNodes(1);
pred_002_td_m_ct = pred_002_td_m.optimumOutput.timeNodes(pred_002_td_m_to) - pred_002_td_m.optimumOutput.timeNodes(1);

pred_006_ik_p_ct = pred_006_ik_p.optimumOutput.timeNodes(pred_006_ik_p_to) - pred_006_ik_p.optimumOutput.timeNodes(1);
pred_004_ik_p_ct = pred_004_ik_p.optimumOutput.timeNodes(pred_004_ik_p_to) - pred_004_ik_p.optimumOutput.timeNodes(1);
pred_002_ik_p_ct = pred_002_ik_p.optimumOutput.timeNodes(pred_002_ik_p_to) - pred_002_ik_p.optimumOutput.timeNodes(1);
pred_006_ik_m_ct = pred_006_ik_m.optimumOutput.timeNodes(pred_006_ik_m_to) - pred_006_ik_m.optimumOutput.timeNodes(1);
pred_004_ik_m_ct = pred_004_ik_m.optimumOutput.timeNodes(pred_004_ik_m_to) - pred_004_ik_m.optimumOutput.timeNodes(1);
pred_002_ik_m_ct = pred_002_ik_m.optimumOutput.timeNodes(pred_002_ik_m_to) - pred_002_ik_m.optimumOutput.timeNodes(1);




% Step duration
pred_sd = pred.optimumOutput.optVars_nsc.totalTime;

pred_006_td_p_sd = pred_006_td_p.optimumOutput.optVars_nsc.totalTime;
pred_004_td_p_sd = pred_004_td_p.optimumOutput.optVars_nsc.totalTime;
pred_002_td_p_sd = pred_002_td_p.optimumOutput.optVars_nsc.totalTime;
pred_006_td_m_sd = pred_006_td_m.optimumOutput.optVars_nsc.totalTime;
pred_004_td_m_sd = pred_004_td_m.optimumOutput.optVars_nsc.totalTime;
pred_002_td_m_sd = pred_002_td_m.optimumOutput.optVars_nsc.totalTime;

pred_006_ik_p_sd = pred_006_ik_p.optimumOutput.optVars_nsc.totalTime;
pred_004_ik_p_sd = pred_004_ik_p.optimumOutput.optVars_nsc.totalTime;
pred_002_ik_p_sd = pred_002_ik_p.optimumOutput.optVars_nsc.totalTime;
pred_006_ik_m_sd = pred_006_ik_m.optimumOutput.optVars_nsc.totalTime;
pred_004_ik_m_sd = pred_004_ik_m.optimumOutput.optVars_nsc.totalTime;
pred_002_ik_m_sd = pred_002_ik_m.optimumOutput.optVars_nsc.totalTime;


% Step frequency
pred_sf = 1/pred_sd;

pred_006_td_p_sf = 1/pred_006_td_p_sd;
pred_004_td_p_sf = 1/pred_004_td_p_sd;
pred_002_td_p_sf = 1/pred_002_td_p_sd;
pred_006_td_m_sf = 1/pred_006_td_m_sd;
pred_004_td_m_sf = 1/pred_004_td_m_sd;
pred_002_td_m_sf = 1/pred_002_td_m_sd;

pred_006_ik_p_sf = 1/pred_006_ik_p_sd;
pred_004_ik_p_sf = 1/pred_004_ik_p_sd;
pred_002_ik_p_sf = 1/pred_002_ik_p_sd;
pred_006_ik_m_sf = 1/pred_006_ik_m_sd;
pred_004_ik_m_sf = 1/pred_004_ik_m_sd;
pred_002_ik_m_sf = 1/pred_002_ik_m_sd;


% Swing time
pred_st = (2*(pred_sd - pred_ct)) + pred_ct;

pred_006_td_p_st = (2*(pred_006_td_p_sd - pred_006_td_p_ct)) + pred_006_td_p_ct;
pred_004_td_p_st = (2*(pred_004_td_p_sd - pred_004_td_p_ct)) + pred_004_td_p_ct;
pred_002_td_p_st = (2*(pred_002_td_p_sd - pred_002_td_p_ct)) + pred_002_td_p_ct;
pred_006_td_m_st = (2*(pred_006_td_m_sd - pred_006_td_m_ct)) + pred_006_td_m_ct;
pred_004_td_m_st = (2*(pred_004_td_m_sd - pred_004_td_m_ct)) + pred_004_td_m_ct;
pred_002_td_m_st = (2*(pred_002_td_m_sd - pred_002_td_m_ct)) + pred_002_td_m_ct;

pred_006_ik_p_st = (2*(pred_006_ik_p_sd - pred_006_ik_p_ct)) + pred_006_ik_p_ct;
pred_004_ik_p_st = (2*(pred_004_ik_p_sd - pred_004_ik_p_ct)) + pred_004_ik_p_ct;
pred_002_ik_p_st = (2*(pred_002_ik_p_sd - pred_002_ik_p_ct)) + pred_002_ik_p_ct;
pred_006_ik_m_st = (2*(pred_006_ik_m_sd - pred_006_ik_m_ct)) + pred_006_ik_m_ct;
pred_004_ik_m_st = (2*(pred_004_ik_m_sd - pred_004_ik_m_ct)) + pred_004_ik_m_ct;
pred_002_ik_m_st = (2*(pred_002_ik_m_sd - pred_002_ik_m_ct)) + pred_002_ik_m_ct;


% Aerial time
pred_at = pred_sd - pred_ct;

pred_006_td_p_at = pred_006_td_p_sd - pred_006_td_p_ct;
pred_004_td_p_at = pred_004_td_p_sd - pred_004_td_p_ct;
pred_002_td_p_at = pred_002_td_p_sd - pred_002_td_p_ct;
pred_006_td_m_at = pred_006_td_m_sd - pred_006_td_m_ct;
pred_004_td_m_at = pred_004_td_m_sd - pred_004_td_m_ct;
pred_002_td_m_at = pred_002_td_m_sd - pred_002_td_m_ct;

pred_006_ik_p_at = pred_006_ik_p_sd - pred_006_ik_p_ct;
pred_004_ik_p_at = pred_004_ik_p_sd - pred_004_ik_p_ct;
pred_002_ik_p_at = pred_002_ik_p_sd - pred_002_ik_p_ct;
pred_006_ik_m_at = pred_006_ik_m_sd - pred_006_ik_m_ct;
pred_004_ik_m_at = pred_004_ik_m_sd - pred_004_ik_m_ct;
pred_002_ik_m_at = pred_002_ik_m_sd - pred_002_ik_m_ct;


% Vertical COM velocity at touchdown
pred_COM_v_td = pred.optimumOutput.modelCOM.vel(2,1);

pred_006_td_p_COM_v_td = pred_006_td_p.optimumOutput.modelCOM.vel(2,1);
pred_004_td_p_COM_v_td = pred_004_td_p.optimumOutput.modelCOM.vel(2,1);
pred_002_td_p_COM_v_td = pred_002_td_p.optimumOutput.modelCOM.vel(2,1);
pred_006_td_m_COM_v_td = pred_006_td_m.optimumOutput.modelCOM.vel(2,1);
pred_004_td_m_COM_v_td = pred_004_td_m.optimumOutput.modelCOM.vel(2,1);
pred_002_td_m_COM_v_td = pred_002_td_m.optimumOutput.modelCOM.vel(2,1);

pred_006_ik_p_COM_v_td = pred_006_ik_p.optimumOutput.modelCOM.vel(2,1);
pred_004_ik_p_COM_v_td = pred_004_ik_p.optimumOutput.modelCOM.vel(2,1);
pred_002_ik_p_COM_v_td = pred_002_ik_p.optimumOutput.modelCOM.vel(2,1);
pred_006_ik_m_COM_v_td = pred_006_ik_m.optimumOutput.modelCOM.vel(2,1);
pred_004_ik_m_COM_v_td = pred_004_ik_m.optimumOutput.modelCOM.vel(2,1);
pred_002_ik_m_COM_v_td = pred_002_ik_m.optimumOutput.modelCOM.vel(2,1);

% Vertical COM velocity at takeoff
pred_COM_v_to = pred.optimumOutput.modelCOM.vel(2,pred_to);

pred_006_td_p_COM_v_to = pred_006_td_p.optimumOutput.modelCOM.vel(2,pred_006_td_p_to);
pred_004_td_p_COM_v_to = pred_004_td_p.optimumOutput.modelCOM.vel(2,pred_004_td_p_to);
pred_002_td_p_COM_v_to = pred_002_td_p.optimumOutput.modelCOM.vel(2,pred_002_td_p_to);
pred_006_td_m_COM_v_to = pred_006_td_m.optimumOutput.modelCOM.vel(2,pred_006_td_m_to);
pred_004_td_m_COM_v_to = pred_004_td_m.optimumOutput.modelCOM.vel(2,pred_004_td_m_to);
pred_002_td_m_COM_v_to = pred_002_td_m.optimumOutput.modelCOM.vel(2,pred_002_td_m_to);

pred_006_ik_p_COM_v_to = pred_006_ik_p.optimumOutput.modelCOM.vel(2,pred_006_ik_p_to);
pred_004_ik_p_COM_v_to = pred_004_ik_p.optimumOutput.modelCOM.vel(2,pred_004_ik_p_to);
pred_002_ik_p_COM_v_to = pred_002_ik_p.optimumOutput.modelCOM.vel(2,pred_002_ik_p_to);
pred_006_ik_m_COM_v_to = pred_006_ik_m.optimumOutput.modelCOM.vel(2,pred_006_ik_m_to);
pred_004_ik_m_COM_v_to = pred_004_ik_m.optimumOutput.modelCOM.vel(2,pred_004_ik_m_to);
pred_002_ik_m_COM_v_to = pred_002_ik_m.optimumOutput.modelCOM.vel(2,pred_002_ik_m_to);

% Horizontal COM velocity at touchdown
pred_COM_h_td = pred.optimumOutput.modelCOM.vel(1,1);

pred_006_td_p_COM_h_td = pred_006_td_p.optimumOutput.modelCOM.vel(1,1);
pred_004_td_p_COM_h_td = pred_004_td_p.optimumOutput.modelCOM.vel(1,1);
pred_002_td_p_COM_h_td = pred_002_td_p.optimumOutput.modelCOM.vel(1,1);
pred_006_td_m_COM_h_td = pred_006_td_m.optimumOutput.modelCOM.vel(1,1);
pred_004_td_m_COM_h_td = pred_004_td_m.optimumOutput.modelCOM.vel(1,1);
pred_002_td_m_COM_h_td = pred_002_td_m.optimumOutput.modelCOM.vel(1,1);

pred_006_ik_p_COM_h_td = pred_006_ik_p.optimumOutput.modelCOM.vel(1,1);
pred_004_ik_p_COM_h_td = pred_004_ik_p.optimumOutput.modelCOM.vel(1,1);
pred_002_ik_p_COM_h_td = pred_002_ik_p.optimumOutput.modelCOM.vel(1,1);
pred_006_ik_m_COM_h_td = pred_006_ik_m.optimumOutput.modelCOM.vel(1,1);
pred_004_ik_m_COM_h_td = pred_004_ik_m.optimumOutput.modelCOM.vel(1,1);
pred_002_ik_m_COM_h_td = pred_002_ik_m.optimumOutput.modelCOM.vel(1,1);

% Horizontal COM velocity at takeoff
pred_COM_h_to = pred.optimumOutput.modelCOM.vel(1,pred_to);

pred_006_td_p_COM_h_to = pred_006_td_p.optimumOutput.modelCOM.vel(1,pred_006_td_p_to);
pred_004_td_p_COM_h_to = pred_004_td_p.optimumOutput.modelCOM.vel(1,pred_004_td_p_to);
pred_002_td_p_COM_h_to = pred_002_td_p.optimumOutput.modelCOM.vel(1,pred_002_td_p_to);
pred_006_td_m_COM_h_to = pred_006_td_m.optimumOutput.modelCOM.vel(1,pred_006_td_m_to);
pred_004_td_m_COM_h_to = pred_004_td_m.optimumOutput.modelCOM.vel(1,pred_004_td_m_to);
pred_002_td_m_COM_h_to = pred_002_td_m.optimumOutput.modelCOM.vel(1,pred_002_td_m_to);

pred_006_ik_p_COM_h_to = pred_006_ik_p.optimumOutput.modelCOM.vel(1,pred_006_ik_p_to);
pred_004_ik_p_COM_h_to = pred_004_ik_p.optimumOutput.modelCOM.vel(1,pred_004_ik_p_to);
pred_002_ik_p_COM_h_to = pred_002_ik_p.optimumOutput.modelCOM.vel(1,pred_002_ik_p_to);
pred_006_ik_m_COM_h_to = pred_006_ik_m.optimumOutput.modelCOM.vel(1,pred_006_ik_m_to);
pred_004_ik_m_COM_h_to = pred_004_ik_m.optimumOutput.modelCOM.vel(1,pred_004_ik_m_to);
pred_002_ik_m_COM_h_to = pred_002_ik_m.optimumOutput.modelCOM.vel(1,pred_002_ik_m_to);

% Step length
pred_sl = pred.optimumOutput.optVars_nsc.q(4,end);

pred_006_td_p_sl = pred_006_td_p.optimumOutput.optVars_nsc.q(4,end);
pred_004_td_p_sl = pred_004_td_p.optimumOutput.optVars_nsc.q(4,end);
pred_002_td_p_sl = pred_002_td_p.optimumOutput.optVars_nsc.q(4,end);
pred_006_td_m_sl = pred_006_td_m.optimumOutput.optVars_nsc.q(4,end);
pred_004_td_m_sl = pred_004_td_m.optimumOutput.optVars_nsc.q(4,end);
pred_002_td_m_sl = pred_002_td_m.optimumOutput.optVars_nsc.q(4,end);

pred_006_ik_p_sl = pred_006_ik_p.optimumOutput.optVars_nsc.q(4,end);
pred_004_ik_p_sl = pred_004_ik_p.optimumOutput.optVars_nsc.q(4,end);
pred_002_ik_p_sl = pred_002_ik_p.optimumOutput.optVars_nsc.q(4,end);
pred_006_ik_m_sl = pred_006_ik_m.optimumOutput.optVars_nsc.q(4,end);
pred_004_ik_m_sl = pred_004_ik_m.optimumOutput.optVars_nsc.q(4,end);
pred_002_ik_m_sl = pred_002_ik_m.optimumOutput.optVars_nsc.q(4,end);

% Speed
pred_spe = pred_sf*pred_sl;

pred_006_td_p_spe = pred_006_td_p_sf*pred_006_td_p_sl;
pred_004_td_p_spe = pred_004_td_p_sf*pred_004_td_p_sl;
pred_002_td_p_spe = pred_002_td_p_sf*pred_002_td_p_sl;
pred_006_td_m_spe = pred_006_td_m_sf*pred_006_td_m_sl;
pred_004_td_m_spe = pred_004_td_m_sf*pred_004_td_m_sl;
pred_002_td_m_spe = pred_002_td_m_sf*pred_002_td_m_sl;

pred_006_ik_p_spe = pred_006_ik_p_sf*pred_006_ik_p_sl;
pred_004_ik_p_spe = pred_004_ik_p_sf*pred_004_ik_p_sl;
pred_002_ik_p_spe = pred_002_ik_p_sf*pred_002_ik_p_sl;
pred_006_ik_m_spe = pred_006_ik_m_sf*pred_006_ik_m_sl;
pred_004_ik_m_spe = pred_004_ik_m_sf*pred_004_ik_m_sl;
pred_002_ik_m_spe = pred_002_ik_m_sf*pred_002_ik_m_sl;


% Calculate stance joint work

% Stance Ankle
pred_aw = trapz(pred.optimumOutput.timeNodes(1:pred_to),pred_power(11,1:pred_to));

% Stance Knee
pred_kw = trapz(pred.optimumOutput.timeNodes(1:pred_to),pred_power(10,1:pred_to));

% Stance Hip
pred_hw = trapz(pred.optimumOutput.timeNodes(1:pred_to),pred_power(7,1:pred_to));

% Calculate stance angular joint impulse

% Stance Ankle
[pred_aai,pred_aai_df,pred_aai_pf] = trapezoidalIntegration(pred.optimumOutput.timeNodes(1:pred_to),pred.optimumOutput.muscleMoments.joints(11,1:pred_to));

[pred_006_p_td_aai,pred_006_td_p_aai_df,pred_006_td_p_aai_pf] = trapezoidalIntegration(pred_006_td_p.optimumOutput.timeNodes(1:pred_006_td_p_to),pred_006_td_p.optimumOutput.muscleMoments.joints(11,1:pred_006_td_p_to));
[pred_004_p_td_aai,pred_004_td_p_aai_df,pred_004_td_p_aai_pf] = trapezoidalIntegration(pred_004_td_p.optimumOutput.timeNodes(1:pred_004_td_p_to),pred_004_td_p.optimumOutput.muscleMoments.joints(11,1:pred_004_td_p_to));
[pred_002_p_td_aai,pred_002_td_p_aai_df,pred_002_td_p_aai_pf] = trapezoidalIntegration(pred_002_td_p.optimumOutput.timeNodes(1:pred_002_td_p_to),pred_002_td_p.optimumOutput.muscleMoments.joints(11,1:pred_002_td_p_to));
[pred_006_m_td_aai,pred_006_td_m_aai_df,pred_006_td_m_aai_pf] = trapezoidalIntegration(pred_006_td_m.optimumOutput.timeNodes(1:pred_006_td_m_to),pred_006_td_m.optimumOutput.muscleMoments.joints(11,1:pred_006_td_m_to));
[pred_004_m_td_aai,pred_004_td_m_aai_df,pred_004_td_m_aai_pf] = trapezoidalIntegration(pred_004_td_m.optimumOutput.timeNodes(1:pred_004_td_m_to),pred_004_td_m.optimumOutput.muscleMoments.joints(11,1:pred_004_td_m_to));
[pred_002_m_td_aai,pred_002_td_m_aai_df,pred_002_td_m_aai_pf] = trapezoidalIntegration(pred_002_td_m.optimumOutput.timeNodes(1:pred_002_td_m_to),pred_002_td_m.optimumOutput.muscleMoments.joints(11,1:pred_002_td_m_to));

[pred_006_p_ik_aai,pred_006_ik_p_aai_df,pred_006_ik_p_aai_pf] = trapezoidalIntegration(pred_006_ik_p.optimumOutput.timeNodes(1:pred_006_ik_p_to),pred_006_ik_p.optimumOutput.muscleMoments.joints(11,1:pred_006_ik_p_to));
[pred_004_p_ik_aai,pred_004_ik_p_aai_df,pred_004_ik_p_aai_pf] = trapezoidalIntegration(pred_004_ik_p.optimumOutput.timeNodes(1:pred_004_ik_p_to),pred_004_ik_p.optimumOutput.muscleMoments.joints(11,1:pred_004_ik_p_to));
[pred_002_p_ik_aai,pred_002_ik_p_aai_df,pred_002_ik_p_aai_pf] = trapezoidalIntegration(pred_002_ik_p.optimumOutput.timeNodes(1:pred_002_ik_p_to),pred_002_ik_p.optimumOutput.muscleMoments.joints(11,1:pred_002_ik_p_to));
[pred_006_m_ik_aai,pred_006_ik_m_aai_df,pred_006_ik_m_aai_pf] = trapezoidalIntegration(pred_006_ik_m.optimumOutput.timeNodes(1:pred_006_ik_m_to),pred_006_ik_m.optimumOutput.muscleMoments.joints(11,1:pred_006_ik_m_to));
[pred_004_m_ik_aai,pred_004_ik_m_aai_df,pred_004_ik_m_aai_pf] = trapezoidalIntegration(pred_004_ik_m.optimumOutput.timeNodes(1:pred_004_ik_m_to),pred_004_ik_m.optimumOutput.muscleMoments.joints(11,1:pred_004_ik_m_to));
[pred_002_m_ik_aai,pred_002_ik_m_aai_df,pred_002_ik_m_aai_pf] = trapezoidalIntegration(pred_002_ik_m.optimumOutput.timeNodes(1:pred_002_ik_m_to),pred_002_ik_m.optimumOutput.muscleMoments.joints(11,1:pred_002_ik_m_to));


% Stance Knee
[pred_kai,pred_kai_ext,pred_kai_fle] = trapezoidalIntegration(pred.optimumOutput.timeNodes(1:pred_to),pred.optimumOutput.muscleMoments.joints(10,1:pred_to));

[pred_006_p_td_kai,pred_006_td_p_kai_ext,pred_006_td_p_kai_fle] = trapezoidalIntegration(pred_006_td_p.optimumOutput.timeNodes(1:pred_006_td_p_to),pred_006_td_p.optimumOutput.muscleMoments.joints(10,1:pred_006_td_p_to));
[pred_004_p_td_kai,pred_004_td_p_kai_ext,pred_004_td_p_kai_fle] = trapezoidalIntegration(pred_004_td_p.optimumOutput.timeNodes(1:pred_004_td_p_to),pred_004_td_p.optimumOutput.muscleMoments.joints(10,1:pred_004_td_p_to));
[pred_002_p_td_kai,pred_002_td_p_kai_ext,pred_002_td_p_kai_fle] = trapezoidalIntegration(pred_002_td_p.optimumOutput.timeNodes(1:pred_002_td_p_to),pred_002_td_p.optimumOutput.muscleMoments.joints(10,1:pred_002_td_p_to));
[pred_006_m_td_kai,pred_006_td_m_kai_ext,pred_006_td_m_kai_fle] = trapezoidalIntegration(pred_006_td_m.optimumOutput.timeNodes(1:pred_006_td_m_to),pred_006_td_m.optimumOutput.muscleMoments.joints(10,1:pred_006_td_m_to));
[pred_004_m_td_kai,pred_004_td_m_kai_ext,pred_004_td_m_kai_fle] = trapezoidalIntegration(pred_004_td_m.optimumOutput.timeNodes(1:pred_004_td_m_to),pred_004_td_m.optimumOutput.muscleMoments.joints(10,1:pred_004_td_m_to));
[pred_002_m_td_kai,pred_002_td_m_kai_ext,pred_002_td_m_kai_fle] = trapezoidalIntegration(pred_002_td_m.optimumOutput.timeNodes(1:pred_002_td_m_to),pred_002_td_m.optimumOutput.muscleMoments.joints(10,1:pred_002_td_m_to));

[pred_006_p_ik_kai,pred_006_ik_p_kai_ext,pred_006_ik_p_kai_fle] = trapezoidalIntegration(pred_006_ik_p.optimumOutput.timeNodes(1:pred_006_ik_p_to),pred_006_ik_p.optimumOutput.muscleMoments.joints(10,1:pred_006_ik_p_to));
[pred_004_p_ik_kai,pred_004_ik_p_kai_ext,pred_004_ik_p_kai_fle] = trapezoidalIntegration(pred_004_ik_p.optimumOutput.timeNodes(1:pred_004_ik_p_to),pred_004_ik_p.optimumOutput.muscleMoments.joints(10,1:pred_004_ik_p_to));
[pred_002_p_ik_kai,pred_002_ik_p_kai_ext,pred_002_ik_p_kai_fle] = trapezoidalIntegration(pred_002_ik_p.optimumOutput.timeNodes(1:pred_002_ik_p_to),pred_002_ik_p.optimumOutput.muscleMoments.joints(10,1:pred_002_ik_p_to));
[pred_006_m_ik_kai,pred_006_ik_m_kai_ext,pred_006_ik_m_kai_fle] = trapezoidalIntegration(pred_006_ik_m.optimumOutput.timeNodes(1:pred_006_ik_m_to),pred_006_ik_m.optimumOutput.muscleMoments.joints(10,1:pred_006_ik_m_to));
[pred_004_m_ik_kai,pred_004_ik_m_kai_ext,pred_004_ik_m_kai_fle] = trapezoidalIntegration(pred_004_ik_m.optimumOutput.timeNodes(1:pred_004_ik_m_to),pred_004_ik_m.optimumOutput.muscleMoments.joints(10,1:pred_004_ik_m_to));
[pred_002_m_ik_kai,pred_002_ik_m_kai_ext,pred_002_ik_m_kai_fle] = trapezoidalIntegration(pred_002_ik_m.optimumOutput.timeNodes(1:pred_002_ik_m_to),pred_002_ik_m.optimumOutput.muscleMoments.joints(10,1:pred_002_ik_m_to));


% Stance Hip
[pred_hai,pred_hai_fle,pred_hai_ext] = trapezoidalIntegration(pred.optimumOutput.timeNodes(1:pred_to),pred.optimumOutput.muscleMoments.joints(7,1:pred_to));

[pred_006_p_td_hai,pred_006_td_p_hai_fle,pred_006_td_p_hai_ext] = trapezoidalIntegration(pred_006_td_p.optimumOutput.timeNodes(1:pred_006_td_p_to),pred_006_td_p.optimumOutput.muscleMoments.joints(7,1:pred_006_td_p_to));
[pred_004_p_td_hai,pred_004_td_p_hai_fle,pred_004_td_p_hai_ext] = trapezoidalIntegration(pred_004_td_p.optimumOutput.timeNodes(1:pred_004_td_p_to),pred_004_td_p.optimumOutput.muscleMoments.joints(7,1:pred_004_td_p_to));
[pred_002_p_td_hai,pred_002_td_p_hai_fle,pred_002_td_p_hai_ext] = trapezoidalIntegration(pred_002_td_p.optimumOutput.timeNodes(1:pred_002_td_p_to),pred_002_td_p.optimumOutput.muscleMoments.joints(7,1:pred_002_td_p_to));
[pred_006_m_td_hai,pred_006_td_m_hai_fle,pred_006_td_m_hai_ext] = trapezoidalIntegration(pred_006_td_m.optimumOutput.timeNodes(1:pred_006_td_m_to),pred_006_td_m.optimumOutput.muscleMoments.joints(7,1:pred_006_td_m_to));
[pred_004_m_td_hai,pred_004_td_m_hai_fle,pred_004_td_m_hai_ext] = trapezoidalIntegration(pred_004_td_m.optimumOutput.timeNodes(1:pred_004_td_m_to),pred_004_td_m.optimumOutput.muscleMoments.joints(7,1:pred_004_td_m_to));
[pred_002_m_td_hai,pred_002_td_m_hai_fle,pred_002_td_m_hai_ext] = trapezoidalIntegration(pred_002_td_m.optimumOutput.timeNodes(1:pred_002_td_m_to),pred_002_td_m.optimumOutput.muscleMoments.joints(7,1:pred_002_td_m_to));

[pred_006_p_ik_hai,pred_006_ik_p_hai_fle,pred_006_ik_p_hai_ext] = trapezoidalIntegration(pred_006_ik_p.optimumOutput.timeNodes(1:pred_006_ik_p_to),pred_006_ik_p.optimumOutput.muscleMoments.joints(7,1:pred_006_ik_p_to));
[pred_004_p_ik_hai,pred_004_ik_p_hai_fle,pred_004_ik_p_hai_ext] = trapezoidalIntegration(pred_004_ik_p.optimumOutput.timeNodes(1:pred_004_ik_p_to),pred_004_ik_p.optimumOutput.muscleMoments.joints(7,1:pred_004_ik_p_to));
[pred_002_p_ik_hai,pred_002_ik_p_hai_fle,pred_002_ik_p_hai_ext] = trapezoidalIntegration(pred_002_ik_p.optimumOutput.timeNodes(1:pred_002_ik_p_to),pred_002_ik_p.optimumOutput.muscleMoments.joints(7,1:pred_002_ik_p_to));
[pred_006_m_ik_hai,pred_006_ik_m_hai_fle,pred_006_ik_m_hai_ext] = trapezoidalIntegration(pred_006_ik_m.optimumOutput.timeNodes(1:pred_006_ik_m_to),pred_006_ik_m.optimumOutput.muscleMoments.joints(7,1:pred_006_ik_m_to));
[pred_004_m_ik_hai,pred_004_ik_m_hai_fle,pred_004_ik_m_hai_ext] = trapezoidalIntegration(pred_004_ik_m.optimumOutput.timeNodes(1:pred_004_ik_m_to),pred_004_ik_m.optimumOutput.muscleMoments.joints(7,1:pred_004_ik_m_to));
[pred_002_m_ik_hai,pred_002_ik_m_hai_fle,pred_002_ik_m_hai_ext] = trapezoidalIntegration(pred_002_ik_m.optimumOutput.timeNodes(1:pred_002_ik_m_to),pred_002_ik_m.optimumOutput.muscleMoments.joints(7,1:pred_002_ik_m_to));

% Calculate impulses

% Vertical
pred_vImp = trapz(pred.optimumOutput.timeNodes(1:pred_to),pred.optimumOutput.GRFs.R(1:pred_to,2)-(72.2*9.81));

pred_006_td_p_vImp = trapz(pred_006_td_p.optimumOutput.timeNodes(1:pred_006_td_p_to),pred_006_td_p.optimumOutput.GRFs.R(1:pred_006_td_p_to,2)-(72.2*9.81));
pred_004_td_p_vImp = trapz(pred_004_td_p.optimumOutput.timeNodes(1:pred_004_td_p_to),pred_004_td_p.optimumOutput.GRFs.R(1:pred_004_td_p_to,2)-(72.2*9.81));
pred_002_td_p_vImp = trapz(pred_002_td_p.optimumOutput.timeNodes(1:pred_002_td_p_to),pred_002_td_p.optimumOutput.GRFs.R(1:pred_002_td_p_to,2)-(72.2*9.81));
pred_006_td_m_vImp = trapz(pred_006_td_m.optimumOutput.timeNodes(1:pred_006_td_m_to),pred_006_td_m.optimumOutput.GRFs.R(1:pred_006_td_m_to,2)-(72.2*9.81));
pred_004_td_m_vImp = trapz(pred_004_td_m.optimumOutput.timeNodes(1:pred_004_td_m_to),pred_004_td_m.optimumOutput.GRFs.R(1:pred_004_td_m_to,2)-(72.2*9.81));
pred_002_td_m_vImp = trapz(pred_002_td_m.optimumOutput.timeNodes(1:pred_002_td_m_to),pred_002_td_m.optimumOutput.GRFs.R(1:pred_002_td_m_to,2)-(72.2*9.81));

pred_006_ik_p_vImp = trapz(pred_006_ik_p.optimumOutput.timeNodes(1:pred_006_ik_p_to),pred_006_ik_p.optimumOutput.GRFs.R(1:pred_006_ik_p_to,2)-(72.2*9.81));
pred_004_ik_p_vImp = trapz(pred_004_ik_p.optimumOutput.timeNodes(1:pred_004_ik_p_to),pred_004_ik_p.optimumOutput.GRFs.R(1:pred_004_ik_p_to,2)-(72.2*9.81));
pred_002_ik_p_vImp = trapz(pred_002_ik_p.optimumOutput.timeNodes(1:pred_002_ik_p_to),pred_002_ik_p.optimumOutput.GRFs.R(1:pred_002_ik_p_to,2)-(72.2*9.81));
pred_006_ik_m_vImp = trapz(pred_006_ik_m.optimumOutput.timeNodes(1:pred_006_ik_m_to),pred_006_ik_m.optimumOutput.GRFs.R(1:pred_006_ik_m_to,2)-(72.2*9.81));
pred_004_ik_m_vImp = trapz(pred_004_ik_m.optimumOutput.timeNodes(1:pred_004_ik_m_to),pred_004_ik_m.optimumOutput.GRFs.R(1:pred_004_ik_m_to,2)-(72.2*9.81));
pred_002_ik_m_vImp = trapz(pred_002_ik_m.optimumOutput.timeNodes(1:pred_002_ik_m_to),pred_002_ik_m.optimumOutput.GRFs.R(1:pred_002_ik_m_to,2)-(72.2*9.81));

% Horizontal
pred_hImp = trapz(pred.optimumOutput.timeNodes(1:pred_to),pred.optimumOutput.GRFs.R(1:pred_to,1));

pred_006_td_p_hImp = trapz(pred_006_td_p.optimumOutput.timeNodes(1:pred_006_td_p_to),pred_006_td_p.optimumOutput.GRFs.R(1:pred_006_td_p_to,1));
pred_004_td_p_hImp = trapz(pred_004_td_p.optimumOutput.timeNodes(1:pred_004_td_p_to),pred_004_td_p.optimumOutput.GRFs.R(1:pred_004_td_p_to,1));
pred_002_td_p_hImp = trapz(pred_002_td_p.optimumOutput.timeNodes(1:pred_002_td_p_to),pred_002_td_p.optimumOutput.GRFs.R(1:pred_002_td_p_to,1));
pred_006_td_m_hImp = trapz(pred_006_td_m.optimumOutput.timeNodes(1:pred_006_td_m_to),pred_006_td_m.optimumOutput.GRFs.R(1:pred_006_td_m_to,1));
pred_004_td_m_hImp = trapz(pred_004_td_m.optimumOutput.timeNodes(1:pred_004_td_m_to),pred_004_td_m.optimumOutput.GRFs.R(1:pred_004_td_m_to,1));
pred_002_td_m_hImp = trapz(pred_002_td_m.optimumOutput.timeNodes(1:pred_002_td_m_to),pred_002_td_m.optimumOutput.GRFs.R(1:pred_002_td_m_to,1));

pred_006_ik_p_hImp = trapz(pred_006_ik_p.optimumOutput.timeNodes(1:pred_006_ik_p_to),pred_006_ik_p.optimumOutput.GRFs.R(1:pred_006_ik_p_to,1));
pred_004_ik_p_hImp = trapz(pred_004_ik_p.optimumOutput.timeNodes(1:pred_004_ik_p_to),pred_004_ik_p.optimumOutput.GRFs.R(1:pred_004_ik_p_to,1));
pred_002_ik_p_hImp = trapz(pred_002_ik_p.optimumOutput.timeNodes(1:pred_002_ik_p_to),pred_002_ik_p.optimumOutput.GRFs.R(1:pred_002_ik_p_to,1));
pred_006_ik_m_hImp = trapz(pred_006_ik_m.optimumOutput.timeNodes(1:pred_006_ik_m_to),pred_006_ik_m.optimumOutput.GRFs.R(1:pred_006_ik_m_to,1));
pred_004_ik_m_hImp = trapz(pred_004_ik_m.optimumOutput.timeNodes(1:pred_004_ik_m_to),pred_004_ik_m.optimumOutput.GRFs.R(1:pred_004_ik_m_to,1));
pred_002_ik_m_hImp = trapz(pred_002_ik_m.optimumOutput.timeNodes(1:pred_002_ik_m_to),pred_002_ik_m.optimumOutput.GRFs.R(1:pred_002_ik_m_to,1));

% Braking Impulse
[pred_hImp_brake,pred_hImp_frame] = min(cumtrapz(pred.optimumOutput.timeNodes(1:pred_to),pred.optimumOutput.GRFs.R(1:pred_to,1)));

[pred_006_td_p_hImp_brake,pred_006_td_p_hImp_frame] = min(cumtrapz(pred_006_td_p.optimumOutput.timeNodes(1:pred_006_td_p_to),pred_006_td_p.optimumOutput.GRFs.R(1:pred_006_td_p_to,1)));
[pred_004_td_p_hImp_brake,pred_004_td_p_hImp_frame] = min(cumtrapz(pred_004_td_p.optimumOutput.timeNodes(1:pred_004_td_p_to),pred_004_td_p.optimumOutput.GRFs.R(1:pred_004_td_p_to,1)));
[pred_002_td_p_hImp_brake,pred_002_td_p_hImp_frame] = min(cumtrapz(pred_002_td_p.optimumOutput.timeNodes(1:pred_002_td_p_to),pred_002_td_p.optimumOutput.GRFs.R(1:pred_002_td_p_to,1)));
[pred_006_td_m_hImp_brake,pred_006_td_m_hImp_frame] = min(cumtrapz(pred_006_td_m.optimumOutput.timeNodes(1:pred_006_td_m_to),pred_006_td_m.optimumOutput.GRFs.R(1:pred_006_td_m_to,1)));
[pred_004_td_m_hImp_brake,pred_004_td_m_hImp_frame] = min(cumtrapz(pred_004_td_m.optimumOutput.timeNodes(1:pred_004_td_m_to),pred_004_td_m.optimumOutput.GRFs.R(1:pred_004_td_m_to,1)));
[pred_002_td_m_hImp_brake,pred_002_td_m_hImp_frame] = min(cumtrapz(pred_002_td_m.optimumOutput.timeNodes(1:pred_002_td_m_to),pred_002_td_m.optimumOutput.GRFs.R(1:pred_002_td_m_to,1)));

[pred_006_ik_p_hImp_brake,pred_006_ik_p_hImp_frame] = min(cumtrapz(pred_006_ik_p.optimumOutput.timeNodes(1:pred_006_ik_p_to),pred_006_ik_p.optimumOutput.GRFs.R(1:pred_006_ik_p_to,1)));
[pred_004_ik_p_hImp_brake,pred_004_ik_p_hImp_frame] = min(cumtrapz(pred_004_ik_p.optimumOutput.timeNodes(1:pred_004_ik_p_to),pred_004_ik_p.optimumOutput.GRFs.R(1:pred_004_ik_p_to,1)));
[pred_002_ik_p_hImp_brake,pred_002_ik_p_hImp_frame] = min(cumtrapz(pred_002_ik_p.optimumOutput.timeNodes(1:pred_002_ik_p_to),pred_002_ik_p.optimumOutput.GRFs.R(1:pred_002_ik_p_to,1)));
[pred_006_ik_m_hImp_brake,pred_006_ik_m_hImp_frame] = min(cumtrapz(pred_006_ik_m.optimumOutput.timeNodes(1:pred_006_ik_m_to),pred_006_ik_m.optimumOutput.GRFs.R(1:pred_006_ik_m_to,1)));
[pred_004_ik_m_hImp_brake,pred_004_ik_m_hImp_frame] = min(cumtrapz(pred_004_ik_m.optimumOutput.timeNodes(1:pred_004_ik_m_to),pred_004_ik_m.optimumOutput.GRFs.R(1:pred_004_ik_m_to,1)));
[pred_002_ik_m_hImp_brake,pred_002_ik_m_hImp_frame] = min(cumtrapz(pred_002_ik_m.optimumOutput.timeNodes(1:pred_002_ik_m_to),pred_002_ik_m.optimumOutput.GRFs.R(1:pred_002_ik_m_to,1)));


% Percentage time spent braking in contact
pred_hImp_brake_time = ((pred.optimumOutput.timeNodes(pred_hImp_frame)-pred.optimumOutput.timeNodes(1))/pred_ct)*100;

% Propulsive Impulse
pred_hImp_prop = pred_hImp - pred_hImp_brake;

pred_006_td_p_hImp_prop = pred_006_td_p_hImp - pred_006_td_p_hImp_brake;
pred_004_td_p_hImp_prop = pred_004_td_p_hImp - pred_004_td_p_hImp_brake;
pred_002_td_p_hImp_prop = pred_002_td_p_hImp - pred_002_td_p_hImp_brake;
pred_006_td_m_hImp_prop = pred_006_td_m_hImp - pred_006_td_m_hImp_brake;
pred_004_td_m_hImp_prop = pred_004_td_m_hImp - pred_004_td_m_hImp_brake;
pred_002_td_m_hImp_prop = pred_002_td_m_hImp - pred_002_td_m_hImp_brake;

pred_006_ik_p_hImp_prop = pred_006_ik_p_hImp - pred_006_ik_p_hImp_brake;
pred_004_ik_p_hImp_prop = pred_004_ik_p_hImp - pred_004_ik_p_hImp_brake;
pred_002_ik_p_hImp_prop = pred_002_ik_p_hImp - pred_002_ik_p_hImp_brake;
pred_006_ik_m_hImp_prop = pred_006_ik_m_hImp - pred_006_ik_m_hImp_brake;
pred_004_ik_m_hImp_prop = pred_004_ik_m_hImp - pred_004_ik_m_hImp_brake;
pred_002_ik_m_hImp_prop = pred_002_ik_m_hImp - pred_002_ik_m_hImp_brake;


% Horizontal touchdown distance
pred_td = pred.optimumOutput.modelCOM.r_toes_pos(1,1) - pred.optimumOutput.modelCOM.pos(1,1);

pred_006_td_p_td = pred_006_td_p.optimumOutput.modelCOM.r_toes_pos(1,1) - pred_006_td_p.optimumOutput.modelCOM.pos(1,1);
pred_004_td_p_td = pred_004_td_p.optimumOutput.modelCOM.r_toes_pos(1,1) - pred_004_td_p.optimumOutput.modelCOM.pos(1,1);
pred_002_td_p_td = pred_002_td_p.optimumOutput.modelCOM.r_toes_pos(1,1) - pred_002_td_p.optimumOutput.modelCOM.pos(1,1);
pred_006_td_m_td = pred_006_td_m.optimumOutput.modelCOM.r_toes_pos(1,1) - pred_006_td_m.optimumOutput.modelCOM.pos(1,1);
pred_004_td_m_td = pred_004_td_m.optimumOutput.modelCOM.r_toes_pos(1,1) - pred_004_td_m.optimumOutput.modelCOM.pos(1,1);
pred_002_td_m_td = pred_002_td_m.optimumOutput.modelCOM.r_toes_pos(1,1) - pred_002_td_m.optimumOutput.modelCOM.pos(1,1);

% Inter-knee touchdown distance
pred_ik = pred.optimumOutput.modelCOM.r_knee_pos(1,1) - pred.optimumOutput.modelCOM.l_knee_pos(1,1);

pred_006_ik_p_ik = pred_006_ik_p.optimumOutput.modelCOM.r_knee_pos(1,1) - pred_006_ik_p.optimumOutput.modelCOM.l_knee_pos(1,1);
pred_004_ik_p_ik = pred_004_ik_p.optimumOutput.modelCOM.r_knee_pos(1,1) - pred_004_ik_p.optimumOutput.modelCOM.l_knee_pos(1,1);
pred_002_ik_p_ik = pred_002_ik_p.optimumOutput.modelCOM.r_knee_pos(1,1) - pred_002_ik_p.optimumOutput.modelCOM.l_knee_pos(1,1);
pred_006_ik_m_ik = pred_006_ik_m.optimumOutput.modelCOM.r_knee_pos(1,1) - pred_006_ik_m.optimumOutput.modelCOM.l_knee_pos(1,1);
pred_004_ik_m_ik = pred_004_ik_m.optimumOutput.modelCOM.r_knee_pos(1,1) - pred_004_ik_m.optimumOutput.modelCOM.l_knee_pos(1,1);
pred_002_ik_m_ik = pred_002_ik_m.optimumOutput.modelCOM.r_knee_pos(1,1) - pred_002_ik_m.optimumOutput.modelCOM.l_knee_pos(1,1);



