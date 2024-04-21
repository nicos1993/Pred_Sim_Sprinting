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

createStride_v4(pred_15,'kinematics_Nominal.mot','activations_Nominal.sto',5);

createStride_v4(pred_15_006_td_p,'kinematics_HTD_plus_6.mot','activations_HTD_plus_6.sto',5);
createStride_v4(pred_15_006_td_m,'kinematics_HTD_minus_6.mot','activations_HTD_minus_6.sto',5);

createStride_v4(pred_15_006_ik_p,'kinematics_IKTD_plus_6.mot','activations_IKTD_plus_6.sto',5);
createStride_v4(pred_15_006_ik_m,'kinematics_IKTD_minus_6.mot','activations_IKTD_minus_6.sto',5);



