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

createStride(pred,'kinematics_Nominal.mot','activations_Nominal.sto',5);

createStride(pred_006_td_p,'kinematics_HTD_Plus_6.mot','activations_HTD_Plus_6.sto',5);
createStride(pred_006_td_m,'kinematics_HTD_Minus_6.mot','activations_HTD_Minus_6.sto',5);

createStride(pred_004_td_p,'kinematics_HTD_Plus_4.mot','activations_HTD_Plus_4.sto',5);
createStride(pred_004_td_m,'kinematics_HTD_Minus_4.mot','activations_HTD_Minus_4.sto',5);

createStride(pred_002_td_p,'kinematics_HTD_Plus_2.mot','activations_HTD_Plus_2.sto',5);
createStride(pred_002_td_m,'kinematics_HTD_Minus_2.mot','activations_HTD_Minus_2.sto',5);

createStride(pred_006_ik_p,'kinematics_IKTD_Plus_6.mot','activations_IKTD_Plus_6.sto',5);
createStride(pred_006_ik_m,'kinematics_IKTD_Minus_6.mot','activations_IKTD_Minus_6.sto',5);

createStride(pred_004_ik_p,'kinematics_IKTD_Plus_4.mot','activations_IKTD_Plus_4.sto',5);
createStride(pred_004_ik_m,'kinematics_IKTD_Minus_4.mot','activations_IKTD_Minus_4.sto',5);

createStride(pred_002_ik_p,'kinematics_IKTD_Plus_2.mot','activations_IKTD_Plus_2.sto',5);
createStride(pred_002_ik_m,'kinematics_IKTD_Minus_2.mot','activations_IKTD_Minus_2.sto',5);



