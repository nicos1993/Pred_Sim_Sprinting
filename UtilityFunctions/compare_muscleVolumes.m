% Script to compare the absolute and relative muscle volumes of muscle
% volumes in Miller et al. (2020) study

clc;
clear all;

% OpenSim muscle names
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

% Load OpenSim muscle model parameters in order matching muscleNames
%   Maximum Isometric Force - N
%   Optimum Fibre Length - m
%   Tendon Slack Length - m
%   Optimal Pennation Angle - rad
%   Maximum Contract Velocity - x10 Optimum Fibre Length
modelParameters = load('volume_comparison_muscleModelParameters.mat');

% Convert necessary length parameters into cm
modelParameters.MTparameters_m(2,:) = modelParameters.MTparameters_m(2,:) .* 100;

% Specific tension parameter
specificTension = 60; % N/cm^2

% Determine relevant individual muscle volumes - cm^3
for i = 1:43 % Excluding the back muscles from comparison
    model_volumes(i) = (modelParameters.MTparameters_m(1,i)*modelParameters.MTparameters_m(2,i))/(cos(modelParameters.MTparameters_m(4,i))*specificTension);
end

% Mass of sprinter
mass = 72.2;

% Relative volume of individual muscles
model_vol_iliopsoas = (model_volumes(23)+model_volumes(24)) / mass;
model_vol_sartorious = (model_volumes(11)) / mass;
model_vol_tfl = (model_volumes(17)) / mass;
model_vol_addmag = (model_volumes(14)+model_volumes(15)+model_volumes(16)) / mass;
model_vol_gracilis = (model_volumes(19)) / mass;
model_vol_glutMax = (model_volumes(20)+model_volumes(21)+model_volumes(22)) / mass;
model_vol_glutMed = (model_volumes(1)+model_volumes(2)+model_volumes(3)) / mass;
model_vol_glutMin = (model_volumes(4)+model_volumes(5)+model_volumes(6)) / mass;
model_vol_rf = (model_volumes(28)) / mass;
model_vol_vl = (model_volumes(31)) / mass;
model_vol_vm = (model_volumes(29)) / mass; 
model_vol_vi = (model_volumes(30)) / mass;
model_vol_sm = (model_volumes(7)) / mass;
model_vol_st = (model_volumes(8)) / mass;
model_vol_bflh = (model_volumes(9)) / mass;
model_vol_bfsh = (model_volumes(10)) / mass;
model_vol_pop = 0; % No Popliteus in model
model_vol_lg = (model_volumes(33)) / mass;
model_vol_mg = (model_volumes(32)) / mass;
model_vol_sol = (model_volumes(34)) / mass;
model_vol_antSha = (model_volumes(38)+model_volumes(42)+model_volumes(43)) / mass;
model_vol_latSha = (model_volumes(39)+model_volumes(40)) / mass;
model_vol_deepSha = 0; % This was not calculated as is also the combination of Plantaris which is not in the OpenSim model

% Total relative volume
model_vol_total = model_vol_iliopsoas+model_vol_sartorious+model_vol_tfl+model_vol_addmag+model_vol_gracilis+model_vol_glutMax+model_vol_glutMed+model_vol_glutMin+...
    +model_vol_rf+model_vol_vl+model_vol_vm+model_vol_vi+model_vol_sm+model_vol_st+model_vol_bflh+model_vol_bfsh+model_vol_lg+model_vol_mg+model_vol_sol...
    +model_vol_antSha+model_vol_latSha;

% Values from elite sprinters in Miller et al. (2020) - Table 2

% Relative volume of individual muscles
exp_vol_iliopsoas = 8.18;
exp_vol_sartorious = 3.56;
exp_vol_tfl = 1.56;
exp_vol_addmag = 12.31;
exp_vol_gracilis = 2.10;
exp_vol_glutMax = 20.75;
exp_vol_glutMed = 5.01;
exp_vol_glutMin = 2.22;
exp_vol_rf = 5.53;
exp_vol_vl = 13.07;
exp_vol_vm = 11.17; 
exp_vol_vi = 7.53;
exp_vol_sm = 4.16;
exp_vol_st = 5.20;
exp_vol_bflh = 3.96;
exp_vol_bfsh = 1.94;
exp_vol_pop = 0;
exp_vol_lg = 2.36;
exp_vol_mg = 3.50;
exp_vol_sol = 7.05;
exp_vol_antSha = 3.48;
exp_vol_latSha = 1.69;
exp_vol_deepSha = 0;

% Total relative volume
exp_vol_total = exp_vol_iliopsoas+exp_vol_sartorious+exp_vol_tfl+exp_vol_addmag+exp_vol_gracilis+exp_vol_glutMax+exp_vol_glutMed+exp_vol_glutMin+...
    +exp_vol_rf+exp_vol_vl+exp_vol_vm+exp_vol_vi+exp_vol_sm+exp_vol_st+exp_vol_bflh+exp_vol_bfsh+exp_vol_lg+exp_vol_mg+exp_vol_sol...
    +exp_vol_antSha+exp_vol_latSha;



