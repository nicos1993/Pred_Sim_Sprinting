function [] = predictiveSprintingSimulations_v5()

clc

import casadi.*


%% Define Options structure
Options.solver        = 'mumps';
Options.tol           = 1;        % 1=1e-5, else=1e-6
Options.derivatives   = 'AD';     % Use AD
Options.N             = 50;       % Number of mesh intervals
Options.prevSol       = 'Y';      % Use prev solution as initial guess;
                                  % Interpolate if necessary; initial guess
                                  % may be adjusted if falls on or off
                                  % passed bounds
Options.MTP_stiff     = 65;       % MTP Spring Stiffness (Nm/rad)
Options.timePercent   = 0.15;     % Reduce overall time by % set

if Options.prevSol == 'Y'
    prevSol = load('IC_pred_Sprinting_optimum_p02_maxVel_01_25-October-2023__23-00-52___Nominal_CONS_BS_PUSH_FRAC.mat');
else
    prevSol = [];
end

% Key examples: _Nominal, _HTD_Plus_1, _IKTD_Minus_6
simulation_type = '_Nominal';

file_ext = checkSimulationType(simulation_type);

% Path of main folder repository
pathmain = pwd; % Print Working Directory


%% Define folder path to store results
pathResults = [pathmain, '\MainFunctions\PredictiveResults\'];
addpath(genpath(pathResults));


%% Load optimised contact model parameters & non-optimised frictional parameters
contPrms = load('Sph_Plane_simultOptContPrms_Fmax_2_Vmax_12.mat');
mu_s = 0.95;
mu_d = 0.3;
mu_v = 0.3;
tv   = 0.001;
contPrms_nsc = [contPrms.simultOptContPrms; mu_s; mu_d; mu_v; tv];


%% Load External Function
pathExternalFuncs = [pathmain, '\ExternalFunctions\'];

% Change directory to load functions
cd(pathExternalFuncs);

% Implicit inverse dynamics, both feet, air drag
% Input: positions,velocities,accelerations,contact parameters
% Output: residuals & joint moments,contact GRFs,plus additional outputs
%         (see below for indices)
F_cont_v21 = external('F_cont_v21','Spr_Imp_GRFs_ownCont_V21.dll');

F_cont_ana21 = external('F_cont_ana21','Spr_Imp_GRFs_ownCont_V21an.dll');

% Change directory back to main directory
cd(pathmain);


%% External function additional output indices
outInd.resids_moments = 1:37;
outInd.r_contGRF      = 38:58;
outInd.l_contGRF      = 59:79;
outInd.posCOM         = 80:82;
outInd.velCOM         = 83:85;
outInd.r_calc_pos     = 86:88; % CoM Position in ground
outInd.r_calc_vel     = 89:91; % CoM Velocity in ground
outInd.l_calc_pos     = 92:94;
outInd.l_calc_vel     = 95:97; 
outInd.r_toes_pos     = 98:100;
outInd.r_toes_vel     = 101:103;
outInd.l_toes_pos     = 104:106;
outInd.l_toes_vel     = 107:109;
outInd.calc_mag            = 110; % Resultant distances between body Frames
outInd.toes_mag            = 111;
outInd.tibia_mag           = 112;
outInd.radius_mag          = 113;
outInd.hand_mag            = 114;
outInd.torso_r_radius_mag  = 115;
outInd.torso_l_radius_mag  = 116;
outInd.torso_r_hand_mag    = 117;
outInd.torso_l_hand_mag    = 118;
outInd.tibia_l_calc_r_mag  = 119;
outInd.tibia_r_calc_l_mag  = 120;
outInd.tibia_l_toes_r_mag  = 121;
outInd.tibia_r_toes_l_mag  = 122;
outInd.hand_r_pelvis_mag   = 123;
outInd.hand_l_pelvis_mag   = 124;
outInd.hand_r_femur_r_mag  = 125;
outInd.hand_l_femur_l_mag  = 126;
outInd.pelvis_radius_r_mag = 127;
outInd.pelvis_radius_l_mag = 128;
outInd.femur_r_XYZ         = 129:131; % Angles relative to global frame
outInd.femur_l_XYZ         = 132:134;
outInd.tibia_r_XYZ         = 135:137;
outInd.tibia_l_XYZ         = 138:140;
outInd.torso_XYZ           = 141:143;

outInd.knee_r_pos_XYZ_F      = 144:146; % These are the specific indices for the first external function
outInd.knee_l_pos_XYZ_F      = 147:149;

outInd.femur_r_H_COM       = 144:146; % Segment Ang Mom about entire COM in global frame
outInd.femur_l_H_COM       = 147:149; % DO NOT USE THESE VARIABLES AS THEY ARE NOT CALCULATED CORRECTLY
outInd.tibia_r_H_COM       = 150:152;
outInd.tibia_l_H_COM       = 153:155;
outInd.talus_r_H_COM       = 156:158;
outInd.talus_l_H_COM       = 159:161;
outInd.calcn_r_H_COM       = 162:164;
outInd.calcn_l_H_COM       = 165:167;
outInd.toes_r_H_COM        = 168:170;
outInd.toes_l_H_COM        = 171:173;
outInd.humerus_r_H_COM     = 174:176;
outInd.humerus_l_H_COM     = 177:179;
outInd.radius_r_H_COM      = 180:182;
outInd.radius_l_H_COM      = 183:185;
outInd.ulna_r_H_COM        = 186:188;
outInd.ulna_l_H_COM        = 189:191;
outInd.hand_r_H_COM        = 192:194;
outInd.hand_l_H_COM        = 195:197;
outInd.pelvis_H_COM        = 198:200;
outInd.torso_H_COM         = 201:203;

outInd.knee_r_pos_XYZ      = 204:206;
outInd.knee_l_pos_XYZ      = 207:209;
outInd.r_sph_1_G_XYZ       = 210:212;
outInd.r_sph_2_G_XYZ       = 213:215;
outInd.r_sph_3_G_XYZ       = 216:218;
outInd.r_sph_4_G_XYZ       = 219:221;
outInd.r_sph_5_G_XYZ       = 222:224;
outInd.r_sph_6_G_XYZ       = 225:227;
outInd.r_sph_7_G_XYZ       = 228:230;
outInd.l_sph_1_G_XYZ       = 231:233;
outInd.l_sph_2_G_XYZ       = 234:236;
outInd.l_sph_3_G_XYZ       = 237:239;
outInd.l_sph_4_G_XYZ       = 240:242;
outInd.l_sph_5_G_XYZ       = 243:245;
outInd.l_sph_6_G_XYZ       = 246:248;
outInd.l_sph_7_G_XYZ       = 249:251;


%% Joint Indices
% These are simply the order in the model file;
% They do not necessarily match Simbody ordering; see .cpp files of
% external functions
jointi.pelvis.tilt    = 1;
jointi.pelvis.list    = 2;
jointi.pelvis.rot     = 3;
jointi.pelvis.tx      = 4;
jointi.pelvis.ty      = 5;
jointi.pelvis.tz      = 6;
jointi.hip_flex.r     = 7;
jointi.hip_add.r      = 8;
jointi.hip_rot.r      = 9;
jointi.knee.r         = 10;
jointi.ankle.r        = 11;
jointi.subt.r         = 12;
jointi.mtp.r          = 13;
jointi.hip_flex.l     = 14;
jointi.hip_add.l      = 15;
jointi.hip_rot.l      = 16;
jointi.knee.l         = 17;
jointi.ankle.l        = 18;
jointi.subt.l         = 19;
jointi.mtp.l          = 20;
jointi.trunk.ext      = 21;
jointi.trunk.ben      = 22;
jointi.trunk.rot      = 23;
jointi.sh_flex.r      = 24;
jointi.sh_add.r       = 25;
jointi.sh_rot.r       = 26;
jointi.elb.r          = 27;
jointi.pro.r          = 28;
jointi.wri_flex.r     = 29;
jointi.wri_dev.r      = 30;
jointi.sh_flex.l      = 31;
jointi.sh_add.l       = 32;
jointi.sh_rot.l       = 33;
jointi.elb.l          = 34;
jointi.pro.l          = 35;
jointi.wri_flex.l     = 36;
jointi.wri_dev.l      = 37;

% Vectors of indices for later use
residualsi        = jointi.pelvis.tilt:jointi.wri_dev.l; % all
ground_pelvisi    = jointi.pelvis.tilt:jointi.pelvis.tz; % ground-pelvis
trunki            = jointi.trunk.ext:jointi.trunk.rot;   % trunk
armsi             = jointi.sh_flex.r:jointi.wri_dev.l;   % arms
residuals_noarmsi = jointi.pelvis.tilt:jointi.trunk.rot; % all but arms

% Number of degrees of freedom for later use
nq.all   = length(residualsi);     % all
nq.abs   = length(ground_pelvisi); % ground-pelvis
nq.trunk = length(trunki);         % trunk
nq.arms  = length(armsi);          % arms
nq.leg   = 10; % #joints needed for polynomials


%% Collocation scheme
% Use Lagrange polynomials to approximate the state derivatives at the
% collocation points in each mesh interval. We use d=3 collocation points
% per mesh interval and Radau collocation points from CasADi; 
% and they are the flipped points defined on the interval [0,1];
% shifted Legendre polynomials [-1,1] -> [0,1]
% Can test the roots by doing the following:
% syms var1
% pL1 = legendreP(3,var1)
% pL2 = legendreP(2,var1)
% tauRoots = vpasolve((pL1+pL2)/(1+var1)==0) % NOT-FLIPPED; [-1,1]
% shifted = (tauRoots+1)./2; % y=mx+c transformation
% flipped = 1-shifted;
d = 3; % degree of interpolating polynomial
method = 'radau'; % collocation method
cd([pathmain, '\CollocationScheme\']);
[tau_root,C,D,B] = CollocationScheme(d,method);
D_control = control_extrapolation(tau_root(2:end));
cd(pathmain);


%% Muscle-tendon parameters
% Muscles from one leg and from the back
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

musclesNamesToPrint = {'glut_med1_l','glut_med2_l','glut_med3_l',...
        'glut_min1_l','glut_min2_l','glut_min3_l','semimem_l',...
        'semiten_l','bifemlh_l','bifemsh_l','sar_l','add_long_l',...
        'add_brev_l','add_mag1_l','add_mag2_l','add_mag3_l','tfl_l',...
        'pect_l','grac_l','glut_max1_l','glut_max2_l','glut_max3_l',......
        'iliacus_l','psoas_l','quad_fem_l','gem_l','peri_l',...
        'rect_fem_l','vas_med_l','vas_int_l','vas_lat_l','med_gas_l',...
        'lat_gas_l','soleus_l','tib_post_l','flex_dig_l','flex_hal_l',...
        'tib_ant_l','per_brev_l','per_long_l','per_tert_l','ext_dig_l',...
        'ext_hal_l','ercspn_l','intobl_l','extobl_l'...
            'glut_med1_r','glut_med2_r','glut_med3_r',...
        'glut_min1_r','glut_min2_r','glut_min3_r','semimem_r',...
        'semiten_r','bifemlh_r','bifemsh_r','sar_r','add_long_r',...
        'add_brev_r','add_mag1_r','add_mag2_r','add_mag3_r','tfl_r',...
        'pect_r','grac_r','glut_max1_r','glut_max2_r','glut_max3_r',......
        'iliacus_r','psoas_r','quad_fem_r','gem_r','peri_r',...
        'rect_fem_r','vas_med_r','vas_int_r','vas_lat_r','med_gas_r',...
        'lat_gas_r','soleus_r','tib_post_r','flex_dig_r','flex_hal_r',...
        'tib_ant_r','per_brev_r','per_long_r','per_tert_r','ext_dig_r',...
        'ext_hal_r','ercspn_r','intobl_r','extobl_r'};

% Muscle model path
pathMuscleModel = [pathmain,'\MuscleModel\'];
addpath(genpath(pathMuscleModel)); 

% Muscle indices; for later use
% Exclude the left torso muscles; avoid double counting them
musInd = MuscleIndices(muscleNames(1:end-3));
% Total number of muscles
NMuscle = length(muscleNames(1:end-3))*2;
 
% Extract muscle-tendon properties
osimModelName = 'Scaled_FullBody_HamnerModel_Muscle_withContact.osim';
modelpath = [pathmain,'\OpenSimModel\'];
addpath(genpath(modelpath));
 
muscProperties = extractMuscProperties([modelpath osimModelName],muscleNames(1:end-3));

% Increase Maximal Isometric by factor of 2
muscleFScale = 2.0; 
muscProperties(1,:) = muscProperties(1,:)*muscleFScale;

% Modify Muscle 25 (quad_fem_r) tendon slack length - large passive forces
muscProperties(2,25) = muscProperties(2,25) + muscProperties(2,25)*0.1;

% Max Shortening Velocity Multiplier
vMaxMult = 12;

MTparameters_m = [muscProperties(:,musInd),muscProperties(:,musInd)];  

totalFmax = sum(MTparameters_m(1,:));
indFmax   = MTparameters_m(1,:);
 
% Optimal muscle fibre length - oMFL
m_oMFL = SX.sym('m_oMFL',NMuscle);

% Tendon slack length - TSL
m_TSL = SX.sym('m_TSL',NMuscle);

% Max shortening velocity - 12 times oMFl - vmax
m_vmax = SX.sym('m_vmax',NMuscle); %12.*m_oMFl;

% Polynomials path
pathpolynomial = [pathmain,'\Polynomials\'];
addpath(genpath(pathpolynomial));
% Indices of the muscles actuating the different joints; for later use
tl = load('muscle_spanning_joint_INFO_subject9.mat');
% mai; indices for dM; start with left followed by right
[~,mai] = MomentArmIndices(muscleNames(1:end-3),tl.muscle_spanning_joint_INFO(1:end-3,:));
 
% By default, the tendon stiffness is 35 and the shift is 0.
aTendon = 35*ones(NMuscle,1);
shift = zeros(NMuscle,1);

% Dummy tensions variable; for usage with the Hill function
% Serves no real purpose for my scenario
tensions = ones(NMuscle,1);


%% Load & process experimental data
pathExpData = [pathmain, '\MainFunctions\ExperimentalData\'];
addpath(genpath(pathExpData));

% Model DOFs
joints = {'pelvis_tilt','pelvis_list','pelvis_rotation','pelvis_tx',...
 'pelvis_ty','pelvis_tz','hip_flexion_r','hip_adduction_r','hip_rotation_r',...
 'knee_angle_r','ankle_angle_r','subtalar_angle_r','mtp_angle_r','hip_flexion_l',...
 'hip_adduction_l','hip_rotation_l','knee_angle_l','ankle_angle_l','subtalar_angle_l','mtp_angle_l',...
 'lumbar_extension','lumbar_bending','lumbar_rotation','arm_flex_r','arm_add_r',...
 'arm_rot_r','elbow_flex_r','pro_sup_r','wrist_flex_r','wrist_dev_r','arm_flex_l',...
 'arm_add_l','arm_rot_l','elbow_flex_l','pro_sup_l','wrist_flex_l','wrist_dev_l'};

% Positional state variables - IK output
statesFname1 = 'p02_maxVel_01.mot';

% Kinematics sampling frequency
kinSF = 250;

[statesF1,timeGrid1,timeNodes1,h1,stateNames] = processIKoutput(statesFname1,kinSF,Options,tau_root,d,nq,modelpath,osimModelName,[pathExpData 'IK_Splined\']);


%% Create variable bounds & scaling factors
% Create scaled continuous design variables & scaling factors
% All variables will lie between [-1,1]
[bounds_sc1,scaling1] = createScaledBounds(statesF1,nq,Options,d,NMuscle,jointi);

% Muscle model parameters
oMFL_2_nsc = MTparameters_m(2,1:NMuscle)';
TSL_2_nsc  = MTparameters_m(3,1:NMuscle)';

% Original total time
orig_totalTime = timeGrid1(end) - timeGrid1(1);

percent_time = orig_totalTime*Options.timePercent;

% To prevent boundaries getting reached
totalTime_bounds_lb_nsc = orig_totalTime - percent_time;
totalTime_bounds_ub_nsc = orig_totalTime + percent_time;

scaling1.totalTime = [totalTime_bounds_lb_nsc 1.0; totalTime_bounds_ub_nsc 1.0]\[-1;1];

bounds_sc1.totalTime.lower = scaling1.totalTime(1)*totalTime_bounds_lb_nsc + scaling1.totalTime(2);
bounds_sc1.totalTime.upper = scaling1.totalTime(1)*totalTime_bounds_ub_nsc + scaling1.totalTime(2);


%% Create initial guess data
guess1 = createGuess(scaling1,statesF1,nq,NMuscle,Options,d,bounds_sc1,prevSol,timeGrid1,timeNodes1); 

guess1.totalTime = orig_totalTime.*scaling1.totalTime(1)+scaling1.totalTime(2);
if Options.prevSol == 'Y'
    guess1.totalTime = prevSol.optimumOutput.optVars_nsc.totalTime.*scaling1.totalTime(1)+scaling1.totalTime(2);
end


%% Determine ranges for scaling objective function terms

% Acceptable percentage tolerance of range of variables to scale objective
% function terms
percent_tol = 0.1;

obj_range1 = calcObjRange(nq,statesF1,percent_tol,scaling1,prevSol);


%% Load CasADi function definitions

costFunctions1 = loadCostFunctions(obj_range1,nq,NMuscle,totalFmax,indFmax);

[f_lMT_vMT_dM,f_forceEquilibrium_FtildeState_all_tendon_M,calcMoms] = loadMuscleModelFunctions(musInd,NMuscle,m_oMFL,m_TSL,m_vmax,pathpolynomial,nq,MTparameters_m);


%% Cost function weights
wJ(1)  = 0.00000; % Pelvis orientations
wJ(2)  = 0.00000; % Pelvis translations excluding horizontal
wJ(3)  = 0.00000; % Lower-limb angles
wJ(4)  = 0.00000; % Upper-limb angles & trunk
wJ(5)  = 0.05; % Accelerations
wJ(6)  = 0.1; % Muscle activations - was 0.1
wJ(7)  = 0.01; % Derivative of muscle activations
wJ(8)  = 0.0; % Normalized tendon forces
wJ(9)  = 0.01; % Derivative of normalized tendon forces
wJ(10) = 0.01; % Reserve actuators
wJ(11) = 0.1; % Arm controls
wJ(12) = 10.0; % Average speed


%% Formulate NLPs

% Define final time as a scaled free variable
finalTime = MX.sym('finalTime',1);

% Unscale final time
finalTime_nsc = (finalTime-scaling1.totalTime(2))/scaling1.totalTime(1);

% Define time interval - 'h'
pred_h = finalTime_nsc/Options.N;

[w1,w01,lbw1,ubw1,J1,g1,lbg1,ubg1,g_names1,change_disp,sum_v_GRF,vertImp] = buildNLP(guess1,bounds_sc1,scaling1,nq,Options,pred_h,F_cont_v21,statesF1,contPrms_nsc,C,D,B,wJ,costFunctions1,oMFL_2_nsc,TSL_2_nsc,NMuscle,vMaxMult,outInd,prevSol,finalTime_nsc,D_control);

% Concatenate the NLP and include final time variable
w   = {w1{:}, finalTime};
w0  = [w01 guess1.totalTime];
lbw = [lbw1 bounds_sc1.totalTime.lower];
ubw = [ubw1 bounds_sc1.totalTime.upper];
J   = J1 - (wJ(12).*(((change_disp)./finalTime_nsc)));
g   = {g1{:}};
lbg = [lbg1];
ubg = [ubg1];

% Intervals to be added when reorganising variables
int1 = 0;


%% Solve problem
% Create an NLP solver
prob = struct('f',J,'x',vertcat(w{:}),'g',vertcat(g{:}));

options.ipopt.hessian_approximation = 'limited-memory';
options.ipopt.mu_strategy = 'adaptive';
options.ipopt.linear_solver = Options.solver;

if Options.tol==1
    options.ipopt.tol=1e-5;
else
    options.ipopt.tol=1e-6;
end

options.ipopt.max_iter = 50e3; % default = 3e3

options.ipopt.print_level=5;

% Explain what these do
% options.ipopt.bound_frac = 1e-16;%0.01;
% options.ipopt.bound_push = 1e-16;%0.01;
% options.ipopt.slack_bound_frac = 1e-16;%0.01;
% options.ipopt.slack_bound_push = 1e-16;%0.01;

solver = nlpsol('solver', 'ipopt', prob, options);

% Solve the NLP
sol = solver('x0',w0,'lbx',lbw','ubx',ubw','lbg',lbg,'ubg',ubg);

w_opt = full(sol.x);
stats = solver.stats;
lam_x_opt = full(sol.lam_x);
lam_g_opt = full(sol.lam_g);



%% Reorganise optimised variables

% Structures containing scaled and non-scaled optimised variables
[optVars_sc1,optVars_nsc1] = sortVariables_muscle_multipleTrials(w_opt,nq,Options,scaling1,d,int1,NMuscle,D_control);

% Calculate new timeGrid and new timeNodes
pred_h_number = optVars_nsc1.totalTime/Options.N;

pred_timeNodes = linspace(timeGrid1(1),timeGrid1(1)+optVars_nsc1.totalTime,Options.N+1);

% Create timeline which includes mesh points and collocation points
for kk = 1:Options.N
    for kkk = 1:d+1
        pred_timeGrid(kk,kkk) = pred_timeNodes(kk) + pred_h_number*tau_root(kkk);
    end
end

% Convert from Matrix to column Vector
pred_timeGrid = pred_timeGrid';
pred_timeGrid = pred_timeGrid(:);

% Create controls time line
for kk = 1:Options.N
    for kkk = 1:d
        pred_timeGrid_con_dummy(kk,kkk) = pred_timeNodes(kk) + pred_h_number*tau_root(kkk+1);
    end
end

% Convert from matrix to column vector
pred_timeGrid_con_dummy = pred_timeGrid_con_dummy';
pred_timeGrid_con_dummy = pred_timeGrid_con_dummy(:);

pred_timeGrid_con = [pred_timeNodes(1); pred_timeGrid_con_dummy];

[termsJ1,moments1,muscleValues1,modelCOM1,GRFs1,bodyAngles1,angMom1,GRF_individual1] = calcObjFuncTerms(wJ,B,pred_h_number,Options,nq,optVars_nsc1,d,statesF1,costFunctions1,F_cont_ana21,NMuscle,contPrms_nsc,vMaxMult,oMFL_2_nsc,TSL_2_nsc,prevSol);

% Save outputs
optimumOutput1 = saveOptimumFiles(scaling1,Options,optVars_sc1,optVars_nsc1,pred_timeGrid_con,pred_timeGrid,...
    statesF1,wJ,stats,statesFname1,stateNames,pathResults,outInd,termsJ1,moments1,muscleValues1,modelCOM1,GRFs1,bodyAngles1,angMom1,musclesNamesToPrint,GRF_individual1);



    % Function to appropriately process IK output
    function [kinProcess,timeGrid,timeNodes,h,statesNames] = processIKoutput(statesFname,kinSF,Options,tau_root,d,nq,modelpath,osimModelName,pathExpData)
        
        % Load
        statesFile = readMOT(statesFname);
        
        % Original timeline of trial
        origTime = statesFile.data(:,1);

        % Time discretization
        % Initial time
        ti = origTime(1);
        % Final time
        tf = origTime(end);
        
        % Discretized time at each mesh interval
        timeNodes = linspace(ti,tf,Options.N);
        
        % Time between intervals (in 'real' time)
        h = (tf-ti)/(Options.N);
        
        % Filter cut-off frequency
        cf = 20;
        
        % Filter coefficients
        [b,a] = butter(2,(cf/(kinSF/2)),'low');
        
        % Pad data prior to filtering
        statesP = paddon(statesFile.data(:,2:end));
        
        % Filtered padded states data
        statesFilt = filtfilt(b,a,statesP);
        
        % Remove padding
        statesFilt = paddoff(statesFilt);
        
        % Resample auxiliary states
        %statesReSamp = interp1(origTime,statesFilt,timeSpline,'spline');
        
        % Create timeline which includes mesh points and collocation points
        for i = 1:Options.N
            for ii = 1:d+1
                timeGrid(i,ii) = timeNodes(i) + h*tau_root(ii);
            end
        end

        % Convert from Matrix to column Vector
        timeGrid = timeGrid';
        timeGrid = timeGrid(:);

        % Insert final mesh point time
        %timeGrid(end+1,1) = tf;
        
        for i = 1:nq.all
            pp = spline(origTime,statesFilt(:,i));
            q_aux(:,i) = ppval(pp,timeGrid);
            pp_1 = fnder(pp,1);
            qdot_aux(:,i) = ppval(pp_1,timeGrid);
            pp_2 = fnder(pp,2);
            qddot_aux(:,i) = ppval(pp_2,timeGrid);
        end
        
        % Import OpenSim methods
        import org.opensim.modeling.*

        % Extract info from Model
        osimModel = Model([modelpath osimModelName]);
        osimState = osimModel.initSystem();
        statesArray = osimModel.getStateVariableNames();

        % Extract state variable names from statesArray
        statesNames{1}='time';
        statesIndices = [0:2:(nq.all*2)-1];
        for i = 1:nq.all
            statesNames{i+1} = char(statesArray.get(statesIndices(i)));
        end
        
        % Write coordinates (q) to .mot file
        motData.labels = statesNames;
        motData.data   = [timeGrid q_aux];
        write_motionFile(motData,[pathExpData 'Splined_' num2str(Options.N) '_meshInts_' erase(statesFname,'.mot') '.mot']);

        % Convert angular coordinates deg->rad
        q_aux(:,[1:3,7:end]) = deg2rad(q_aux(:,[1:3,7:end]));
        qdot_aux(:,[1:3,7:end]) = deg2rad(qdot_aux(:,[1:3,7:end]));
        qddot_aux(:,[1:3,7:end]) = deg2rad(qddot_aux(:,[1:3,7:end]));
        
        % Reorder the states for usage with the ID External Function
        for i = 0:nq.all-1
            states_aux(:,i+1*(i+1)) = q_aux(:,i+1);
            states_aux(:,2*(i+1))   = qdot_aux(:,i+1);
        end
        
        kinProcess.q_aux     = q_aux;
        kinProcess.qdot_aux  = qdot_aux;
        kinProcess.qddot_aux = qddot_aux;
        
        kinProcess.states_aux = states_aux;
        
    end

    function [bounds,scaling] = createScaledBounds(statesF,nq,Options,d,NMuscle,jointi)
        
        % Non-scaled lower-bounds for q, qdot & uAcc are minimum experimental value minus 25% minimum
        bounds_nsc.q.lower    = (min(statesF.q_aux) - abs(min(statesF.q_aux))*0.25)';
        bounds_nsc.q.lower([jointi.sh_flex.r,jointi.sh_flex.l]) = deg2rad(-75);
        bounds_nsc.q.lower([jointi.sh_rot.r,jointi.sh_rot.l]) = deg2rad(-15);
        bounds_nsc.qdot.lower = (min(statesF.qdot_aux) - abs(min(statesF.qdot_aux))*0.25)';
        bounds_nsc.uAcc.lower = (min(statesF.qddot_aux) - abs(min(statesF.qddot_aux))*0.25)';
        
        % Symmetrify bounds to ensure endpoint configuration is achievable 
        bounds_nsc.q.lower(jointi.hip_flex.r:jointi.mtp.r) = min([bounds_nsc.q.lower(jointi.hip_flex.r:jointi.mtp.r) bounds_nsc.q.lower(jointi.hip_flex.l:jointi.mtp.l)]')';
        bounds_nsc.q.lower(jointi.hip_flex.l:jointi.mtp.l) = min([bounds_nsc.q.lower(jointi.hip_flex.r:jointi.mtp.r) bounds_nsc.q.lower(jointi.hip_flex.l:jointi.mtp.l)]')';
        bounds_nsc.q.lower(jointi.sh_flex.r:jointi.wri_dev.r) = min([bounds_nsc.q.lower(jointi.sh_flex.r:jointi.wri_dev.r) bounds_nsc.q.lower(jointi.sh_flex.l:jointi.wri_dev.l)]')';
        bounds_nsc.q.lower(jointi.sh_flex.l:jointi.wri_dev.l) = min([bounds_nsc.q.lower(jointi.sh_flex.r:jointi.wri_dev.r) bounds_nsc.q.lower(jointi.sh_flex.l:jointi.wri_dev.l)]')';

        bounds_nsc.qdot.lower(jointi.hip_flex.r:jointi.mtp.r) = min([bounds_nsc.qdot.lower(jointi.hip_flex.r:jointi.mtp.r) bounds_nsc.qdot.lower(jointi.hip_flex.l:jointi.mtp.l)]')';
        bounds_nsc.qdot.lower(jointi.hip_flex.l:jointi.mtp.l) = min([bounds_nsc.qdot.lower(jointi.hip_flex.r:jointi.mtp.r) bounds_nsc.qdot.lower(jointi.hip_flex.l:jointi.mtp.l)]')';
        bounds_nsc.qdot.lower(jointi.sh_flex.r:jointi.wri_dev.r) = min([bounds_nsc.qdot.lower(jointi.sh_flex.r:jointi.wri_dev.r) bounds_nsc.qdot.lower(jointi.sh_flex.l:jointi.wri_dev.l)]')';
        bounds_nsc.qdot.lower(jointi.sh_flex.l:jointi.wri_dev.l) = min([bounds_nsc.qdot.lower(jointi.sh_flex.r:jointi.wri_dev.r) bounds_nsc.qdot.lower(jointi.sh_flex.l:jointi.wri_dev.l)]')';

        bounds_nsc.uAcc.lower(jointi.hip_flex.r:jointi.mtp.r) = min([bounds_nsc.uAcc.lower(jointi.hip_flex.r:jointi.mtp.r) bounds_nsc.uAcc.lower(jointi.hip_flex.l:jointi.mtp.l)]')';
        bounds_nsc.uAcc.lower(jointi.hip_flex.l:jointi.mtp.l) = min([bounds_nsc.uAcc.lower(jointi.hip_flex.r:jointi.mtp.r) bounds_nsc.uAcc.lower(jointi.hip_flex.l:jointi.mtp.l)]')';
        bounds_nsc.uAcc.lower(jointi.sh_flex.r:jointi.wri_dev.r) = min([bounds_nsc.uAcc.lower(jointi.sh_flex.r:jointi.wri_dev.r) bounds_nsc.uAcc.lower(jointi.sh_flex.l:jointi.wri_dev.l)]')';
        bounds_nsc.uAcc.lower(jointi.sh_flex.l:jointi.wri_dev.l) = min([bounds_nsc.uAcc.lower(jointi.sh_flex.r:jointi.wri_dev.r) bounds_nsc.uAcc.lower(jointi.sh_flex.l:jointi.wri_dev.l)]')';
        
        % Non-scaled upper-bounds for q, qdot & uAcc are maximum experimental value plus 25% maximum
        bounds_nsc.q.upper    = (max(statesF.q_aux) + abs(max(statesF.q_aux))*0.25)';
        bounds_nsc.q.upper(4) = 5.0;
        bounds_nsc.q.upper([jointi.sh_flex.r,jointi.sh_flex.l]) = deg2rad(90);
        bounds_nsc.q.upper([jointi.sh_rot.r,jointi.sh_rot.l]) = deg2rad(0); % was 15
        bounds_nsc.q.upper([jointi.pro.r,jointi.pro.l])       = deg2rad(90); % Didn't enforce before
        bounds_nsc.qdot.upper = (max(statesF.qdot_aux) + abs(max(statesF.qdot_aux))*0.25)';
        bounds_nsc.uAcc.upper = (max(statesF.qddot_aux) + abs(max(statesF.qddot_aux))*0.25)';
        
        % Symmetrify bounds to ensure endpoint configuration is achievable 
        bounds_nsc.q.upper(jointi.hip_flex.r:jointi.mtp.r) = max([bounds_nsc.q.upper(jointi.hip_flex.r:jointi.mtp.r) bounds_nsc.q.upper(jointi.hip_flex.l:jointi.mtp.l)]')';
        bounds_nsc.q.upper(jointi.hip_flex.l:jointi.mtp.l) = max([bounds_nsc.q.upper(jointi.hip_flex.r:jointi.mtp.r) bounds_nsc.q.upper(jointi.hip_flex.l:jointi.mtp.l)]')';
        bounds_nsc.q.upper(jointi.sh_flex.r:jointi.wri_dev.r) = max([bounds_nsc.q.upper(jointi.sh_flex.r:jointi.wri_dev.r) bounds_nsc.q.upper(jointi.sh_flex.l:jointi.wri_dev.l)]')';
        bounds_nsc.q.upper(jointi.sh_flex.l:jointi.wri_dev.l) = max([bounds_nsc.q.upper(jointi.sh_flex.r:jointi.wri_dev.r) bounds_nsc.q.upper(jointi.sh_flex.l:jointi.wri_dev.l)]')';

        bounds_nsc.qdot.upper(jointi.hip_flex.r:jointi.mtp.r) = max([bounds_nsc.qdot.upper(jointi.hip_flex.r:jointi.mtp.r) bounds_nsc.qdot.upper(jointi.hip_flex.l:jointi.mtp.l)]')';
        bounds_nsc.qdot.upper(jointi.hip_flex.l:jointi.mtp.l) = max([bounds_nsc.qdot.upper(jointi.hip_flex.r:jointi.mtp.r) bounds_nsc.qdot.upper(jointi.hip_flex.l:jointi.mtp.l)]')';
        bounds_nsc.qdot.upper(jointi.sh_flex.r:jointi.wri_dev.r) = max([bounds_nsc.qdot.upper(jointi.sh_flex.r:jointi.wri_dev.r) bounds_nsc.qdot.upper(jointi.sh_flex.l:jointi.wri_dev.l)]')';
        bounds_nsc.qdot.upper(jointi.sh_flex.l:jointi.wri_dev.l) = max([bounds_nsc.qdot.upper(jointi.sh_flex.r:jointi.wri_dev.r) bounds_nsc.qdot.upper(jointi.sh_flex.l:jointi.wri_dev.l)]')';

        bounds_nsc.uAcc.upper(jointi.hip_flex.r:jointi.mtp.r) = max([bounds_nsc.uAcc.upper(jointi.hip_flex.r:jointi.mtp.r) bounds_nsc.uAcc.upper(jointi.hip_flex.l:jointi.mtp.l)]')';
        bounds_nsc.uAcc.upper(jointi.hip_flex.l:jointi.mtp.l) = max([bounds_nsc.uAcc.upper(jointi.hip_flex.r:jointi.mtp.r) bounds_nsc.uAcc.upper(jointi.hip_flex.l:jointi.mtp.l)]')';
        bounds_nsc.uAcc.upper(jointi.sh_flex.r:jointi.wri_dev.r) = max([bounds_nsc.uAcc.upper(jointi.sh_flex.r:jointi.wri_dev.r) bounds_nsc.uAcc.upper(jointi.sh_flex.l:jointi.wri_dev.l)]')';
        bounds_nsc.uAcc.upper(jointi.sh_flex.l:jointi.wri_dev.l) = max([bounds_nsc.uAcc.upper(jointi.sh_flex.r:jointi.wri_dev.r) bounds_nsc.uAcc.upper(jointi.sh_flex.l:jointi.wri_dev.l)]')';

        % Symmetrify pelvis and trunk coordinates
        bounds_nsc.q.lower(jointi.pelvis.tilt:jointi.pelvis.rot) = max([abs(bounds_nsc.q.lower(jointi.pelvis.tilt:jointi.pelvis.rot)) abs(bounds_nsc.q.upper(jointi.pelvis.tilt:jointi.pelvis.rot))]')' .* -1; 
        bounds_nsc.q.upper(jointi.pelvis.tilt:jointi.pelvis.rot) = -1 .* bounds_nsc.q.lower(jointi.pelvis.tilt:jointi.pelvis.rot);
        bounds_nsc.q.lower(jointi.trunk.ext:jointi.trunk.rot) = max([abs(bounds_nsc.q.lower(jointi.trunk.ext:jointi.trunk.rot)) abs(bounds_nsc.q.upper(jointi.trunk.ext:jointi.trunk.rot))]')' .* -1;
        bounds_nsc.q.upper(jointi.trunk.ext:jointi.trunk.rot) = -1 .* bounds_nsc.q.lower(jointi.trunk.ext:jointi.trunk.rot);
        
        bounds_nsc.qdot.lower(jointi.pelvis.tilt:jointi.pelvis.rot) = max([abs(bounds_nsc.qdot.lower(jointi.pelvis.tilt:jointi.pelvis.rot)) abs(bounds_nsc.qdot.upper(jointi.pelvis.tilt:jointi.pelvis.rot))]')' .* -1; 
        bounds_nsc.qdot.upper(jointi.pelvis.tilt:jointi.pelvis.rot) = -1 .* bounds_nsc.qdot.lower(jointi.pelvis.tilt:jointi.pelvis.rot);
        bounds_nsc.qdot.lower(jointi.trunk.ext:jointi.trunk.rot) = max([abs(bounds_nsc.qdot.lower(jointi.trunk.ext:jointi.trunk.rot)) abs(bounds_nsc.qdot.upper(jointi.trunk.ext:jointi.trunk.rot))]')' .* -1;
        bounds_nsc.qdot.upper(jointi.trunk.ext:jointi.trunk.rot) = -1 .* bounds_nsc.qdot.lower(jointi.trunk.ext:jointi.trunk.rot);
        
        bounds_nsc.uAcc.lower(jointi.pelvis.tilt:jointi.pelvis.rot) = max([abs(bounds_nsc.uAcc.lower(jointi.pelvis.tilt:jointi.pelvis.rot)) abs(bounds_nsc.uAcc.upper(jointi.pelvis.tilt:jointi.pelvis.rot))]')' .* -1; 
        bounds_nsc.uAcc.upper(jointi.pelvis.tilt:jointi.pelvis.rot) = -1 .* bounds_nsc.uAcc.lower(jointi.pelvis.tilt:jointi.pelvis.rot);
        bounds_nsc.uAcc.lower(jointi.trunk.ext:jointi.trunk.rot) = max([abs(bounds_nsc.uAcc.lower(jointi.trunk.ext:jointi.trunk.rot)) abs(bounds_nsc.uAcc.upper(jointi.trunk.ext:jointi.trunk.rot))]')' .* -1;
        bounds_nsc.uAcc.upper(jointi.trunk.ext:jointi.trunk.rot) = -1 .* bounds_nsc.uAcc.lower(jointi.trunk.ext:jointi.trunk.rot);
        
        
        % Determine scaling values from lower- & upper-bounds
        scaling.q    = max(abs(bounds_nsc.q.lower),abs(bounds_nsc.q.upper));
        scaling.qdot = max(abs(bounds_nsc.qdot.lower),abs(bounds_nsc.qdot.upper));
        scaling.uAcc = max(abs(bounds_nsc.uAcc.lower),abs(bounds_nsc.uAcc.upper));
        
        % Scale bounds
        bounds.q.lower    = bounds_nsc.q.lower./scaling.q;
        bounds.qdot.lower = bounds_nsc.qdot.lower./scaling.qdot;
        bounds.uAcc.lower = bounds_nsc.uAcc.lower./scaling.uAcc;
        
        bounds.q.upper    = bounds_nsc.q.upper./scaling.q;
        bounds.qdot.upper = bounds_nsc.qdot.upper./scaling.qdot;
        bounds.uAcc.upper = bounds_nsc.uAcc.upper./scaling.uAcc;
  
        % Repeat scaled bounds
        bounds.q.lower    = repmat(bounds.q.lower,1,(d+1)*Options.N);
        bounds.qdot.lower = repmat(bounds.qdot.lower,1,(d+1)*Options.N);
        bounds.uAcc.lower = repmat(bounds.uAcc.lower,1,(d+1)*Options.N);
        
        bounds.q.upper    = repmat(bounds.q.upper,1,(d+1)*Options.N);
        bounds.qdot.upper = repmat(bounds.qdot.upper,1,(d+1)*Options.N);
        bounds.uAcc.upper = repmat(bounds.uAcc.upper,1,(d+1)*Options.N);
        
        % Arm Torques scaling factor
        scaling.uArms = 100; % was 200
        
        % Lower-bounds Arm Excitation controls
        bounds.eArms.lower = -1.*(repmat(1,nq.arms,1));
        
        % Upper-bounds Arm Excitations controls
        bounds.eArms.upper = 1.*(repmat(1,nq.arms,1));
        
        % Lower-bounds Arm Activations states
        bounds.aArms.lower = -1.*(repmat(1,nq.arms,1));
        
        % Upper-bounds Arm Activations states
        bounds.aArms.upper = 1.*(repmat(1,nq.arms,1));
        
        % Lower-bounds Muscle Activations
        bounds_nsc.act.lower = zeros(1,NMuscle);
        
        % Upper-bounds Muscle Activations
        bounds_nsc.act.upper = ones(1,NMuscle);
        
        tact = 0.015;
        tdeact = 0.06;
        
        % Lower-bounds Derivative of Muscle Activations
        bounds_nsc.uActdot.lower = (-1/100*ones(1,NMuscle))./(ones(1,NMuscle)*tdeact);

        % Upper-bounds Derivative of Muscle Activations
        bounds_nsc.uActdot.upper = (1/100*ones(1,NMuscle))./(ones(1,NMuscle)*tact);
        
        % Lower-bounds Tendon Forces
        bounds_nsc.FTtilde.lower = zeros(1,NMuscle);
        
        % Upper-bounds Tendon Forces
        bounds_nsc.FTtilde.upper = 5*ones(1,NMuscle);
        
        % Lower-bounds Derivative of Tendon Forces
        bounds_nsc.dFTtilde.lower = -1*ones(1,NMuscle);
        
        % Upper-bounds Derivative of Tendon Forces
        bounds_nsc.dFTtilde.upper = 1*ones(1,NMuscle);
        
        % Muscle Activations scaling factor
        scaling.act = ones(1,NMuscle);
        
        % Derivative of Muscle Activations scaling factor
        scaling.uActdot = 100*ones(1,NMuscle);
        
        % Tendon Forces scaling factor
        scaling.FTtilde = 5*ones(1,NMuscle);
        
        % Derivative of Tendon Forces scaling factor
        scaling.dFTtilde = 100*ones(1,NMuscle);
        
        % Scale Bounds for additional variables where necessary
        bounds.act.lower = bounds_nsc.act.lower;
        bounds.act.upper = bounds_nsc.act.upper;
        
        bounds.uActdot.lower = bounds_nsc.uActdot.lower;
        bounds.uActdot.upper = bounds_nsc.uActdot.upper;
        
        bounds.FTtilde.lower = bounds_nsc.FTtilde.lower./scaling.FTtilde;
        bounds.FTtilde.upper = bounds_nsc.FTtilde.upper./scaling.FTtilde;
        
        bounds.dFTtilde.lower = bounds_nsc.dFTtilde.lower;
        bounds.dFTtilde.upper = bounds_nsc.dFTtilde.upper;
        
        
        % Lower-limbs & Trunk reserves
        scaling.uReserves = 40.*ones(2,1);
        
        % Lower-bounds lower-limbs and trunk reserves
        bounds.uReserves.lower = (-(scaling.uReserves))./scaling.uReserves;
        
        % Upper-bounds lower-limbs and trunk reserves
        bounds.uReserves.upper = ((scaling.uReserves))./scaling.uReserves;
        
    end

    function guess = createGuess(scaling,statesF,nq,NMuscle,Options,d,bounds_sc,prevOpti,timeGrid,timeNodes)
        
        guess.q        = statesF.q_aux./(scaling.q');

        if ~isempty(prevOpti)
            guess.q = [];
            guess.q = prevOpti.optimumOutput.optVars_nsc.q'./(scaling.q');
        end
        
        [ind_r,ind_c] = find(guess.q(:,jointi.sh_flex.r) < bounds_sc.q.lower(jointi.sh_flex.r,:)'); % Why am I using the same variables? That is because I iteratively looked at each ...
        [ind_r,ind_c] = find(guess.q(:,jointi.sh_flex.r) > bounds_sc.q.upper(jointi.sh_flex.r,:)'); % of them to check if empty, if not the guess is modified to conform to bound
        [ind_r,ind_c] = find(guess.q(:,jointi.sh_flex.l) < bounds_sc.q.lower(jointi.sh_flex.l,:)');
        guess.q(ind_r,jointi.sh_flex.l) = deg2rad(-70)./scaling.q(jointi.sh_flex.l);
        [ind_r,ind_c] = find(guess.q(:,jointi.sh_flex.l) > bounds_sc.q.upper(jointi.sh_flex.l,:)');
        
        [ind_r,ind_c] = find(guess.q(:,jointi.sh_rot.r) < bounds_sc.q.lower(jointi.sh_rot.r,:)');
        [ind_r,ind_c] = find(guess.q(:,jointi.sh_rot.r) > bounds_sc.q.upper(jointi.sh_rot.r,:)');
        guess.q(ind_r,jointi.sh_rot.r) = deg2rad(10)./scaling.q(jointi.sh_rot.r);
        [ind_r,ind_c] = find(guess.q(:,jointi.sh_rot.l) < bounds_sc.q.lower(jointi.sh_rot.l,:)');
        guess.q(ind_r,jointi.sh_rot.l) = deg2rad(-10)./scaling.q(jointi.sh_rot.l);
        [ind_r,ind_c] = find(guess.q(:,jointi.sh_rot.l) > bounds_sc.q.upper(jointi.sh_rot.l,:)');
        
        
        guess.qdot     = statesF.qdot_aux./(scaling.qdot');
        inds_ignore    = [1:4:(Options.N*(d+1))]'; % Indices to ignore
        guess.uAcc     = statesF.qddot_aux./(scaling.uAcc');
        guess.uAcc(inds_ignore,:) = [];
        guess.eArms    = zeros(nq.arms,Options.N*d);

        if ~isempty(prevOpti)
            guess.qdot  = [];
            guess.qdot  = prevOpti.optimumOutput.optVars_nsc.qdot'./(scaling.qdot');
            guess.uAcc  = [];
            guess.uAcc  = prevOpti.optimumOutput.optVars_nsc.uAcc(:,2:end)'./(scaling.uAcc');
            guess.eArms = [];
            guess.eArms = prevOpti.optimumOutput.optVars_nsc.armExcts(:,2:end);
        end
        
        guess.act      = 0.5*ones(NMuscle,(Options.N*(d+1)));
        guess.uActdot  = 0.01*ones(NMuscle,Options.N*d);
        guess.FTtilde  = 0.5*ones(NMuscle,(Options.N*(d+1)));
        guess.dFTtilde = 0.01*ones(NMuscle,Options.N*d);
        
        guess.aArms    = zeros(nq.arms,(Options.N*(d+1)));
        
        guess.uReserves = zeros(2,Options.N*d)./scaling.uReserves;

        if ~isempty(prevOpti)
            guess.act       = [];
            guess.act       = prevOpti.optimumOutput.optVars_nsc.act;
            guess.uActdot   = [];
            guess.uActdot   = prevOpti.optimumOutput.optVars_nsc.uActdot(:,2:end)./scaling.uActdot';
            guess.FTtilde   = [];
            guess.FTtilde   = prevOpti.optimumOutput.optVars_nsc.FTtilde./scaling.FTtilde';
            guess.dFTtilde  = [];
            guess.dFTtilde  = prevOpti.optimumOutput.optVars_nsc.dFTtilde(:,2:end)./scaling.dFTtilde';
            guess.aArms     = [];
            guess.aArms     = prevOpti.optimumOutput.optVars_nsc.armActs;
            guess.uReserves = [];
            guess.uReserves = prevOpti.optimumOutput.optVars_nsc.uReserves(:,2:end)./scaling.uReserves;
        end
               
    end

   function [obj_range] = calcObjRange(nq,statesF,percent_tol,scaling,preOpt)
               
        obj_range.q        = (max(statesF.q_aux) - min(statesF.q_aux))' * percent_tol;
        obj_range.qdot     = (max(statesF.qdot_aux) - min(statesF.qdot_aux))' * percent_tol;
        obj_range.uAcc     = (max(statesF.qddot_aux) - min(statesF.qddot_aux))' * percent_tol;

        obj_range.uAcc = scaling.uAcc;
        
        obj_range.act      = scaling.act;
        obj_range.uActdot  = scaling.uActdot;
        obj_range.FTtilde  = scaling.FTtilde;
        obj_range.dFTtilde = scaling.dFTtilde;
        
        obj_range.uReserves = scaling.uReserves;
        
        obj_range.uArms = scaling.uArms;
        
    end

    function costFunctions = loadCostFunctions(obj_range,nq,NMuscle,totalFmax,indFmax)
        
        import casadi.*
        
        % Global Pelvis Orientations
        gpo_e = MX.sym('gpo_e',nq.abs/2); 
        gpo_s = MX.sym('gpo_s',nq.abs/2);
        gPelOri_temp = 0;
        for i = 1:length(gpo_e)
            gPelOri_temp = gPelOri_temp + ((gpo_e(i)-gpo_s(i))./obj_range.q(i)).^2;
        end
        costFunctions.f_J_gPelOri = Function('f_J_gPelOri',{gpo_e,gpo_s},{gPelOri_temp});
        
        % Global Pelvis Translations - excluding horizontal pelvis
        % displacement
        gpt_e = MX.sym('gpt_e',(nq.abs/2)-1);
        gpt_s = MX.sym('gpt_s',(nq.abs/2)-1);
        gPelTra_temp = 0;
        for i = 1:length(gpt_e)
            gPelTra_temp = gPelTra_temp + ((gpt_e(i)-gpt_s(i))./obj_range.q(4+i)).^2;
        end 
        costFunctions.f_J_gPelTra = Function('f_J_gPelTra',{gpt_e,gpt_s},{gPelTra_temp});

        % Lower-limb joint angles
        lljangs_e = MX.sym('lljangs_e',nq.all-nq.abs-nq.trunk-nq.arms);
        lljangs_s = MX.sym('lljangs_s',nq.all-nq.abs-nq.trunk-nq.arms);
        lljAngs_temp = 0;
        for i = 1:length(lljangs_e)
            lljAngs_temp = lljAngs_temp + ((lljangs_e(i)-lljangs_s(i))./obj_range.q(nq.abs+i)).^2;
        end
        costFunctions.f_J_lljAngs = Function('f_J_lljAngs',{lljangs_e,lljangs_s},{lljAngs_temp});

        % Upper-limb & trunk joint angles
        uljangs_e = MX.sym('uljangs_e',nq.trunk+nq.arms);
        uljangs_s = MX.sym('uljangs_s',nq.trunk+nq.arms);
        uljAngs_temp = 0;
        for i = 1:length(uljangs_e)
            uljAngs_temp = uljAngs_temp + ((uljangs_e(i)-uljangs_s(i))./obj_range.q(20+i)).^2;
        end
        costFunctions.f_J_uljAngs = Function('f_J_uljAngs',{uljangs_e,uljangs_s},{uljAngs_temp});

        % Accelerations
        accs = MX.sym('accs',nq.all);
        accs_temp = 0;
        scale_accs = 1;
        for i = 1:length(accs)
            if i > 20 % last trunk DOF
                scale_accs = 1; % 0.01
            end
            %if i == 21 || i == 22 || i == 23
            %    scale_accs = 0.001;
            %elseif i > 23
            %    scale_accs = 0.1;
            %end
            accs_temp = accs_temp + ((accs(i)./obj_range.uAcc(i)).^2)*scale_accs;
        end
        costFunctions.f_J_accs = Function('f_J_accs',{accs},{accs_temp});
        
        % Muscle activations
        muscle_act = MX.sym('muscle_act',NMuscle);
        muscle_act_temp = 0;
        for i = 1:length(muscle_act)
            muscle_act_temp = muscle_act_temp + (((muscle_act(i).^2).*indFmax(i))./totalFmax);
        end
        costFunctions.f_J_muscle_act = Function('f_J_muscle_act',{muscle_act},{muscle_act_temp});

        % Derivative of Muscle activations
        dmuscle_act = MX.sym('dmuscle_act',NMuscle);
        dmuscle_act_temp = 0;
        for i = 1:length(dmuscle_act)
            dmuscle_act_temp = dmuscle_act_temp + (((dmuscle_act(i)./obj_range.uActdot(i))).^2);
        end
        costFunctions.f_J_dmuscle_act = Function('f_J_dmuscle_act',{dmuscle_act},{dmuscle_act_temp});
        
        % Tendon Forces
        FT = MX.sym('FT',NMuscle);
        FT_temp = 0;
        for i = 1:length(FT)
            FT_temp = FT_temp + (((FT(i)./obj_range.FTtilde(i))).^2);
        end
        costFunctions.f_J_FT = Function('f_J_FT',{FT},{FT_temp});

        % Derivative of Tendon Forces
        dFT = MX.sym('dFT',NMuscle);
        dFT_temp = 0;
        for i = 1:length(dFT)
            dFT_temp = dFT_temp + (((dFT(i)./obj_range.dFTtilde(i))).^2);
        end
        costFunctions.f_J_dFT = Function('f_J_dFT',{dFT},{dFT_temp});
        
        reserves_weights = ones(2,1);
        reserves_weights(1:2) = 1; % was 0.01
        
        % Reserve actuators
        reserves = MX.sym('reserves',2);
        reserves_temp = 0;
        for i = 1:length(reserves)
            reserves_temp = reserves_temp + ((reserves_weights(i).*reserves(i))./obj_range.uReserves(i)).^2;
        end
        costFunctions.f_J_reserves = Function('f_J_reserves',{reserves},{reserves_temp});
        
        % Arm activations
        arm_acts = MX.sym('arm_acts',nq.arms);
        arm_acts_temp = 0;
        for i = 1:length(arm_acts)
            arm_acts_temp = arm_acts_temp + (arm_acts(i)./1).^2;
        end
        costFunctions.f_J_arms = Function('f_J_arms',{arm_acts},{arm_acts_temp});
        
    end

    function [f_lMT_vMT_dM,f_forceEquilibrium_FtildeState_all_tendon_M,calcMoms] = loadMuscleModelFunctions(musInd,NMuscle,m_oMFL,m_TSL,m_vmax,pathpolynomial,nq,MTparameters_m)

        import casadi.*

        % We load some variables for the polynomial approximations
        joint_info = load([pathpolynomial,'\muscle_spanning_joint_INFO_subject9.mat']);
        muscle_info = load([pathpolynomial,'\MuscleINFO_subject9.mat'],'MuscleInfo');
        % For the polynomials, we want all independent muscles. So we do not need
        % the muscles from both legs, since we assume bilateral symmetry, but want
        % all muscles from the back (indices 47:49)
        musi_pol = [musInd,47,48,49];
        NMuscle_pol = NMuscle/2+3;

        % Polynomial Approximation
        muscle_spanning_info_m = joint_info.muscle_spanning_joint_INFO(musi_pol,:);
        MuscleInfo_m.muscle    = muscle_info.MuscleInfo.muscle(musi_pol);
        qin    = SX.sym('qin',1,nq.leg);
        qdotin = SX.sym('qdotin',1,nq.leg);
        lMT    = SX(NMuscle_pol,1);
        vMT    = SX(NMuscle_pol,1);
        dM     = SX(NMuscle_pol,nq.leg);

        for i=1:NMuscle_pol      
            index_dof_crossing  = find(muscle_spanning_info_m(i,:)==1);
            order               = MuscleInfo_m.muscle(i).order;
            [mat,diff_mat_q]    = n_art_mat_3_cas_SX(qin(1,index_dof_crossing),...
                order);
            lMT(i,1)            = mat*MuscleInfo_m.muscle(i).coeff;
            vMT(i,1)            = 0;
            dM(i,1:nq.leg)      = 0;
            nr_dof_crossing     = length(index_dof_crossing); 
            for dof_nr = 1:nr_dof_crossing
                dM(i,index_dof_crossing(dof_nr)) = ...
                    (-(diff_mat_q(:,dof_nr)))'*MuscleInfo_m.muscle(i).coeff;
                vMT(i,1) = vMT(i,1) + (-dM(i,index_dof_crossing(dof_nr))*...
                    qdotin(1,index_dof_crossing(dof_nr)));
            end 
        end
        f_lMT_vMT_dM = Function('f_lMT_vMT_dM',{qin,qdotin},{lMT,vMT,dM});

        % Muscle contraction dynamics
        % Function for Hill-equilibrium
        FTtilde    = SX.sym('FTtilde',NMuscle);  % Normalized tendon forces
        a          = SX.sym('a',NMuscle);        % Muscle activations
        dFTtilde   = SX.sym('dFTtilde',NMuscle); % Normalized...? Time derivative tendon forces
        lMT        = SX.sym('lMT',NMuscle);      % Muscle-tendon lengths
        vMT        = SX.sym('vMT',NMuscle);      % Muscle-tendon velocities
        tension_SX = SX.sym('tension',NMuscle);  % Tensions...specific tensions...
        atendon_SX = SX.sym('atendon',NMuscle);  % Tendon stiffness
        shift_SX   = SX.sym('shift',NMuscle);    % shift curve-tendon...?
        Hilldiff   = SX(NMuscle,1);              % Hill-equilibrium
        FT         = SX(NMuscle,1);              % Tendon forces
        Fce        = SX(NMuscle,1);              % Contractile element forces
        Fiso       = SX(NMuscle,1);              % Normalized forces from force-length curve
        vMmax      = SX(NMuscle,1);              % Maximum contraction velocities
        Fpass      = SX(NMuscle,1);              % Passive element forces
        lTtilde    = SX(NMuscle,1);
        lM         = SX(NMuscle,1);
        lMtilde    = SX(NMuscle,1);
        FMvtilde   = SX(NMuscle,1);
        vMtilde    = SX(NMuscle,1);
        Fpetilde   = SX(NMuscle,1);

        % Parameters of force-length-velocity curves
        load Fvparam.mat Fvparam % Velocity
        load Fpparam.mat Fpparam % Passive
        load Faparam.mat Faparam % Active
        for m = 1:NMuscle
            [Hilldiff(m),FT(m),Fce(m),Fpass(m),Fiso(m),vMmax(m),lTtilde(m),lM(m),lMtilde(m),FMvtilde(m),vMtilde(m),Fpetilde(m)] = ...
                ForceEquilibrium_FtildeState_all_tendon_M(a(m),FTtilde(m),...
                dFTtilde(m),lMT(m),vMT(m),MTparameters_m([1,4],m),Fvparam,Fpparam,...
                Faparam,tension_SX(m),atendon_SX(m),shift_SX(m),m_oMFL(m),m_TSL(m),m_vmax(m));
        end
        f_forceEquilibrium_FtildeState_all_tendon_M = ...
            Function('f_forceEquilibrium_FtildeState_all_tendon_M',{a,FTtilde,...
            dFTtilde,lMT,vMT,tension_SX,atendon_SX,shift_SX,m_oMFL,m_TSL,m_vmax},...
            {Hilldiff,FT,Fce,Fpass,Fiso,vMmax,lTtilde,lM,lMtilde,FMvtilde,vMtilde,Fpetilde});
        
        % Muscle-driven joint torques for the lower limbs and the trunk
        % Define several CasADi functions for immediate use
        % Function for 27 elements 
        ma_temp27 = SX.sym('ma_temp27',27);
        ft_temp27 = SX.sym('ft_temp27',27);
        J_sptemp27 = 0;
        for i=1:length(ma_temp27)
            J_sptemp27 = J_sptemp27 + ma_temp27(i,1)*ft_temp27(i,1);    
        end
        calcMoms.f_T27 = Function('f_T27',{ma_temp27,ft_temp27},{J_sptemp27});

        % Function for 13 elements 
        ma_temp13 = SX.sym('ma_temp13',13);
        ft_temp13 = SX.sym('ft_temp13',13);
        J_sptemp13 = 0;
        for i=1:length(ma_temp13)
            J_sptemp13 = J_sptemp13 + ma_temp13(i,1)*ft_temp13(i,1);    
        end
        calcMoms.f_T13 = Function('f_T13',{ma_temp13,ft_temp13},{J_sptemp13});

        % Function for 12 elements 
        ma_temp12 = SX.sym('ma_temp12',12);
        ft_temp12 = SX.sym('ft_temp12',12);
        J_sptemp12 = 0;
        for i=1:length(ma_temp12)
            J_sptemp12 = J_sptemp12 + ma_temp12(i,1)*ft_temp12(i,1);    
        end
        calcMoms.f_T12 = Function('f_T12',{ma_temp12,ft_temp12},{J_sptemp12});

        % Function for 4 elements 
        ma_temp4 = SX.sym('ma_temp4',4);
        ft_temp4 = SX.sym('ft_temp4',4);
        J_sptemp4 = 0;
        for i=1:length(ma_temp4)
            J_sptemp4 = J_sptemp4 + ma_temp4(i,1)*ft_temp4(i,1);    
        end
        calcMoms.f_T4 = Function('f_T4',{ma_temp4,ft_temp4},{J_sptemp4});

        % Function for 6 elements 
        ma_temp6 = SX.sym('ma_temp6',6);
        ft_temp6 = SX.sym('ft_temp6',6);
        J_sptemp6 = 0;
        for i=1:length(ma_temp6)
            J_sptemp6 = J_sptemp6 + ma_temp6(i,1)*ft_temp6(i,1);    
        end
        calcMoms.f_T6 = Function('f_T6',{ma_temp6,ft_temp6},{J_sptemp6});

    end

    function [w,w0,lbw,ubw,J,g,lbg,ubg,g_names,change_p_disp,J_v_GRF,vertImp] = buildNLP(guess,bounds_sc,scalingN,nq,Options,h,F,statesF,contPrms_nsc,C,D,B,wJ,costFunctions,oMFL_2_nsc,TSL_2_nsc,NMuscle,vMaxMult,outInd,preOpt,finalTime_nsc,D_controls)

        import casadi.*

        scalingCons = scalingN;
        
        J_v_GRF = 0;
        vertImp = 0;

        J_work_net = 0;
        J_work_net_pos_ext = 0;
        J_work_net_neg_ext = 0;
        J_work_net_pos_fle = 0;
        J_work_net_neg_fle = 0;

        % Start with an empty NLP
        w   = {}; % DVs vector
        w0  = []; % Scaled initial guess of DVs
        lbw = []; % Scaled lower bounds
        ubw = []; % Scaled upper bounds
        J   = 0;  % Objective Function
        g   = {}; % Constraints
        lbg = []; % Lower bounds constraints (e.g., Path constraints)
        ubg = []; % Upper bounds constraints

        % Create cell of constraints - to help debug later
        g_names = {};    

        counter = MX(Options.N*d,1);
        time_cont_vec = MX(Options.N*d,1);
        diff_cont = MX(Options.N,1);


        for k = 0:Options.N-1

            % Define states at first mesh point
            % Qs & Qdots
            % Organised as: [q1 q2...q1dot q2dot...]
            Xk       = MX.sym(['X_' num2str(k)],2*nq.all);
            w        = {w{:}, Xk};
            lbw      = [lbw [bounds_sc.q.lower(:,k*(d+1)+1)' bounds_sc.qdot.lower(:,k*(d+1)+1)']];
            ubw      = [ubw [bounds_sc.q.upper(:,k*(d+1)+1)' bounds_sc.qdot.upper(:,k*(d+1)+1)']];
            w0       = [w0  guess.q(k*(d+1)+1,:) guess.qdot(k*(d+1)+1,:)];

            Xk_nsc = Xk.*[scalingN.q; scalingN.qdot];

            % Muscle activations
            actk     = MX.sym(['act_' num2str(k)],NMuscle);
            w        = {w{:}, actk};
            lbw      = [lbw bounds_sc.act.lower];
            ubw      = [ubw bounds_sc.act.upper];
            w0       = [w0 guess.act(:,k*(d+1)+1)'];

            actk_nsc = actk.*scalingN.act';

            % Normalised Tendon forces
            FTtildek = MX.sym(['FTtilde_' num2str(k)],NMuscle);
            w        = {w{:}, FTtildek};
            lbw      = [lbw bounds_sc.FTtilde.lower];
            ubw      = [ubw bounds_sc.FTtilde.upper];
            w0       = [w0 guess.FTtilde(:,k*(d+1)+1)'];

            FTtildek_nsc = FTtildek.*scalingN.FTtilde';

            % Arm Activations
            armActsk = MX.sym(['armActs_' num2str(k)],nq.arms);
            w        = {w{:}, armActsk};
            lbw      = [lbw bounds_sc.aArms.lower'];
            ubw      = [ubw bounds_sc.aArms.upper'];
            w0       = [w0 guess.aArms(:,k*(d+1)+1)'];

            armActsk_nsc = armActsk;
            
            % Grab a hold of states at initial discretization point for
            % later use
            if k == 0
                Xk_nsc_ini       = Xk_nsc;                
                actk_nsc_ini     = actk_nsc;
                FTtildek_nsc_ini = FTtildek_nsc;
                armActsk_nsc_ini = armActsk_nsc;   
            end

            % Define states at collocation points
            % Qs & Qdots
            Xkj = {};
            for j=1:d
                Xkj{j}             = MX.sym(['X_' num2str(k) '_' num2str(j)],2*nq.all);
                w                  = {w{:}, Xkj{j}};
                lbw                = [lbw [bounds_sc.q.lower(:,k*(d+1)+1+j)' bounds_sc.qdot.lower(:,k*(d+1)+1+j)']];
                ubw                = [ubw [bounds_sc.q.upper(:,k*(d+1)+1+j)' bounds_sc.qdot.upper(:,k*(d+1)+1+j)']];
                w0                 = [w0 guess.q(k*(d+1)+1+j,:) guess.qdot(k*(d+1)+1+j,:)];

                Xkj_nsc{j}         = Xkj{j}.*[scalingN.q; scalingN.qdot]; 
            end

            % Muscle activations
            actkj = {};
            for j=1:d
                actkj{j}           = MX.sym(['act_' num2str(k) '_' num2str(j)],NMuscle);
                w                  = {w{:}, actkj{j}};
                lbw                = [lbw bounds_sc.act.lower];
                ubw                = [ubw bounds_sc.act.upper];
                w0                 = [w0 guess.act(:,k*(d+1)+1+j)'];

                actkj_nsc{j}       = actkj{j}.*scalingN.act';
            end

            % Normalised tendon forces
            FTtildekj = {};
            for j=1:d
                FTtildekj{j}       = MX.sym(['FTtilde_' num2str(k) '_' num2str(j)],NMuscle);
                w                  = {w{:}, FTtildekj{j}};
                lbw                = [lbw bounds_sc.FTtilde.lower];
                ubw                = [ubw bounds_sc.FTtilde.upper];
                w0                 = [w0 guess.FTtilde(:,k*(d+1)+1+j)'];

                FTtildekj_nsc{j}   = FTtildekj{j}.*scalingN.FTtilde';
            end

            % Arm activations
            armActskj = {};
            for j=1:d
                armActskj{j}       = MX.sym(['armActs_' num2str(k) '_' num2str(j)],nq.arms);
                w                  = {w{:}, armActskj{j}};
                lbw                = [lbw bounds_sc.aArms.lower'];
                ubw                = [ubw bounds_sc.aArms.upper'];
                w0                 = [w0 guess.aArms(:,k*(d+1)+1+j)'];

                armActskj_nsc{j}   = armActskj{j};
            end

            % Define controls at collocation points
            % Time derivative of normalised tendon forces
            dFTtildekj = {};
            for j=1:d
                dFTtildekj{j}      = MX.sym(['dFTtilde_' num2str(k) '_' num2str(j)],NMuscle);
                w                  = {w{:}, dFTtildekj{j}};
                lbw                = [lbw bounds_sc.dFTtilde.lower];
                ubw                = [ubw bounds_sc.dFTtilde.upper];
                w0                 = [w0 guess.dFTtilde(:,k*d+j)'];

                dFTtildekj_nsc{j}  = dFTtildekj{j}.*scalingN.dFTtilde';
            end

            % Time derivatives of muscle activations
            uActdotkj = {};
            for j=1:d
                uActdotkj{j}       = MX.sym(['uActdot_' num2str(k) '_' num2str(j)],NMuscle);
                w                  = {w{:}, uActdotkj{j}};
                lbw                = [lbw bounds_sc.uActdot.lower];
                ubw                = [ubw bounds_sc.uActdot.upper];
                w0                 = [w0 guess.uActdot(:,k*d+j)'];

                uActdotkj_nsc{j}   = uActdotkj{j}.*scalingN.uActdot';
            end

            % Time derivatives of velocities - accelerations
            uAcckj = {};
            for j=1:d
                uAcckj{j}          = MX.sym(['uAcc_' num2str(k) '_' num2str(j)],nq.all);
                w                  = {w{:}, uAcckj{j}};
                lbw                = [lbw bounds_sc.uAcc.lower(:,k*d+j)'];
                ubw                = [ubw bounds_sc.uAcc.upper(:,k*d+j)'];
                w0                 = [w0 guess.uAcc(k*d+j,:)];

                uAcckj_nsc{j}      = uAcckj{j}.*scalingN.uAcc;
            end

            % Reserve actuators controls
            uReserveskj = {};
            for j=1:d
                uReserveskj{j}     = MX.sym(['uReserves_' num2str(k) '_' num2str(j)],2);
                w                  = {w{:}, uReserveskj{j}};
                lbw                = [lbw bounds_sc.uReserves.lower'];
                ubw                = [ubw bounds_sc.uReserves.upper'];
                w0                 = [w0 guess.uReserves(:,k*d+j)'];

                uReserveskj_nsc{j} = uReserveskj{j}.*scalingN.uReserves;
            end

            % Arm excitation controls
            armExctkj = {};
            for j=1:d
                armExctkj{j}       = MX.sym(['armExct_' num2str(k) '_' num2str(j)],nq.arms);
                w                  = {w{:}, armExctkj{j}};
                lbw                = [lbw bounds_sc.eArms.lower'];
                ubw                = [ubw bounds_sc.eArms.upper'];
                w0                 = [w0 guess.eArms(:,k*d+j)'];

                armExctkj_nsc{j}   = armExctkj{j};
            end

                    % For first mesh interval evaluate all constraints at the beginning 
                    if k == 0
                        % First extrapolate all controls to beginning of mesh interval as
                        % we do not discretize them there
                        dFTtilde_nsc_ini  = 0;
                        uActdot_nsc_ini   = 0;
                        uAcc_nsc_ini      = 0;
                        uReserves_nsc_ini = 0;
                        armExct_nsc_ini   = 0;

                        for j=1:d
                            dFTtilde_nsc_ini  = dFTtilde_nsc_ini  + D_controls(j).*dFTtildekj_nsc{j};
                            uActdot_nsc_ini   = uActdot_nsc_ini   + D_controls(j).*uActdotkj_nsc{j};
                            uAcc_nsc_ini      = uAcc_nsc_ini      + D_controls(j).*uAcckj_nsc{j};
                            uReserves_nsc_ini = uReserves_nsc_ini + D_controls(j).*uReserveskj_nsc{j};
                            armExct_nsc_ini   = armExct_nsc_ini   + D_controls(j).*armExctkj_nsc{j};
                        end

                        % Reorder the skeleton states as q1 u1 q2 u2...
                        % For compatibility with F function
                        Xk_nsc_ORD = MX(nq.all*2,1);
                        for i = 0:nq.all-1
                            Xk_nsc_ORD(i+1*(i+1),1) = Xk_nsc_ini(i+1,1);
                            Xk_nsc_ORD(2*(i+1),1)   = Xk_nsc_ini(nq.all+i+1,1);
                        end

                        % Evaluate F Function 
                        % Evaluation returns residuals, joint torques 
                        % and GRFs from contact model
                        % Output used later
                        outputF = F([Xk_nsc_ORD;uAcc_nsc_ini;contPrms_nsc ]);
                        outputF_ini = outputF;
                        knee_pos_R_ini = outputF_ini(outInd.knee_r_pos_XYZ_F);
                        knee_pos_L_ini = outputF_ini(outInd.knee_l_pos_XYZ_F);

                        % Get MTU lengths, MTU velocities & moment arms
                        % First left leg
                        qin_l = [Xk_nsc(jointi.hip_flex.l),...
                            Xk_nsc(jointi.hip_add.l),...
                            Xk_nsc(jointi.hip_rot.l),...
                            Xk_nsc(jointi.knee.l),...
                            Xk_nsc(jointi.ankle.l),...
                            Xk_nsc(jointi.subt.l),...
                            Xk_nsc(jointi.mtp.l),...
                            Xk_nsc(jointi.trunk.ext),...
                            Xk_nsc(jointi.trunk.ben),...
                            Xk_nsc(jointi.trunk.rot)];
                        qdotin_l = [Xk_nsc(jointi.hip_flex.l+nq.all),...
                            Xk_nsc(jointi.hip_add.l+nq.all),...
                            Xk_nsc(jointi.hip_rot.l+nq.all),...
                            Xk_nsc(jointi.knee.l+nq.all),...
                            Xk_nsc(jointi.ankle.l+nq.all),...
                            Xk_nsc(jointi.subt.l+nq.all),...
                            Xk_nsc(jointi.mtp.l+nq.all),...
                            Xk_nsc(jointi.trunk.ext+nq.all),...
                            Xk_nsc(jointi.trunk.ben+nq.all),...
                            Xk_nsc(jointi.trunk.rot+nq.all)];
                        [lMTk_l,vMTk_l,MA_l] = f_lMT_vMT_dM(qin_l,qdotin_l);
                        MA.hip_flex.l   = MA_l(mai(1).mus.l',1);
                        MA.hip_add.l    = MA_l(mai(2).mus.l',2);
                        MA.hip_rot.l    = MA_l(mai(3).mus.l',3);
                        MA.knee.l       = MA_l(mai(4).mus.l',4);
                        MA.ankle.l      = MA_l(mai(5).mus.l',5);
                        MA.subt.l       = MA_l(mai(6).mus.l',6);
                        MA.mtp.l        = MA_l(mai(7).mus.l',7);

                        % For the back muscles, we want left & right together:
                        % left first, right second. In MuscleInfo, we first have right
                        % back muscles (44:46) & then left back muscles (47:49). Since 
                        % back muscles only depend on back DOFs, we do not care if we 
                        % extract from "left leg or right leg"; so here we just pick left
                        MA.trunk_ext    = MA_l([47:49,mai(8).mus.l]',8);
                        MA.trunk_ben    = MA_l([47:49,mai(9).mus.l]',9);
                        MA.trunk_rot    = MA_l([47:49,mai(10).mus.l]',10);

                        % Second right leg
                        qin_r = [Xk_nsc(jointi.hip_flex.r),...
                            Xk_nsc(jointi.hip_add.r),...
                            Xk_nsc(jointi.hip_rot.r),...
                            Xk_nsc(jointi.knee.r),...
                            Xk_nsc(jointi.ankle.r),...
                            Xk_nsc(jointi.subt.r),...
                            Xk_nsc(jointi.mtp.r),...
                            Xk_nsc(jointi.trunk.ext),...
                            Xk_nsc(jointi.trunk.ben),...
                            Xk_nsc(jointi.trunk.rot)];
                        qdotin_r = [Xk_nsc(jointi.hip_flex.r+nq.all),...
                            Xk_nsc(jointi.hip_add.r+nq.all),...
                            Xk_nsc(jointi.hip_rot.r+nq.all),...
                            Xk_nsc(jointi.knee.r+nq.all),...
                            Xk_nsc(jointi.ankle.r+nq.all),...
                            Xk_nsc(jointi.subt.r+nq.all),...
                            Xk_nsc(jointi.mtp.r+nq.all),...
                            Xk_nsc(jointi.trunk.ext+nq.all),...
                            Xk_nsc(jointi.trunk.ben+nq.all),...
                            Xk_nsc(jointi.trunk.rot+nq.all)];
                        [lMTk_r,vMTk_r,MA_r] = f_lMT_vMT_dM(qin_r,qdotin_r);
                        % Here we take the indices from left since the vector is 1:49
                        % MA_r: 49 x 10
                        % mai: is created starting with left muscles then right muscles
                        % Querying MA_r with 'mus.r' would result in an error
                        MA.hip_flex.r   = MA_r(mai(1).mus.l',1);
                        MA.hip_add.r    = MA_r(mai(2).mus.l',2);
                        MA.hip_rot.r    = MA_r(mai(3).mus.l',3);
                        MA.knee.r       = MA_r(mai(4).mus.l',4);
                        MA.ankle.r      = MA_r(mai(5).mus.l',5);
                        MA.subt.r       = MA_r(mai(6).mus.l',6);
                        MA.mtp.r        = MA_r(mai(7).mus.l',7);

                        % Both legs
                        % In MuscleInfo, we first have the right back muscles (44:46)
                        % and then the left back muscles (47:49). We reorganise so that
                        % we first have the left muscles, then the right muscles
                        lMTk_lr = [lMTk_l([1:43,47:49],1);lMTk_r(1:46,1)];
                        vMTk_lr = [vMTk_l([1:43,47:49],1);vMTk_r(1:46,1)];

                        % Get the non-normalised unscaled tendon forces & 
                        % Hill equilibrium evaluation (normalised unscaled)
                        % Outputs are left leg & back muscles, then right leg & back muscles
                        [Hilldiff_ini,FT_ini,Fce_ini,Fpass_ini,Fiso_ini,vMax_ini,lTtilde_ini,lM_ini,lMtilde_ini] = ...
                            f_forceEquilibrium_FtildeState_all_tendon_M(actk_nsc_ini,FTtildek_nsc_ini,...
                            dFTtilde_nsc_ini,lMTk_lr,vMTk_lr,tensions,aTendon,shift,oMFL_2_nsc,TSL_2_nsc,oMFL_2_nsc.*vMaxMult);

%                         % Add path constraints (implicit formulation)
%                         % Pelvis residuals
%                         g   = {g{:}, outputF(1:6)./(72.2*9.81)};
%                         lbg = [lbg; zeros(6,1)]; 
%                         ubg = [ubg; zeros(6,1)];  
% 
%                         % Arm torques
%                         g   = {g{:}, (outputF(armsi)-(armActsk_nsc_ini.*scalingN.uArms))./(72.2*9.81)};
%                         lbg = [lbg; zeros(length(armsi),1)];
%                         ubg = [ubg; zeros(length(armsi),1)];
% 
%                         % Hip flexion, left
%                         % FTk: non-normalised unscaled tendon force
%                         Ft_hip_flex_l   = FT_ini(mai(1).mus.l',1);
%                         T_hip_flex_l    = calcMoms.f_T27(MA.hip_flex.l,Ft_hip_flex_l);
%                         g               = {g{:}, (T_hip_flex_l - outputF(jointi.hip_flex.l))./1};
%                         lbg             = [lbg; 0];
%                         ubg             = [ubg; 0];
% 
%                         % Hip flexion, right
%                         Ft_hip_flex_r   = FT_ini(mai(1).mus.r',1);
%                         T_hip_flex_r    = calcMoms.f_T27(MA.hip_flex.r,Ft_hip_flex_r);
%                         g               = {g{:}, (T_hip_flex_r - outputF(jointi.hip_flex.r))./1};
%                         lbg             = [lbg; 0];
%                         ubg             = [ubg; 0];
% 
%                         % Hip adduction, left
%                         Ft_hip_add_l    = FT_ini(mai(2).mus.l',1);
%                         T_hip_add_l     = calcMoms.f_T27(MA.hip_add.l,Ft_hip_add_l);
%                         g               = {g{:}, (T_hip_add_l - outputF(jointi.hip_add.l))./1};
%                         lbg             = [lbg; 0];
%                         ubg             = [ubg; 0];
% 
%                         % Hip adduction, right
%                         Ft_hip_add_r    = FT_ini(mai(2).mus.r',1);
%                         T_hip_add_r     = calcMoms.f_T27(MA.hip_add.r,Ft_hip_add_r);
%                         g               = {g{:}, (T_hip_add_r - outputF(jointi.hip_add.r))./1};
%                         lbg             = [lbg; 0];
%                         ubg             = [ubg; 0];
% 
%                         % Hip rotation, left
%                         Ft_hip_rot_l    = FT_ini(mai(3).mus.l',1);
%                         T_hip_rot_l     = calcMoms.f_T27(MA.hip_rot.l,Ft_hip_rot_l);
%                         g               = {g{:}, (T_hip_rot_l - outputF(jointi.hip_rot.l))./1};
%                         lbg             = [lbg; 0];
%                         ubg             = [ubg; 0];
% 
%                         % Hip rotation, right
%                         Ft_hip_rot_r    = FT_ini(mai(3).mus.r',1);
%                         T_hip_rot_r     = calcMoms.f_T27(MA.hip_rot.r,Ft_hip_rot_r);
%                         g               = {g{:}, (T_hip_rot_r - outputF(jointi.hip_rot.r))./1};
%                         lbg             = [lbg; 0];
%                         ubg             = [ubg; 0];
% 
%                         % Knee, left
%                         Ft_knee_l       = FT_ini(mai(4).mus.l',1);
%                         T_knee_l        = calcMoms.f_T13(MA.knee.l,Ft_knee_l);
%                         g               = {g{:}, (T_knee_l - outputF(jointi.knee.l))./1};
%                         lbg             = [lbg; 0];
%                         ubg             = [ubg; 0];
% 
%                         % Knee, right
%                         Ft_knee_r       = FT_ini(mai(4).mus.r',1);
%                         T_knee_r        = calcMoms.f_T13(MA.knee.r,Ft_knee_r);
%                         g               = {g{:}, (T_knee_r - outputF(jointi.knee.r))./1};
%                         lbg             = [lbg; 0];
%                         ubg             = [ubg; 0];
% 
%                         % Ankle, left
%                         Ft_ankle_l      = FT_ini(mai(5).mus.l',1);
%                         T_ankle_l       = calcMoms.f_T12(MA.ankle.l,Ft_ankle_l);
%                         g               = {g{:}, (T_ankle_l - outputF(jointi.ankle.l))./1};
%                         lbg             = [lbg; 0];
%                         ubg             = [ubg; 0];
% 
%                         % Ankle, right
%                         Ft_ankle_r      = FT_ini(mai(5).mus.r',1);
%                         T_ankle_r       = calcMoms.f_T12(MA.ankle.r,Ft_ankle_r);
%                         g               = {g{:}, (T_ankle_r - outputF(jointi.ankle.r))./1};
%                         lbg             = [lbg; 0];
%                         ubg             = [ubg; 0];
% 
%                         % Subtalar, left
%                         Ft_subt_l       = FT_ini(mai(6).mus.l',1);
%                         T_subt_l        = calcMoms.f_T12(MA.subt.l,Ft_subt_l);
%                         g               = {g{:}, (T_subt_l - outputF(jointi.subt.l))./1};
%                         lbg             = [lbg; 0];
%                         ubg             = [ubg; 0];
% 
%                         % Subtalar, right
%                         Ft_subt_r       = FT_ini(mai(6).mus.r',1);
%                         T_subt_r        = calcMoms.f_T12(MA.subt.r,Ft_subt_r);
%                         g               = {g{:}, (T_subt_r - outputF(jointi.subt.r))./1};
%                         lbg             = [lbg; 0];
%                         ubg             = [ubg; 0];
% 
%                         % MTP, left
%                         Ft_mtp_l        = FT_ini(mai(7).mus.l',1);
%                         T_mtp_l         = calcMoms.f_T4(MA.mtp.l,Ft_mtp_l);
%                         g               = {g{:}, (T_mtp_l - outputF(jointi.mtp.l) + uReserves_nsc_ini(1) - Options.MTP_stiff*Xk_nsc(jointi.mtp.l))./1};
%                         lbg             = [lbg; 0];
%                         ubg             = [ubg; 0];
% 
%                         % MTP, right
%                         Ft_mtp_r        = FT_ini(mai(7).mus.r',1);
%                         T_mtp_r         = calcMoms.f_T4(MA.mtp.r,Ft_mtp_r);
%                         g               = {g{:}, (T_mtp_r - outputF(jointi.mtp.r) + uReserves_nsc_ini(2) - Options.MTP_stiff*Xk_nsc(jointi.mtp.r))./1};
%                         lbg             = [lbg; 0];
%                         ubg             = [ubg; 0];
% 
%                         % Lumbar extension
%                         Ft_trunk_ext    = FT_ini([mai(8).mus.l,mai(8).mus.r]',1);
%                         T_trunk_ext     = calcMoms.f_T6(MA.trunk_ext,Ft_trunk_ext);
%                         g               = {g{:}, (T_trunk_ext - outputF(jointi.trunk.ext))./1};
%                         lbg             = [lbg; 0];
%                         ubg             = [ubg; 0];
% 
%                         % Lumbar bending 
%                         Ft_trunk_ben    = FT_ini([mai(9).mus.l,mai(9).mus.r]',1);
%                         T_trunk_ben     = calcMoms.f_T6(MA.trunk_ben,Ft_trunk_ben);
%                         g               = {g{:}, (T_trunk_ben - outputF(jointi.trunk.ben))./1};
%                         lbg             = [lbg; 0];
%                         ubg             = [ubg; 0];
% 
%                         % Lumbar rotating
%                         Ft_trunk_rot    = FT_ini([mai(10).mus.l,mai(10).mus.r]',1);
%                         T_trunk_rot     = calcMoms.f_T6(MA.trunk_rot,Ft_trunk_rot);
%                         g               = {g{:}, (T_trunk_rot - outputF(jointi.trunk.rot))./1};
%                         lbg             = [lbg; 0];
%                         ubg             = [ubg; 0];
% 
% %                         % Constrain maximum amplitude of thigh flexion angles
% %                         g   = {g{:}, outputF([outInd.femur_r_XYZ(3),outInd.femur_l_XYZ(3)])};
% %                         lbg = [lbg; -inf*ones(2,1)];  
% %                         ubg = [ubg; deg2rad(140).*ones(2,1)];
%                         
%                         % Constrain magnitude of distance between frames
%                         g   = {g{:}, outputF([outInd.tibia_l_calc_r_mag,outInd.tibia_r_calc_l_mag])};
%                         lbg = [lbg; 0.12*ones(2,1)];
%                         ubg = [ubg; 5*ones(2,1)];
% 
% % % %                         % Constrain magnitude of distance between frames
% % % %                         g   = {g{:}, outputF([outInd.calc_mag])};
% % % %                         lbg = [lbg; 0.45];
% % % %                         ubg = [ubg; 5];
% 
%                         % Activation dynamics (implicit formulation)
%                         tact = 0.015;
%                         tdeact = 0.06;
%                         act1 = uActdot_nsc_ini + actk_nsc_ini./(ones(size(actk_nsc,1),1)*tdeact);
%                         act2 = uActdot_nsc_ini + actk_nsc_ini./(ones(size(actk_nsc,1),1)*tact);
% 
%                         % act 1 path constraint
%                         g   = {g{:},act1};
%                         lbg = [lbg; zeros(NMuscle,1)];
%                         ubg = [ubg; inf*ones(NMuscle,1)];
% 
%                         % act 2 path contraint
%                         g   = {g{:},act2};
%                         lbg = [lbg; -inf*ones(NMuscle,1)];
%                         ubg = [ubg; ones(NMuscle,1)./(ones(NMuscle,1)*tact)];
% 
%                         % Contraction dynamics (implicit formulation)
%                         g   = {g{:},Hilldiff_ini};
%                         lbg = [lbg; zeros(NMuscle,1)];
%                         ubg = [ubg; zeros(NMuscle,1)];

                    end

            % Include continuity constraint between mesh intervals for states
            if k > 0

                g   = {g{:}, (Xk_nsc_end - Xk_nsc)./[1]};
                lbg = [lbg; zeros(2*nq.all,1)];
                ubg = [ubg; zeros(2*nq.all,1)];

                g   = {g{:}, (actk_nsc_end - actk_nsc)};
                lbg = [lbg; zeros(NMuscle,1)];
                ubg = [ubg; zeros(NMuscle,1)];

                g   = {g{:}, (FTtildek_nsc_end - FTtildek_nsc)./scalingCons.FTtilde'};
                lbg = [lbg; zeros(NMuscle,1)];
                ubg = [ubg; zeros(NMuscle,1)];

                g   = {g{:}, (armActsk_nsc_end - armActsk_nsc)};
                lbg = [lbg; zeros(nq.arms,1)];
                ubg = [ubg; zeros(nq.arms,1)];       

            end

            % Loop over collocation points
            % Use D matrix to evaluate state at end of k^th mesh interval
            Xk_nsc_end        = D(1)*Xk_nsc;
            actk_nsc_end      = D(1)*actk_nsc;
            FTtildek_nsc_end  = D(1)*FTtildek_nsc;
            armActsk_nsc_end  = D(1)*armActsk_nsc;

            % Evaluate all constraints at the collocation points
            for j=1:d
                % Expression for the state derivatives at the collocation point
                Xp_nsc           = C(1,j+1)*Xk_nsc;
                actp_nsc         = C(1,j+1)*actk_nsc;
                FTtildep_nsc     = C(1,j+1)*FTtildek_nsc;
                armActsp_nsc     = C(1,j+1)*armActsk_nsc;
                for r = 1:d
                    Xp_nsc       = Xp_nsc + C(r+1,j+1)*Xkj_nsc{r};
                    actp_nsc     = actp_nsc + C(r+1,j+1)*actkj_nsc{r};
                    FTtildep_nsc = FTtildep_nsc + C(r+1,j+1)*FTtildekj_nsc{r};
                    armActsp_nsc = armActsp_nsc + C(r+1,j+1)*armActskj_nsc{r};
                end
                % Append collocation equations
                % Dynamic constraints are scaled using the same scale
                % factors as was used to scale the states
                % Activation dynamics (implicit formulation)
                g      = {g{:}, (h*uActdotkj_nsc{j} - actp_nsc)};
                lbg    = [lbg; zeros(NMuscle,1)];
                ubg    = [ubg; zeros(NMuscle,1)];

                % Contraction dynamics (implicit formulation)
                g      = {g{:}, (h*dFTtildekj_nsc{j} - FTtildep_nsc)./(scalingCons.FTtilde')};
                lbg    = [lbg; zeros(NMuscle,1)];
                ubg    = [ubg; zeros(NMuscle,1)];

                % Skeleton dynamics (implicit formulation)
                fj_nsc = [Xkj_nsc{j}(nq.all+1:end); uAcckj_nsc{j}];
                g      = {g{:}, (h*fj_nsc - Xp_nsc)./[scalingCons.q; scalingCons.qdot]};
                lbg    = [lbg; zeros(2*nq.all,1)];
                ubg    = [ubg; zeros(2*nq.all,1)];

                % Arm Activations dynamics (explicit formulation)
                dadt   = (armExctkj_nsc{j} - armActskj_nsc{j}) ./ 0.035;
                g      = {g{:}, h*dadt - armActsp_nsc};
                lbg    = [lbg; zeros(nq.arms,1)];
                ubg    = [ubg; zeros(nq.arms,1)];

                % Add contribution to the end state
                Xk_nsc_end       = Xk_nsc_end + D(j+1)*Xkj_nsc{j};
                actk_nsc_end     = actk_nsc_end + D(j+1)*actkj_nsc{j};
                FTtildek_nsc_end = FTtildek_nsc_end + D(j+1)*FTtildekj_nsc{j};
                armActsk_nsc_end = armActsk_nsc_end + D(j+1)*armActskj_nsc{j};

                % Reorder the skeleton states as q1 u1 q2 u2...
                % For compatibility with F function
                Xkj_nsc_ORD = MX(nq.all*2,1);
                for i = 0:nq.all-1
                    Xkj_nsc_ORD(i+1*(i+1),1) = Xkj_nsc{j}(i+1,1);
                    Xkj_nsc_ORD(2*(i+1),1)   = Xkj_nsc{j}(nq.all+i+1,1);
                end

                % Evaluate F Function 
                % Evaluation returns residuals, joint torques 
                % and GRFs from contact model
                % Output used later
                outputF = F([Xkj_nsc_ORD;uAcckj_nsc{j};contPrms_nsc ]);

                % Get MTU lengths, MTU velocities & moment arms
                % First left leg
                qin_l = [Xkj_nsc{j}(jointi.hip_flex.l),...
                    Xkj_nsc{j}(jointi.hip_add.l),...
                    Xkj_nsc{j}(jointi.hip_rot.l),...
                    Xkj_nsc{j}(jointi.knee.l),...
                    Xkj_nsc{j}(jointi.ankle.l),...
                    Xkj_nsc{j}(jointi.subt.l),...
                    Xkj_nsc{j}(jointi.mtp.l),...
                    Xkj_nsc{j}(jointi.trunk.ext),...
                    Xkj_nsc{j}(jointi.trunk.ben),...
                    Xkj_nsc{j}(jointi.trunk.rot)];
                qdotin_l = [Xkj_nsc{j}(jointi.hip_flex.l+nq.all),...
                    Xkj_nsc{j}(jointi.hip_add.l+nq.all),...
                    Xkj_nsc{j}(jointi.hip_rot.l+nq.all),...
                    Xkj_nsc{j}(jointi.knee.l+nq.all),...
                    Xkj_nsc{j}(jointi.ankle.l+nq.all),...
                    Xkj_nsc{j}(jointi.subt.l+nq.all),...
                    Xkj_nsc{j}(jointi.mtp.l+nq.all),...
                    Xkj_nsc{j}(jointi.trunk.ext+nq.all),...
                    Xkj_nsc{j}(jointi.trunk.ben+nq.all),...
                    Xkj_nsc{j}(jointi.trunk.rot+nq.all)];
                [lMTk_l,vMTk_l,MA_l] = f_lMT_vMT_dM(qin_l,qdotin_l);
                MA.hip_flex.l   = MA_l(mai(1).mus.l',1);
                MA.hip_add.l    = MA_l(mai(2).mus.l',2);
                MA.hip_rot.l    = MA_l(mai(3).mus.l',3);
                MA.knee.l       = MA_l(mai(4).mus.l',4);
                MA.ankle.l      = MA_l(mai(5).mus.l',5);
                MA.subt.l       = MA_l(mai(6).mus.l',6);
                MA.mtp.l        = MA_l(mai(7).mus.l',7);

                % For the back muscles, we want left & right together:
                % left first, right second. In MuscleInfo, we first have right
                % back muscles (44:46) & then left back muscles (47:49). Since 
                % back muscles only depend on back DOFs, we do not care if we 
                % extract from "left leg or right leg"; so here we just pick left
                MA.trunk_ext    = MA_l([47:49,mai(8).mus.l]',8);
                MA.trunk_ben    = MA_l([47:49,mai(9).mus.l]',9);
                MA.trunk_rot    = MA_l([47:49,mai(10).mus.l]',10);

                % Second right leg
                qin_r = [Xkj_nsc{j}(jointi.hip_flex.r),...
                    Xkj_nsc{j}(jointi.hip_add.r),...
                    Xkj_nsc{j}(jointi.hip_rot.r),...
                    Xkj_nsc{j}(jointi.knee.r),...
                    Xkj_nsc{j}(jointi.ankle.r),...
                    Xkj_nsc{j}(jointi.subt.r),...
                    Xkj_nsc{j}(jointi.mtp.r),...
                    Xkj_nsc{j}(jointi.trunk.ext),...
                    Xkj_nsc{j}(jointi.trunk.ben),...
                    Xkj_nsc{j}(jointi.trunk.rot)];
                qdotin_r = [Xkj_nsc{j}(jointi.hip_flex.r+nq.all),...
                    Xkj_nsc{j}(jointi.hip_add.r+nq.all),...
                    Xkj_nsc{j}(jointi.hip_rot.r+nq.all),...
                    Xkj_nsc{j}(jointi.knee.r+nq.all),...
                    Xkj_nsc{j}(jointi.ankle.r+nq.all),...
                    Xkj_nsc{j}(jointi.subt.r+nq.all),...
                    Xkj_nsc{j}(jointi.mtp.r+nq.all),...
                    Xkj_nsc{j}(jointi.trunk.ext+nq.all),...
                    Xkj_nsc{j}(jointi.trunk.ben+nq.all),...
                    Xkj_nsc{j}(jointi.trunk.rot+nq.all)];
                [lMTk_r,vMTk_r,MA_r] = f_lMT_vMT_dM(qin_r,qdotin_r);

                % Here we take the indices from left since the vector is 1:49
                % MA_r: 49 x 10
                % mai: is created starting with left muscles then right muscles
                % Querying MA_r with 'mus.r' would result in an error
                MA.hip_flex.r   = MA_r(mai(1).mus.l',1);
                MA.hip_add.r    = MA_r(mai(2).mus.l',2);
                MA.hip_rot.r    = MA_r(mai(3).mus.l',3);
                MA.knee.r       = MA_r(mai(4).mus.l',4);
                MA.ankle.r      = MA_r(mai(5).mus.l',5);
                MA.subt.r       = MA_r(mai(6).mus.l',6);
                MA.mtp.r        = MA_r(mai(7).mus.l',7);

                % Both legs
                % In MuscleInfo, we first have the right back muscles (44:46)
                % and then the left back muscles (47:49). We reorganise so that
                % we first have the left muscles, then the right muscles
                lMTk_lr = [lMTk_l([1:43,47:49],1);lMTk_r(1:46,1)];
                vMTk_lr = [vMTk_l([1:43,47:49],1);vMTk_r(1:46,1)];

                % Get the non-normalised unscaled tendon forces & 
                % Hill equilibrium evaluation (normalised unscaled)
                % Outputs are left leg & back muscles, then right leg & back muscles
                [Hilldiffkj,FTkj,Fcekj,Fpasskj,Fisokj,vMaxkj,~,lMkj,lMtildekj] = ...
                    f_forceEquilibrium_FtildeState_all_tendon_M(actkj_nsc{j},FTtildekj_nsc{j},...
                    dFTtildekj_nsc{j},lMTk_lr,vMTk_lr,tensions,aTendon,shift,oMFL_2_nsc,TSL_2_nsc,oMFL_2_nsc.*vMaxMult);

                % Add path constraints (implicit formulation)
                % Pelvis residuals
                g   = {g{:}, outputF(1:6)./(72.2*9.81)};
                lbg = [lbg; zeros(6,1)]; 
                ubg = [ubg; zeros(6,1)];

                % Arm torques
                g   = {g{:}, (outputF(armsi)-(armActskj_nsc{j}.*scalingN.uArms))./(72.2*9.81)};
                lbg = [lbg; zeros(length(armsi),1)];
                ubg = [ubg; zeros(length(armsi),1)];

                % Hip flexion, left
                % FTk: non-normalised unscaled tendon force
                Ft_hip_flex_l   = FTkj(mai(1).mus.l',1);
                T_hip_flex_l    = calcMoms.f_T27(MA.hip_flex.l,Ft_hip_flex_l);
                g               = {g{:}, (T_hip_flex_l - outputF(jointi.hip_flex.l))./1};
                lbg             = [lbg; 0];
                ubg             = [ubg; 0];

                % Hip flexion, right
                Ft_hip_flex_r   = FTkj(mai(1).mus.r',1);
                T_hip_flex_r    = calcMoms.f_T27(MA.hip_flex.r,Ft_hip_flex_r);
                g               = {g{:}, (T_hip_flex_r - outputF(jointi.hip_flex.r))./1};
                lbg             = [lbg; 0];
                ubg             = [ubg; 0];

                % Hip adduction, left
                Ft_hip_add_l    = FTkj(mai(2).mus.l',1);
                T_hip_add_l     = calcMoms.f_T27(MA.hip_add.l,Ft_hip_add_l);
                g               = {g{:}, (T_hip_add_l - outputF(jointi.hip_add.l))./1};
                lbg             = [lbg; 0];
                ubg             = [ubg; 0];

                % Hip adduction, right
                Ft_hip_add_r    = FTkj(mai(2).mus.r',1);
                T_hip_add_r     = calcMoms.f_T27(MA.hip_add.r,Ft_hip_add_r);
                g               = {g{:}, (T_hip_add_r - outputF(jointi.hip_add.r))./1};
                lbg             = [lbg; 0];
                ubg             = [ubg; 0];

                % Hip rotation, left
                Ft_hip_rot_l    = FTkj(mai(3).mus.l',1);
                T_hip_rot_l     = calcMoms.f_T27(MA.hip_rot.l,Ft_hip_rot_l);
                g               = {g{:}, (T_hip_rot_l - outputF(jointi.hip_rot.l))./1};
                lbg             = [lbg; 0];
                ubg             = [ubg; 0];

                % Hip rotation, right
                Ft_hip_rot_r    = FTkj(mai(3).mus.r',1);
                T_hip_rot_r     = calcMoms.f_T27(MA.hip_rot.r,Ft_hip_rot_r);
                g               = {g{:}, (T_hip_rot_r - outputF(jointi.hip_rot.r))./1};
                lbg             = [lbg; 0];
                ubg             = [ubg; 0];

                % Knee, left
                Ft_knee_l       = FTkj(mai(4).mus.l',1);
                T_knee_l        = calcMoms.f_T13(MA.knee.l,Ft_knee_l);
                g               = {g{:}, (T_knee_l - outputF(jointi.knee.l))./1};
                lbg             = [lbg; 0];
                ubg             = [ubg; 0];

                % Knee, right
                Ft_knee_r       = FTkj(mai(4).mus.r',1);
                T_knee_r        = calcMoms.f_T13(MA.knee.r,Ft_knee_r);
                g               = {g{:}, (T_knee_r - outputF(jointi.knee.r))./1};
                lbg             = [lbg; 0];
                ubg             = [ubg; 0];

                % Ankle, left
                Ft_ankle_l      = FTkj(mai(5).mus.l',1);
                T_ankle_l       = calcMoms.f_T12(MA.ankle.l,Ft_ankle_l);
                g               = {g{:}, (T_ankle_l - outputF(jointi.ankle.l))./1};
                lbg             = [lbg; 0];
                ubg             = [ubg; 0];

                % Ankle, right
                Ft_ankle_r      = FTkj(mai(5).mus.r',1);
                T_ankle_r       = calcMoms.f_T12(MA.ankle.r,Ft_ankle_r);
                g               = {g{:}, (T_ankle_r - outputF(jointi.ankle.r))./1};
                lbg             = [lbg; 0];
                ubg             = [ubg; 0];

                % Subtalar, left
                Ft_subt_l       = FTkj(mai(6).mus.l',1);
                T_subt_l        = calcMoms.f_T12(MA.subt.l,Ft_subt_l);
                g               = {g{:}, (T_subt_l - outputF(jointi.subt.l))./1};
                lbg             = [lbg; 0];
                ubg             = [ubg; 0];

                % Subtalar, right
                Ft_subt_r       = FTkj(mai(6).mus.r',1);
                T_subt_r        = calcMoms.f_T12(MA.subt.r,Ft_subt_r);
                g               = {g{:}, (T_subt_r - outputF(jointi.subt.r))./1};
                lbg             = [lbg; 0];
                ubg             = [ubg; 0];

                % MTP, left
                Ft_mtp_l        = FTkj(mai(7).mus.l',1);
                T_mtp_l         = calcMoms.f_T4(MA.mtp.l,Ft_mtp_l);
                g               = {g{:}, (T_mtp_l - outputF(jointi.mtp.l) + uReserveskj_nsc{j}(1) - Options.MTP_stiff*Xkj_nsc{j}(jointi.mtp.l))./1};
                lbg             = [lbg; 0];
                ubg             = [ubg; 0];

                % MTP, right
                Ft_mtp_r        = FTkj(mai(7).mus.r',1);
                T_mtp_r         = calcMoms.f_T4(MA.mtp.r,Ft_mtp_r);
                g               = {g{:}, (T_mtp_r - outputF(jointi.mtp.r) + uReserveskj_nsc{j}(2) - Options.MTP_stiff*Xkj_nsc{j}(jointi.mtp.r))./1};
                lbg             = [lbg; 0];
                ubg             = [ubg; 0];

                % Lumbar extension
                Ft_trunk_ext    = FTkj([mai(8).mus.l,mai(8).mus.r]',1);
                T_trunk_ext     = calcMoms.f_T6(MA.trunk_ext,Ft_trunk_ext);
                g               = {g{:}, (T_trunk_ext - outputF(jointi.trunk.ext))./1};
                lbg             = [lbg; 0];
                ubg             = [ubg; 0];

                % Lumbar bending 
                Ft_trunk_ben    = FTkj([mai(9).mus.l,mai(9).mus.r]',1);
                T_trunk_ben     = calcMoms.f_T6(MA.trunk_ben,Ft_trunk_ben);
                g               = {g{:}, (T_trunk_ben - outputF(jointi.trunk.ben))./1};
                lbg             = [lbg; 0];
                ubg             = [ubg; 0];

                % Lumbar rotating
                Ft_trunk_rot    = FTkj([mai(10).mus.l,mai(10).mus.r]',1);
                T_trunk_rot     = calcMoms.f_T6(MA.trunk_rot,Ft_trunk_rot);
                g               = {g{:}, (T_trunk_rot - outputF(jointi.trunk.rot))./1};
                lbg             = [lbg; 0];
                ubg             = [ubg; 0];

%                 % Constrain maximum amplitude of thigh flexion angles
%                 g   = {g{:}, outputF([outInd.femur_r_XYZ(3),outInd.femur_l_XYZ(3)])};
%                 lbg = [lbg; -inf*ones(2,1)];  
%                 ubg = [ubg; deg2rad(140).*ones(2,1)];
                
                yGRFk_r = sum(outputF(outInd.r_contGRF(1)+1:3:outInd.r_contGRF(end)-1));
                J_v_GRF = J_v_GRF + exp(yGRFk_r/3000);
                
                % Constrain magnitude of distance between frames
                g   = {g{:}, outputF([outInd.tibia_l_calc_r_mag,outInd.tibia_r_calc_l_mag])};
                lbg = [lbg; 0.12*ones(2,1)];
                ubg = [ubg; 5*ones(2,1)];
                
                %g   = {g{:}, yGRFk_r};
                %lbg = [lbg; 0];
                %ubg = [ubg; 3200]; % was 50

                % Activation dynamics (implicit formulation)
                tact = 0.015;
                tdeact = 0.06;
                act1 = uActdotkj_nsc{j} + actkj_nsc{j}./(ones(size(actk_nsc,1),1)*tdeact);
                act2 = uActdotkj_nsc{j} + actkj_nsc{j}./(ones(size(actk_nsc,1),1)*tact);

                % act 1 path constraint
                g   = {g{:},act1};
                lbg = [lbg; zeros(NMuscle,1)];
                ubg = [ubg; inf*ones(NMuscle,1)];

                % act 2 path contraint
                g   = {g{:},act2};
                lbg = [lbg; -inf*ones(NMuscle,1)];
                ubg = [ubg; ones(NMuscle,1)./(ones(NMuscle,1)*tact)];

                % Contraction dynamics (implicit formulation)
                g   = {g{:},Hilldiffkj};
                lbg = [lbg; zeros(NMuscle,1)];
                ubg = [ubg; zeros(NMuscle,1)];

                % Add contribution to quadrature function
                % Experimental input first;
                % Simulation input second when appropriate
                J = J + wJ(1).*B(j+1)*(costFunctions.f_J_gPelOri(statesF.q_aux(k*(d+1)+1+j,1:3),Xkj_nsc{j}(1:3)))*h + ...
                        wJ(2).*B(j+1)*(costFunctions.f_J_gPelTra(statesF.q_aux(k*(d+1)+1+j,5:6),Xkj_nsc{j}(5:6)))*h + ...
                        wJ(3).*B(j+1)*(costFunctions.f_J_lljAngs(statesF.q_aux(k*(d+1)+1+j,[7:13,14:20]),Xkj_nsc{j}([7:13,14:20])))*h + ...
                        wJ(4).*B(j+1)*(costFunctions.f_J_uljAngs(statesF.q_aux(k*(d+1)+1+j,21:end),Xkj_nsc{j}(21:37)))*h + ...
                        wJ(5).*B(j+1)*(costFunctions.f_J_accs(uAcckj_nsc{j}))*h;

                J = J + wJ(6).*B(j+1)*(costFunctions.f_J_muscle_act(actkj_nsc{j}))*h + ...
                        wJ(7).*B(j+1)*(costFunctions.f_J_dmuscle_act(uActdotkj_nsc{j}))*h + ...
                        wJ(8).*B(j+1)*(costFunctions.f_J_FT(FTtildekj_nsc{j}))*h + ...
                        wJ(9).*B(j+1)*(costFunctions.f_J_dFT(dFTtildekj_nsc{j}))*h;

                J = J + wJ(10).*B(j+1)*(costFunctions.f_J_reserves(uReserveskj_nsc{j}))*h;
                J = J + wJ(11).*B(j+1)*(costFunctions.f_J_arms(armActskj_nsc{j}))*h;
                
                vertImp = vertImp + B(j+1)*sum(outputF(outInd.r_contGRF(1)+1:3:outInd.r_contGRF(end)-1))*h;

                % Counter function
                counter(k*d+j) = tanh((sum(outputF(outInd.r_contGRF(1)+1:3:outInd.r_contGRF(end)-1))) * 0.2);

                J_work_net = J_work_net + B(j+1)*outputF(11)*Xkj_nsc{j}(nq.all+11)*h;

                % Smooth function to return ideally 0 or 1; 
                % If the angle is (+) -> 0.5+0.5*tanh(q/0.005)
                % If the angle is (-) -> 0.5+0.5*-tanh(q/0.005)
                % The 0.5's shift the range of the function -> [0,1]
                smooth_output_ext = 0.5 + 0.5 * -tanh(Xkj_nsc{j}(11) / 0.005);
                smooth_output_fle = 0.5 + 0.5 * tanh(Xkj_nsc{j}(11) / 0.005);

                % Only positive extensor work
                J_work_net_pos_ext = J_work_net_pos_ext + B(j+1)*(smooth_output_ext*((1/1).*log(1+exp(1*1*(outputF(11).*Xkj_nsc{j}(nq.all+11)./72.2))))*1)*h;
                J_work_net_neg_ext = J_work_net_neg_ext + B(j+1)*(smooth_output_ext*((1/1).*log(1+exp(1*-1*(outputF(11).*Xkj_nsc{j}(nq.all+11)./72.2))))*-1)*h;
                J_work_net_pos_fle = J_work_net_pos_fle + B(j+1)*(smooth_output_fle*((1/1).*log(1+exp(1*1*(outputF(11).*Xkj_nsc{j}(nq.all+11)./72.2))))*1)*h;
                J_work_net_neg_fle = J_work_net_neg_fle + B(j+1)*(smooth_output_fle*((1/1).*log(1+exp(1*-1*(outputF(11).*Xkj_nsc{j}(nq.all+11)./72.2))))*-1)*h;




            end

            % Initial multibody dynamics matching constraint
            if k == 0

                g   = {g{:}, Xk_nsc_ini([1:3,7:37]) - [statesF.q_aux(1,[1:3,7:37])]'};
                lbg = [lbg; [-deg2rad(15).*ones(34,1); ]];
                ubg = [ubg; [deg2rad(15).*ones(34,1); ]];

                g   = {g{:}, Xk_nsc_ini(4)};
                lbg = [lbg; 0];
                ubg = [ubg; 0];
                
                yGRFk_r = sum(outputF_ini(outInd.r_contGRF(1)+1:3:outInd.r_contGRF(end)-1));
                g   = {g{:}, yGRFk_r};
                lbg = [lbg; 20];
                ubg = [ubg; 40]; % was 50
                
%                 g   = {g{:}, outputF_ini(outInd.r_toes_pos(1))-outputF_ini(outInd.posCOM(1)) - 0.359841099322690 - 0.08}; % and an offset (-) => longer; (+) => shorter
%                 lbg = [lbg; 0];
%                 ubg = [ubg; 0];

%                 g   = {g{:}, knee_pos_R_ini(1) - knee_pos_L_ini(1) - 0.125171649703219 + 0.15};
%                 lbg = [lbg; 0];
%                 ubg = [ubg; 0];

                if contains(file_ext, 'HTD')
                    if contains(file_ext,'Plus')
                        offset = str2double(file_ext(end)) / 100;
                        if offset == 0; offset = offset + 0.1; end
                        g   = {g{:}, outputF_ini(outInd.r_toes_pos(1))-outputF_ini(outInd.posCOM(1)) - 0.328890590509637 - offset}; % and an offset (-) => longer; (+) => shorter
                        lbg = [lbg; 0];
                        ubg = [ubg; 0];
                    else
                        offset = str2double(file_ext(end)) / 100;
                        if offset == 0; offset = offset + 0.1; end
                        g   = {g{:}, outputF_ini(outInd.r_toes_pos(1))-outputF_ini(outInd.posCOM(1)) - 0.328890590509637 + offset}; 
                        lbg = [lbg; 0];
                        ubg = [ubg; 0];
                    end
                end

                if contains(file_ext, 'IKTD')
                    if contains(file_ext,'Plus')
                        offset = str2double(file_ext(end)) / 100;
                        if offset == 0; offset = offset + 0.1; end
                        g   = {g{:}, knee_pos_R_ini(1) - knee_pos_L_ini(1) - 0.040824168566493 - offset};
                        lbg = [lbg; 0];
                        ubg = [ubg; 0];
                    else
                        offset = str2double(file_ext(end)) / 100;
                        if offset == 0; offset = offset + 0.1; end
                        g   = {g{:}, knee_pos_R_ini(1) - knee_pos_L_ini(1) - 0.040824168566493 + offset};
                        lbg = [lbg; 0];
                        ubg = [ubg; 0];
                    end
                end



            end

            % Symmetry constraints
            if k == Options.N-1

                Xk_nsc_fin = Xkj_nsc{j};
                actk_nsc_fin = actkj_nsc{j};
                FTtildek_nsc_fin = FTtildekj_nsc{j};
                armActsk_nsc_fin = armActskj_nsc{j};

                change_p_disp = Xk_nsc_fin(4) - Xk_nsc_ini(4);
%                 Hip
%                 g   = {g{:}, J_work_net_pos_ext};
%                 lbg = [lbg; 0];
%                 ubg = [ubg; (70.0617./72.2) * 0.8]; % 124.2960 J from nominal simulation

                % Ankle
                %g   = {g{:}, J_work_net_neg_ext};
                %lbg = [lbg; (-31.1244./72.2)*0.8];
                %ubg = [ubg; 0]; 

                %g   = {g{:}, J_work_net_pos_fle};
                %lbg = [lbg; 0];
                %ubg = [ubg; (63.3150./72.2)*0.8]; 
                
%                 g   = {g{:}, change_p_disp - 11.53793*finalTime_nsc}; % Matching speed - 9.81
%                 lbg = [lbg; 0];
%                 ubg = [ubg; 0];

                g   = {g{:}, Xk_nsc_fin(4)};
                lbg = [lbg; 0];
                ubg = [ubg; 2.47];

                for z = 1:length(counter)/3
                    for zz = 1:3
                        time_cont_vec(zz+(z-1)*3) = h * tau_root(zz+1)*counter(zz+(z-1)*3);
                    end
                end

                for z = 1:length(counter)/3
                    diff_cont(z) = time_cont_vec(z+(z-1)*2) + time_cont_vec(z+(z*2)-1) - time_cont_vec(z+(z-1)*2) + ...
                        time_cont_vec(z*3) - time_cont_vec(z+(z*2)-1);
                end

                contact_time = sum(diff_cont);

                F_ave = (72.2 * 9.81 * finalTime_nsc) / contact_time;

%                 g   = {g{:}, F_ave};
%                 lbg = [lbg; 0];
%                 ubg = [ubg; 2083.01 * 0.7];
% % % % 
% % % %                 g   = {g{:}, (finalTime_nsc - contact_time)*2 + contact_time};
% % % %                 lbg = [lbg; 0];
% % % %                 ubg = [ubg; 0.360]; % Based on data in Weyand et al. (2009)

                % Multibody dynamics symmetry

                % Positions
                g   = {g{:}, Xk_nsc_ini(7:13) - Xk_nsc_fin(14:20)};
                lbg = [lbg; zeros(7,1)];
                ubg = [ubg; zeros(7,1)];

                g   = {g{:}, Xk_nsc_ini(14:20) - Xk_nsc_fin(7:13)};
                lbg = [lbg; zeros(7,1)];
                ubg = [ubg; zeros(7,1)];

                g   = {g{:}, Xk_nsc_ini(5:6) - Xk_nsc_fin(5:6)};
                lbg = [lbg; zeros(2,1)];
                ubg = [ubg; zeros(2,1)];

                g   = {g{:}, Xk_nsc_ini([1,21]) - Xk_nsc_fin([1,21])};
                lbg = [lbg; zeros(2,1)];
                ubg = [ubg; zeros(2,1)];

                g   = {g{:}, Xk_nsc_ini([2:3,22:23]) + Xk_nsc_fin([2:3,22:23])};
                lbg = [lbg; zeros(4,1)];
                ubg = [ubg; zeros(4,1)];

                g   = {g{:}, Xk_nsc_ini(24:30) - Xk_nsc_fin(31:37)};
                lbg = [lbg; zeros(7,1)];
                ubg = [ubg; zeros(7,1)];

                g   = {g{:}, Xk_nsc_ini(31:37) - Xk_nsc_fin(24:30)};
                lbg = [lbg; zeros(7,1)];
                ubg = [ubg; zeros(7,1)];

                % Velocities
                g   = {g{:}, Xk_nsc_ini(44:50) - Xk_nsc_fin(51:57)};
                lbg = [lbg; zeros(7,1)];
                ubg = [ubg; zeros(7,1)];

                g   = {g{:}, Xk_nsc_ini(51:57) - Xk_nsc_fin(44:50)};
                lbg = [lbg; zeros(7,1)];
                ubg = [ubg; zeros(7,1)];

                g   = {g{:}, Xk_nsc_ini(41:42) - Xk_nsc_fin(41:42)};
                lbg = [lbg; zeros(2,1)];
                ubg = [ubg; zeros(2,1)];

                g   = {g{:}, Xk_nsc_ini(43) + Xk_nsc_fin(43)};
                lbg = [lbg; zeros(1,1)];
                ubg = [ubg; zeros(1,1)];

                g   = {g{:}, Xk_nsc_ini([38,58]) - Xk_nsc_fin([38,58])};
                lbg = [lbg; zeros(2,1)];
                ubg = [ubg; zeros(2,1)];

                g   = {g{:}, Xk_nsc_ini([39:40,59:60]) + Xk_nsc_fin([39:40,59:60])};
                lbg = [lbg; zeros(4,1)];
                ubg = [ubg; zeros(4,1)];

                g   = {g{:}, Xk_nsc_ini(61:67) - Xk_nsc_fin(68:74)};
                lbg = [lbg; zeros(7,1)];
                ubg = [ubg; zeros(7,1)];

                g   = {g{:}, Xk_nsc_ini(68:74) - Xk_nsc_fin(61:67)};
                lbg = [lbg; zeros(7,1)];
                ubg = [ubg; zeros(7,1)];

                % Muscle activations symmetry
                g   = {g{:}, actk_nsc_ini(1:NMuscle/2) - actk_nsc_fin(0.5*NMuscle+1:NMuscle)};
                lbg = [lbg; zeros(NMuscle/2,1)];
                ubg = [ubg; zeros(NMuscle/2,1)];

                % Muscle activations symmetry
                g   = {g{:}, actk_nsc_ini(0.5*NMuscle+1:NMuscle) - actk_nsc_fin(1:NMuscle/2)};
                lbg = [lbg; zeros(NMuscle/2,1)];
                ubg = [ubg; zeros(NMuscle/2,1)];

                % Tendon forces symmetry
                g   = {g{:}, FTtildek_nsc_ini(1:NMuscle/2) - FTtildek_nsc_fin(0.5*NMuscle+1:NMuscle)};
                lbg = [lbg; zeros(NMuscle/2,1)];
                ubg = [ubg; zeros(NMuscle/2,1)];

                % Tendon forces symmetry
                g   = {g{:}, FTtildek_nsc_ini(0.5*NMuscle+1:NMuscle) - FTtildek_nsc_fin(1:NMuscle/2)};
                lbg = [lbg; zeros(NMuscle/2,1)];
                ubg = [ubg; zeros(NMuscle/2,1)];

                % Arm Activations symmetry
                g   = {g{:}, armActsk_nsc_ini(1:nq.arms/2) - armActsk_nsc_fin((nq.arms/2)+1:end)};
                lbg = [lbg; zeros(nq.arms/2,1)];
                ubg = [ubg; zeros(nq.arms/2,1)];         

                % Arm Activations symmetry
                g   = {g{:}, armActsk_nsc_ini((nq.arms/2)+1:end) - armActsk_nsc_fin(1:nq.arms/2)};
                lbg = [lbg; zeros(nq.arms/2,1)];
                ubg = [ubg; zeros(nq.arms/2,1)];

                dFTtildek_nsc_fin  = dFTtildekj_nsc{j};
                uActdotk_nsc_fin   = uActdotkj_nsc{j};
                uAcck_nsc_fin      = uAcckj_nsc{j};
                uReserves_nsc_fin = uReserveskj_nsc{j};
                armExct_nsc_fin   = armExctkj_nsc{j};

                % Arm Excitations symmetry
                g   = {g{:}, armExct_nsc_ini(1:nq.arms/2) - armExct_nsc_fin((nq.arms/2)+1:end)};
                lbg = [lbg; zeros(nq.arms/2,1)];
                ubg = [ubg; zeros(nq.arms/2,1)];         

                % Arm Excitations symmetry
                g   = {g{:}, armExct_nsc_ini((nq.arms/2)+1:end) - armExct_nsc_fin(1:nq.arms/2)};
                lbg = [lbg; zeros(nq.arms/2,1)];
                ubg = [ubg; zeros(nq.arms/2,1)];

                % Derivative Tendon forces symmetry
                g   = {g{:}, dFTtilde_nsc_ini(1:NMuscle/2) - dFTtildek_nsc_fin(0.5*NMuscle+1:NMuscle)};
                lbg = [lbg; zeros(NMuscle/2,1)];
                ubg = [ubg; zeros(NMuscle/2,1)];

                % Derivative Tendon forces symmetry
                g   = {g{:}, dFTtilde_nsc_ini(0.5*NMuscle+1:NMuscle) - dFTtildek_nsc_fin(1:NMuscle/2)};
                lbg = [lbg; zeros(NMuscle/2,1)];
                ubg = [ubg; zeros(NMuscle/2,1)];

                % Derivative activations symmetry
                g   = {g{:}, uActdot_nsc_ini(1:NMuscle/2) - uActdotk_nsc_fin(0.5*NMuscle+1:NMuscle)};
                lbg = [lbg; zeros(NMuscle/2,1)];
                ubg = [ubg; zeros(NMuscle/2,1)];

                % Derivative activations symmetry
                g   = {g{:}, uActdot_nsc_ini(0.5*NMuscle+1:NMuscle) - uActdotk_nsc_fin(1:NMuscle/2)};
                lbg = [lbg; zeros(NMuscle/2,1)];
                ubg = [ubg; zeros(NMuscle/2,1)];

                % Reserves symmetry
                g   = {g{:}, uReserves_nsc_ini(1) - uReserves_nsc_fin(2)};
                lbg = [lbg; 0];
                ubg = [ubg; 0];

                % Reserves symmetry
                g   = {g{:}, uReserves_nsc_ini(2) - uReserves_nsc_fin(1)};
                lbg = [lbg; 0];
                ubg = [ubg; 0];

                % Accelerations
                g   = {g{:}, uAcc_nsc_ini(7:13) - uAcck_nsc_fin(14:20)};
                lbg = [lbg; zeros(7,1)];
                ubg = [ubg; zeros(7,1)];

                g   = {g{:}, uAcc_nsc_ini(14:20) - uAcck_nsc_fin(7:13)};
                lbg = [lbg; zeros(7,1)];
                ubg = [ubg; zeros(7,1)];

                g   = {g{:}, uAcc_nsc_ini(4:5) - uAcck_nsc_fin(4:5)};
                lbg = [lbg; zeros(2,1)];
                ubg = [ubg; zeros(2,1)];

                g   = {g{:}, uAcc_nsc_ini(6) + uAcck_nsc_fin(6)};
                lbg = [lbg; zeros(1,1)];
                ubg = [ubg; zeros(1,1)];

                g   = {g{:}, uAcc_nsc_ini([1,21]) - uAcck_nsc_fin([1,21])};
                lbg = [lbg; zeros(2,1)];
                ubg = [ubg; zeros(2,1)];

                g   = {g{:}, uAcc_nsc_ini([2:3,22:23]) + uAcck_nsc_fin([2:3,22:23])};
                lbg = [lbg; zeros(4,1)];
                ubg = [ubg; zeros(4,1)];

                g   = {g{:}, uAcc_nsc_ini(24:30) - uAcck_nsc_fin(31:37)};
                lbg = [lbg; zeros(7,1)];
                ubg = [ubg; zeros(7,1)];

                g   = {g{:}, uAcc_nsc_ini(31:37) - uAcck_nsc_fin(24:30)};
                lbg = [lbg; zeros(7,1)];
                ubg = [ubg; zeros(7,1)];



                % Reserves symmetry
                %g   = {g{:}, uReserves_nsc_ini([1,3,5,7,9,11,13]) - uReserves_nsc_fin([2,4,6,8,10,12,14])};
                %lbg = [lbg; zeros(7,1)];
                %ubg = [ubg; zeros(7,1)];
                %g   = {g{:}, uReserves_nsc_ini([1]) - uReserves_nsc_fin([2])};
                %lbg = [lbg; zeros(1,1)];
                %ubg = [ubg; zeros(1,1)];

                % Reserves symmetry
                %g   = {g{:}, uReserves_nsc_fin([1,3,5,7,9,11,13]) - uReserves_nsc_ini([2,4,6,8,10,12,14])};
                %lbg = [lbg; zeros(7,1)];
                %ubg = [ubg; zeros(7,1)];
                %g   = {g{:}, uReserves_nsc_fin([1]) - uReserves_nsc_ini([2])};
                %lbg = [lbg; zeros(1,1)];
                %ubg = [ubg; zeros(1,1)];

                % Reserves symmetry
                %g   = {g{:}, uReserves_nsc_ini(15) - uReserves_nsc_fin(15)};
                %lbg = [lbg; 0];
                %ubg = [ubg; 0];

                % Reserves symmetry
                %g   = {g{:}, uReserves_nsc_ini([16,17]) + uReserves_nsc_fin([16,17])};
                %lbg = [lbg; zeros(2,1)];
                %ubg = [ubg; zeros(2,1)];



            end

        end

    end

    function [outVars_sc,outVars_nsc] = sortVariables_muscle_multipleTrials(w0,nq,Options,scaling,d,interval,NMuscle,D_cont)
        
        x       = zeros(nq.all*2,(d+1)*Options.N); % Generalized coordinates & velocities
        act     = zeros(NMuscle,(d+1)*Options.N);  % Muscle activations
        FTtilde = zeros(NMuscle,(d+1)*Options.N);  % Normalised Tendon Forces
        armActs = zeros(nq.arms,(d+1)*Options.N);  % Arm Activations
        
        nReserves = 2;

        dFTtilde_d  = zeros(NMuscle,d*Options.N);
        uActdot_d   = zeros(NMuscle,d*Options.N);
        uAcc_d      = zeros(nq.all,d*Options.N);        
        uReserves_d = zeros(nReserves,d*Options.N);
        armExcts_d  = zeros(nq.arms,d*Options.N);
        
        dFTtilde = zeros(NMuscle,d*Options.N+1);
        uActdot   = zeros(NMuscle,d*Options.N+1);
        uAcc      = zeros(nq.all,d*Options.N+1);        
        uReserves = zeros(nReserves,d*Options.N+1);
        armExcts  = zeros(nq.arms,d*Options.N+1);
        
        for i = 0:Options.N-1
            
            x(:,i*(d+1)+1)        = w0(interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(NMuscle+NMuscle+nq.all+nReserves+nq.arms))+1:...
                interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(NMuscle+NMuscle+nq.all+nReserves+nq.arms))+nq.all*2);
            
            act(:,i*(d+1)+1)      = w0(interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(NMuscle+NMuscle+nq.all+nReserves+nq.arms))+nq.all*2+1:...
                interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(NMuscle+NMuscle+nq.all+nReserves+nq.arms))+nq.all*2+NMuscle);
            
            FTtilde(:,i*(d+1)+1)  = w0(interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(NMuscle+NMuscle+nq.all+nReserves+nq.arms))+nq.all*2+NMuscle+1:...
                interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(NMuscle+NMuscle+nq.all+nReserves+nq.arms))+nq.all*2+NMuscle+NMuscle);
            
            armActs(:,i*(d+1)+1)  = w0(interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(NMuscle+NMuscle+nq.all+nReserves+nq.arms))+nq.all*2+NMuscle+NMuscle+1:...
                interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(NMuscle+NMuscle+nq.all+nReserves+nq.arms))+nq.all*2+NMuscle+NMuscle+nq.arms);
            
            for r = 1:d
               
                x(:,r+1+i*(d+1)) = w0(interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(NMuscle+NMuscle+nq.all+nReserves+nq.arms))+nq.all*2+NMuscle*2+nq.arms+(r-1)*nq.all*2+1:...
                    interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(nq.all+NMuscle*2+nq.arms+nReserves))+nq.all*2+NMuscle+NMuscle+nq.arms+(r-1)*nq.all*2+nq.all*2);
                
            end
            
            for r = 1:d
               
                act(:,r+1+i*(d+1)) = w0(interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(NMuscle+NMuscle+nq.all+nReserves+nq.arms))+nq.all*2+NMuscle*2+nq.arms+d*(nq.all*2)+(r-1)*NMuscle+1:...
                    interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(nq.all+NMuscle*2+nq.arms+nReserves))+nq.all*2+NMuscle+NMuscle+nq.arms+d*(nq.all*2)+(r-1)*NMuscle+NMuscle);
                
            end
            
            for r = 1:d
               
                FTtilde(:,r+1+i*(d+1)) = w0(interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(NMuscle+NMuscle+nq.all+nReserves+nq.arms))+nq.all*2+NMuscle*2+nq.arms+d*(nq.all*2+NMuscle)+(r-1)*NMuscle+1:...
                    interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(nq.all+NMuscle*2+nq.arms+nReserves))+nq.all*2+NMuscle+NMuscle+nq.arms+d*(nq.all*2+NMuscle)+(r-1)*NMuscle+NMuscle);

            end
            
            for r = 1:d
               
                armActs(:,r+1+i*(d+1)) = w0(interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(NMuscle+NMuscle+nq.all+nReserves+nq.arms))+nq.all*2+NMuscle*2+nq.arms+d*(nq.all*2+NMuscle*2)+(r-1)*nq.arms+1:...
                    interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(nq.all+NMuscle*2+nq.arms+nReserves))+nq.all*2+NMuscle+NMuscle+nq.arms+d*(nq.all*2+NMuscle*2)+(r-1)*nq.arms+nq.arms);
                
            end
            
            for r = 1:d
               
                dFTtilde_d(:,r+i*d) = w0(interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(NMuscle+NMuscle+nq.all+nReserves+nq.arms))+nq.all*2+NMuscle*2+nq.arms+d*(nq.all*2+NMuscle*2+nq.arms)+(r-1)*NMuscle+1:...
                    interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(nq.all+NMuscle*2+nq.arms+nReserves))+nq.all*2+NMuscle+NMuscle+nq.arms+d*(nq.all*2+NMuscle*2+nq.arms)+(r-1)*NMuscle+NMuscle);
                
            end
            
            for r = 1:d
               
                uActdot_d(:,r+i*d) = w0(interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(NMuscle+NMuscle+nq.all+nReserves+nq.arms))+nq.all*2+NMuscle*2+nq.arms+d*(nq.all*2+NMuscle*2+nq.arms+NMuscle)+(r-1)*NMuscle+1:...
                    interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(nq.all+NMuscle*2+nq.arms+nReserves))+nq.all*2+NMuscle+NMuscle+nq.arms+d*(nq.all*2+NMuscle*2+nq.arms+NMuscle)+(r-1)*NMuscle+NMuscle);
                
            end
            
            for r = 1:d
               
                uAcc_d(:,r+i*d) = w0(interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(NMuscle+NMuscle+nq.all+nReserves+nq.arms))+nq.all*2+NMuscle*2+nq.arms+d*(nq.all*2+NMuscle*2+nq.arms+NMuscle*2)+(r-1)*nq.all+1:...
                    interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(nq.all+NMuscle*2+nq.arms+nReserves))+nq.all*2+NMuscle+NMuscle+nq.arms+d*(nq.all*2+NMuscle*2+nq.arms+NMuscle*2)+(r-1)*nq.all+nq.all);
                
            end
            
            for r = 1:d
               
                uReserves_d(:,r+i*d) = w0(interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(NMuscle+NMuscle+nq.all+nReserves+nq.arms))+nq.all*2+NMuscle*2+nq.arms+d*(nq.all*2+NMuscle*2+nq.arms+NMuscle*2+nq.all)+(r-1)*nReserves+1:...
                    interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(nq.all+NMuscle*2+nq.arms+nReserves))+nq.all*2+NMuscle+NMuscle+nq.arms+d*(nq.all*2+NMuscle*2+nq.arms+NMuscle*2+nq.all)+(r-1)*nReserves+nReserves);
                
            end
            
            for r = 1:d
               
                armExcts_d(:,r+i*d) = w0(interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(NMuscle+NMuscle+nq.all+nReserves+nq.arms))+nq.all*2+NMuscle*2+nq.arms+d*(nq.all*2+NMuscle*2+nq.arms+NMuscle*2+nq.all+nReserves)+(r-1)*nq.arms+1:...
                    interval+i*((d+1)*(nq.all*2+NMuscle*2+nq.arms)+d*(nq.all+NMuscle*2+nq.arms+nReserves))+nq.all*2+NMuscle+NMuscle+nq.arms+d*(nq.all*2+NMuscle*2+nq.arms+NMuscle*2+nq.all+nReserves)+(r-1)*nq.arms+nq.arms);
                
            end
            
        end
        
        % Extrapolated control points for beginning of mesh interval
        dFTtilde  = [dFTtilde_d(:,1:3)*D_cont dFTtilde_d];
        uActdot   = [uActdot_d(:,1:3)*D_cont uActdot_d];
        uAcc      = [uAcc_d(:,1:3)*D_cont uAcc_d];
        uReserves = [uReserves_d(:,1:3)*D_cont uReserves_d];
        armExcts  = [armExcts_d(:,1:3)*D_cont armExcts_d];
        
        outVars_sc.q        = x(1:nq.all,:);
        outVars_sc.qdot     = x(nq.all+1:end,:);
        outVars_sc.act      = act;
        outVars_sc.FTtilde  = FTtilde;
        outVars_sc.armActs  = armActs;
        outVars_sc.dFTtilde = dFTtilde;
        outVars_sc.armExcts = armExcts;
        outVars_sc.uActdot  = uActdot;
        outVars_sc.uAcc     = uAcc;
        
        outVars_sc.uReserves = uReserves;
        
        outVars_sc.totalTime = w0(end);
        
        outVars_nsc.q        = x(1:nq.all,:).*scaling.q;
        outVars_nsc.qdot     = x(nq.all+1:end,:).*scaling.qdot;
        outVars_nsc.act      = act.*scaling.act';
        outVars_nsc.FTtilde  = FTtilde.*scaling.FTtilde';
        outVars_nsc.armActs  = armActs;
        outVars_nsc.dFTtilde = dFTtilde.*scaling.dFTtilde';
        outVars_nsc.armExcts = armExcts;
        outVars_nsc.uActdot  = uActdot.*scaling.uActdot';
        outVars_nsc.uAcc     = uAcc.*scaling.uAcc;
        
        outVars_nsc.uReserves = uReserves.*scaling.uReserves;
        
        outVars_nsc.totalTime = (w0(end)-scaling.totalTime(2))/scaling.totalTime(1);
        
    end

    function optimumOutput = saveOptimumFiles(scaling,Options,optVars_sc,optVars_nsc,timeNodes,timeGrid,...
        statesF,weightJ,statistics,statesFname,stateNames,pathResults,outInd,termsJ,moments,muscleValues,modelCOM,GRFs,bodyAngles,angMom,musclesToPrint,GRF_individual)

    % Create data structures to store outputs
    optimumOutput.scaling       = scaling;
    optimumOutput.options       = Options;
    optimumOutput.optVars_sc    = optVars_sc;
    optimumOutput.optVars_nsc   = optVars_nsc;
    optimumOutput.timeNodes     = timeNodes;
    optimumOutput.timeGrid      = timeGrid;
    optimumOutput.objFunWeights = weightJ;
    optimumOutput.objContribs   = termsJ;
    optimumOutput.muscleMoments = moments;
    optimumOutput.muscleValues  = muscleValues;
    optimumOutput.stats         = statistics;
    optimumOutput.outInd        = outInd;
    optimumOutput.modelCOM      = modelCOM;
    optimumOutput.GRFs.R       = GRFs.R;
    optimumOutput.GRFs.L       = GRFs.L;
    optimumOutput.GRF_individual = GRF_individual;
    optimumOutput.ave_speed    = (optVars_nsc.q(4,end)-optVars_nsc.q(4,1))./optVars_nsc.totalTime;

    optimumOutput.lam_x_opt = lam_x_opt;
    optimumOutput.lam_g_opt = lam_g_opt;
    
    optimumOutput.bodyAngles.femur_r = rad2deg(bodyAngles.femur_r);
    optimumOutput.bodyAngles.femur_l = rad2deg(bodyAngles.femur_l);
    optimumOutput.bodyAngles.tibia_r = rad2deg(bodyAngles.tibia_r);
    optimumOutput.bodyAngles.tibia_l = rad2deg(bodyAngles.tibia_l);
    optimumOutput.bodyAngles.torso   = rad2deg(bodyAngles.torso);

    % Print optimised Qs for each trial as a .mot file
    q_deg = optVars_nsc.q;
    q_deg([1:3,7:nq.all],:) = rad2deg(q_deg([1:3,7:nq.all],:));
    motData.labels = stateNames(1,1:nq.all+1);
    motData.data   = [timeGrid q_deg'];

    save([pathResults 'IC_pred_Sprinting_optimum_' erase(statesFname,'.mot') '_' datestr(now,'dd-mmmm-yyyy__HH-MM-SS') '__' file_ext '.mat'],'optimumOutput');
    write_motionFile(motData,[pathResults 'IC_pred_Sprinting_optimum_' datestr(now,'dd-mmmm-yyyy__HH-MM-SS') '__' file_ext '_' statesFname]);
        
    stoData.labels = cat(1,[stateNames{1,1} musclesToPrint]);
    stoData.data   = [timeGrid optVars_nsc.act'];
    
    write_storageFile(stoData,[pathResults 'IC_pred_Sprinting_acts_optimum_' datestr(now,'dd-mmmm-yyyy__HH-MM-SS') '__' file_ext '.sto']);

    createExternalForces_visualization([pathResults 'IC_pred_Sprinting_optimum_' datestr(now,'dd-mmmm-yyyy__HH-MM-SS') '_' erase(statesFname,'.mot') '__' file_ext '_GRF'],GRF_individual,timeNodes);

    end

    function [termsJ,moments,muscleValues,modelCOM,GRFs,bodyAngles,angMom,GRF_individual] = calcObjFuncTerms(wJ,B,h,Options,nq,optVars_nsc,d,statesF,costFunctions,F,NMuscle,contactParameters,vMaxMult,oMFL_2_nsc,TSL_2_nsc,preOpt)

        import casadi.*
    
        J_pel_q  = 0;
        J_pel_t  = 0;
        J_kin_ll = 0;
        J_kin_ul = 0;
        J_accs   = 0;
        J_act    = 0;
        J_dAct   = 0;
        J_ftTil  = 0;
        J_dftTil = 0;
        J_reserv = 0;
        J_arms   = 0;
        J_time   = -wJ(12).*((optVars_nsc.q(4,end)-optVars_nsc.q(4,1))./optVars_nsc.totalTime);

        J_work_net_hip_l = 0;
        J_work_net_pos_ext_hip_l = 0;
        J_work_net_neg_ext_hip_l = 0;
        J_work_net_pos_fle_hip_l = 0;
        J_work_net_neg_fle_hip_l = 0;

        J_work_net_hip_r = 0;
        J_work_net_pos_ext_hip_r = 0;
        J_work_net_neg_ext_hip_r = 0;
        J_work_net_pos_fle_hip_r = 0;
        J_work_net_neg_fle_hip_r = 0;

        J_work_net_knee_l = 0;
        J_work_net_pos_ext_knee_l = 0;
        J_work_net_neg_ext_knee_l = 0;
        J_work_net_pos_fle_knee_l = 0;
        J_work_net_neg_fle_knee_l = 0;

        J_work_net_knee_r = 0;
        J_work_net_pos_ext_knee_r = 0;
        J_work_net_neg_ext_knee_r = 0;
        J_work_net_pos_fle_knee_r = 0;
        J_work_net_neg_fle_knee_r = 0;

        J_work_net_ankle_l = 0;
        J_work_net_pos_ext_ankle_l = 0;
        J_work_net_neg_ext_ankle_l = 0;
        J_work_net_pos_fle_ankle_l = 0;
        J_work_net_neg_fle_ankle_l = 0;

        J_work_net_ankle_r = 0;
        J_work_net_pos_ext_ankle_r = 0;
        J_work_net_neg_ext_ankle_r = 0;
        J_work_net_pos_fle_ankle_r = 0;
        J_work_net_neg_fle_ankle_r = 0;

        termsJ.counter = zeros(Options.N*d,1);
        time_cont_vec = zeros(Options.N*d,1);
        diff_cont = zeros(Options.N,1);
        
        dFTtilde  = optVars_nsc.dFTtilde(:,2:end);
        uActdot   = optVars_nsc.uActdot(:,2:end);
        uAcc      = optVars_nsc.uAcc(:,2:end);
        uReserves = optVars_nsc.uReserves(:,2:end);
        armExct   = optVars_nsc.armExcts(:,2:end);
        
        for k = 0:Options.N-1
            
            Xk_nsc       = [optVars_nsc.q(:,1+k*(d+1)); optVars_nsc.qdot(:,1+k*(d+1))];
            actk_nsc     = [optVars_nsc.act(:,1+k*(d+1))];
            FTtildek_nsc = [optVars_nsc.FTtilde(:,1+k*(d+1))];
            armActsk_nsc = [optVars_nsc.armActs(:,1+k*(d+1))];
            
            if k == 0
                Xk_nsc_ini       = Xk_nsc;                
                actk_nsc_ini     = actk_nsc;
                FTtildek_nsc_ini = FTtildek_nsc;
                armActsk_nsc_ini = armActsk_nsc; 
            end
            
            Xkj_nsc = {};
            for j=1:d
                Xkj_nsc{j} = [optVars_nsc.q(:,k*(d+1)+1+j); optVars_nsc.qdot(:,k*(d+1)+1+j)];
            end
            
            actkj_nsc = {};
            for j=1:d
                actkj_nsc{j} = [optVars_nsc.act(:,k*(d+1)+1+j)];
            end
            
            FTtildekj_nsc = {};
            for j=1:d
                FTtildekj_nsc{j} = [optVars_nsc.FTtilde(:,k*(d+1)+1+j)];
            end
            
            armActskj_nsc = {};
            for j=1:d
                armActskj_nsc{j} = [optVars_nsc.armActs(:,k*(d+1)+1+j)];
            end
            
            dFTtildekj_nsc = {};
            for j=1:d
                dFTtildekj_nsc{j} = [dFTtilde(:,k*d+j)];
            end
            
            uActdotkj_nsc = {};
            for j=1:d
                uActdotkj_nsc{j} = [uActdot(:,k*d+j)];
            end
            
            uAcckj_nsc = {};
            for j=1:d
                uAcckj_nsc{j} = [uAcc(:,k*d+j)];
            end
            
            uReserveskj_nsc = {};
            for j=1:d
                uReserveskj_nsc{j} = [uReserves(:,k*d+j)];
            end
            
            armExctkj_nsc = {};
            for j=1:d
                armExctkj_nsc{j} = [armExct(:,k*d+j)];
            end
            
                    if k == 0
                        
                        dFTtilde_nsc_ini  = optVars_nsc.dFTtilde(:,1);
                        uActdot_nsc_ini   = optVars_nsc.uActdot(:,1);
                        uAcc_nsc_ini      = optVars_nsc.uAcc(:,1);
                        uReserves_nsc_ini = optVars_nsc.uReserves(:,1);
                        armExct_nsc_ini   = optVars_nsc.armExcts(:,1);

                        Xk_nsc_ini_dummy = zeros(nq.all*2,1);
                        Xk_nsc_ini_dummy([1,21])      = optVars_nsc.q([1,21],end);
                        Xk_nsc_ini_dummy([2:3,22:23]) = optVars_nsc.q([2:3,22:23],end) * -1; 
                        Xk_nsc_ini_dummy(5:6)         = optVars_nsc.q(5:6,end);
                        Xk_nsc_ini_dummy(7:13)        = optVars_nsc.q(14:20,end);
                        Xk_nsc_ini_dummy(14:20)        = optVars_nsc.q(7:13,end);
                        Xk_nsc_ini_dummy(24:30)        = optVars_nsc.q(31:37,end);
                        Xk_nsc_ini_dummy(31:37)        = optVars_nsc.q(24:30,end);

                        Xk_nsc_ini_dummy(nq.all+[1,21])      = optVars_nsc.qdot([1,21],end);
                        Xk_nsc_ini_dummy(nq.all+[2:3,22:23]) = optVars_nsc.qdot([2:3,22:23],end) * -1; 
                        Xk_nsc_ini_dummy(nq.all+(4:6))         = optVars_nsc.qdot(4:6,end);
                        Xk_nsc_ini_dummy(nq.all+(7:13))        = optVars_nsc.qdot(14:20,end);
                        Xk_nsc_ini_dummy(nq.all+(14:20))        = optVars_nsc.qdot(7:13,end);
                        Xk_nsc_ini_dummy(nq.all+(24:30))        = optVars_nsc.qdot(31:37,end);
                        Xk_nsc_ini_dummy(nq.all+(31:37))        = optVars_nsc.qdot(24:30,end);

                        uAcc_nsc_ini_dummy = zeros(nq.all,1);
                        uAcc_nsc_ini_dummy([1,21])      = optVars_nsc.uAcc([1,21],end);
                        uAcc_nsc_ini_dummy([2:3,22:23]) = optVars_nsc.uAcc([2:3,22:23],end) * -1; 
                        uAcc_nsc_ini_dummy(4:6)         = optVars_nsc.uAcc(4:6,end);
                        uAcc_nsc_ini_dummy(7:13)        = optVars_nsc.uAcc(14:20,end);
                        uAcc_nsc_ini_dummy(14:20)        = optVars_nsc.uAcc(7:13,end);
                        uAcc_nsc_ini_dummy(24:30)        = optVars_nsc.uAcc(31:37,end);
                        uAcc_nsc_ini_dummy(31:37)        = optVars_nsc.uAcc(24:30,end);                        


                        % Reorder the skeleton states as q1 u1 q2 u2...
                        % For compatibility with F function
                        Xk_nsc_ORD = zeros(nq.all*2,1);
                        for i = 0:nq.all-1
                            Xk_nsc_ORD(i+1*(i+1),1) = Xk_nsc_ini(i+1,1);
                            Xk_nsc_ORD(2*(i+1),1)   = Xk_nsc_ini(nq.all+i+1,1);
                        end
                        
                        % Evaluate F Function 
                        % Evaluation returns residuals, joint torques 
                        % and GRFs from contact model
                        % Output used later
                        outputF = F([Xk_nsc_ORD;uAcc_nsc_ini;contPrms_nsc ]);
                        
                        % Contact model GRFs; right then left
                        xGRFk_r(1,1) = sum(full(outputF(outInd.r_contGRF(1):3:outInd.r_contGRF(end)-2)));
                        yGRFk_r(1,1) = sum(full(outputF(outInd.r_contGRF(1)+1:3:outInd.r_contGRF(end)-1)));
                        zGRFk_r(1,1) = sum(full(outputF(outInd.r_contGRF(1)+2:3:outInd.r_contGRF(end))));            

                        xGRFk_l(1,1) = sum(full(outputF(outInd.l_contGRF(1):3:outInd.l_contGRF(end)-2)));
                        yGRFk_l(1,1) = sum(full(outputF(outInd.l_contGRF(1)+1:3:outInd.l_contGRF(end)-1)));
                        zGRFk_l(1,1) = sum(full(outputF(outInd.l_contGRF(1)+2:3:outInd.l_contGRF(end))));

                        r_sph_1_G(:,1) = full(outputF(outInd.r_sph_1_G_XYZ));
                        r_sph_2_G(:,1) = full(outputF(outInd.r_sph_2_G_XYZ));
                        r_sph_3_G(:,1) = full(outputF(outInd.r_sph_3_G_XYZ));
                        r_sph_4_G(:,1) = full(outputF(outInd.r_sph_4_G_XYZ));
                        r_sph_5_G(:,1) = full(outputF(outInd.r_sph_5_G_XYZ));
                        r_sph_6_G(:,1) = full(outputF(outInd.r_sph_6_G_XYZ));
                        r_sph_7_G(:,1) = full(outputF(outInd.r_sph_7_G_XYZ));
                        l_sph_1_G(:,1) = full(outputF(outInd.l_sph_1_G_XYZ));
                        l_sph_2_G(:,1) = full(outputF(outInd.l_sph_2_G_XYZ));
                        l_sph_3_G(:,1) = full(outputF(outInd.l_sph_3_G_XYZ));
                        l_sph_4_G(:,1) = full(outputF(outInd.l_sph_4_G_XYZ));
                        l_sph_5_G(:,1) = full(outputF(outInd.l_sph_5_G_XYZ));
                        l_sph_6_G(:,1) = full(outputF(outInd.l_sph_6_G_XYZ));
                        l_sph_7_G(:,1) = full(outputF(outInd.l_sph_7_G_XYZ));

                        r_sph_F_1_G(:,1) = full(outputF(outInd.r_contGRF(1):outInd.r_contGRF(3)));
                        r_sph_F_2_G(:,1) = full(outputF(outInd.r_contGRF(4):outInd.r_contGRF(6)));
                        r_sph_F_3_G(:,1) = full(outputF(outInd.r_contGRF(7):outInd.r_contGRF(9)));
                        r_sph_F_4_G(:,1) = full(outputF(outInd.r_contGRF(10):outInd.r_contGRF(12)));
                        r_sph_F_5_G(:,1) = full(outputF(outInd.r_contGRF(13):outInd.r_contGRF(15)));
                        r_sph_F_6_G(:,1) = full(outputF(outInd.r_contGRF(16):outInd.r_contGRF(18)));
                        r_sph_F_7_G(:,1) = full(outputF(outInd.r_contGRF(19):outInd.r_contGRF(21)));
                        l_sph_F_1_G(:,1) = full(outputF(outInd.l_contGRF(1):outInd.l_contGRF(3)));
                        l_sph_F_2_G(:,1) = full(outputF(outInd.l_contGRF(4):outInd.l_contGRF(6)));
                        l_sph_F_3_G(:,1) = full(outputF(outInd.l_contGRF(7):outInd.l_contGRF(9)));
                        l_sph_F_4_G(:,1) = full(outputF(outInd.l_contGRF(10):outInd.l_contGRF(12)));
                        l_sph_F_5_G(:,1) = full(outputF(outInd.l_contGRF(13):outInd.l_contGRF(15)));
                        l_sph_F_6_G(:,1) = full(outputF(outInd.l_contGRF(16):outInd.l_contGRF(18)));
                        l_sph_F_7_G(:,1) = full(outputF(outInd.l_contGRF(19):outInd.l_contGRF(21)));

                        % Pelvis residuals & joint moments
                        moments.joints(:,1) = full(outputF(1:nq.all));

                        % Calculate joint power
                        moments.power(:,1) = moments.joints(:,1).*Xk_nsc_ini(nq.all+1:end,1);

                        % Model CoM Outputs
                        modelCOM.pos(:,1) = full(outputF(outInd.posCOM));
                        modelCOM.vel(:,1) = full(outputF(outInd.velCOM));
                        modelCOM.r_toes_pos(:,1) = full(outputF(outInd.r_toes_pos));
                        modelCOM.r_toes_vel(:,1) = full(outputF(outInd.r_toes_vel));

                        modelCOM.r_knee_pos(:,1) = full(outputF(outInd.knee_r_pos_XYZ));
                        modelCOM.l_knee_pos(:,1) = full(outputF(outInd.knee_l_pos_XYZ));

                        % Body Angles
                        bodyAngles.femur_r(:,1) = full(outputF(outInd.femur_r_XYZ));
                        bodyAngles.femur_l(:,1) = full(outputF(outInd.femur_l_XYZ));
                        bodyAngles.tibia_r(:,1) = full(outputF(outInd.tibia_r_XYZ));
                        bodyAngles.tibia_l(:,1) = full(outputF(outInd.tibia_l_XYZ));
                        bodyAngles.torso(:,1)   = full(outputF(outInd.torso_XYZ));
                        
                        % Angular Momentum
                        angMom.femur_r(:,1)     = full(outputF(outInd.femur_r_H_COM));
                        angMom.femur_l(:,1)     = full(outputF(outInd.femur_l_H_COM));
                        angMom.tibia_r(:,1)     = full(outputF(outInd.tibia_r_H_COM));
                        angMom.tibia_l(:,1)     = full(outputF(outInd.tibia_l_H_COM));
                        angMom.talus_r(:,1)     = full(outputF(outInd.talus_r_H_COM));
                        angMom.talus_l(:,1)     = full(outputF(outInd.talus_l_H_COM));
                        angMom.calcn_r(:,1)     = full(outputF(outInd.calcn_r_H_COM));
                        angMom.calcn_l(:,1)     = full(outputF(outInd.calcn_l_H_COM));
                        angMom.toes_r(:,1)     = full(outputF(outInd.toes_r_H_COM));
                        angMom.toes_l(:,1)     = full(outputF(outInd.toes_l_H_COM));
                        angMom.humerus_r(:,1)     = full(outputF(outInd.humerus_r_H_COM));
                        angMom.humerus_l(:,1)     = full(outputF(outInd.humerus_l_H_COM));
                        angMom.radius_r(:,1)     = full(outputF(outInd.radius_r_H_COM));
                        angMom.radius_l(:,1)     = full(outputF(outInd.radius_l_H_COM));
                        angMom.ulna_r(:,1)     = full(outputF(outInd.ulna_r_H_COM));
                        angMom.ulna_l(:,1)     = full(outputF(outInd.ulna_l_H_COM));
                        angMom.hand_r(:,1)     = full(outputF(outInd.hand_r_H_COM));
                        angMom.hand_l(:,1)     = full(outputF(outInd.hand_l_H_COM));
                        angMom.pelvis(:,1)     = full(outputF(outInd.pelvis_H_COM));
                        angMom.torso(:,1)     = full(outputF(outInd.torso_H_COM));

                        % Get MTU lengths, MTU velocities & moment arms
                        % First left leg
                        qin_l = [Xk_nsc(jointi.hip_flex.l),...
                            Xk_nsc(jointi.hip_add.l),...
                            Xk_nsc(jointi.hip_rot.l),...
                            Xk_nsc(jointi.knee.l),...
                            Xk_nsc(jointi.ankle.l),...
                            Xk_nsc(jointi.subt.l),...
                            Xk_nsc(jointi.mtp.l),...
                            Xk_nsc(jointi.trunk.ext),...
                            Xk_nsc(jointi.trunk.ben),...
                            Xk_nsc(jointi.trunk.rot)];
                        qdotin_l = [Xk_nsc(jointi.hip_flex.l+nq.all),...
                            Xk_nsc(jointi.hip_add.l+nq.all),...
                            Xk_nsc(jointi.hip_rot.l+nq.all),...
                            Xk_nsc(jointi.knee.l+nq.all),...
                            Xk_nsc(jointi.ankle.l+nq.all),...
                            Xk_nsc(jointi.subt.l+nq.all),...
                            Xk_nsc(jointi.mtp.l+nq.all),...
                            Xk_nsc(jointi.trunk.ext+nq.all),...
                            Xk_nsc(jointi.trunk.ben+nq.all),...
                            Xk_nsc(jointi.trunk.rot+nq.all)];
                        [lMTk_l,vMTk_l,MA_l] = f_lMT_vMT_dM(qin_l,qdotin_l);
                        MA.hip_flex.l   = MA_l(mai(1).mus.l',1);
                        MA.hip_add.l    = MA_l(mai(2).mus.l',2);
                        MA.hip_rot.l    = MA_l(mai(3).mus.l',3);
                        MA.knee.l       = MA_l(mai(4).mus.l',4);
                        MA.ankle.l      = MA_l(mai(5).mus.l',5);
                        MA.subt.l       = MA_l(mai(6).mus.l',6);
                        MA.mtp.l        = MA_l(mai(7).mus.l',7);

                        % For the back muscles, we want left & right together:
                        % left first, right second. In MuscleInfo, we first have right
                        % back muscles (44:46) & then left back muscles (47:49). Since 
                        % back muscles only depend on back DOFs, we do not care if we 
                        % extract from "left leg or right leg"; so here we just pick left
                        MA.trunk_ext    = MA_l([47:49,mai(8).mus.l]',8);
                        MA.trunk_ben    = MA_l([47:49,mai(9).mus.l]',9);
                        MA.trunk_rot    = MA_l([47:49,mai(10).mus.l]',10);

                        % Second right leg
                        qin_r = [Xk_nsc(jointi.hip_flex.r),...
                            Xk_nsc(jointi.hip_add.r),...
                            Xk_nsc(jointi.hip_rot.r),...
                            Xk_nsc(jointi.knee.r),...
                            Xk_nsc(jointi.ankle.r),...
                            Xk_nsc(jointi.subt.r),...
                            Xk_nsc(jointi.mtp.r),...
                            Xk_nsc(jointi.trunk.ext),...
                            Xk_nsc(jointi.trunk.ben),...
                            Xk_nsc(jointi.trunk.rot)];
                        qdotin_r = [Xk_nsc(jointi.hip_flex.r+nq.all),...
                            Xk_nsc(jointi.hip_add.r+nq.all),...
                            Xk_nsc(jointi.hip_rot.r+nq.all),...
                            Xk_nsc(jointi.knee.r+nq.all),...
                            Xk_nsc(jointi.ankle.r+nq.all),...
                            Xk_nsc(jointi.subt.r+nq.all),...
                            Xk_nsc(jointi.mtp.r+nq.all),...
                            Xk_nsc(jointi.trunk.ext+nq.all),...
                            Xk_nsc(jointi.trunk.ben+nq.all),...
                            Xk_nsc(jointi.trunk.rot+nq.all)];
                        [lMTk_r,vMTk_r,MA_r] = f_lMT_vMT_dM(qin_r,qdotin_r);
                        % Here we take the indices from left since the vector is 1:49
                        % MA_r: 49 x 10
                        % mai: is created starting with left muscles then right muscles
                        % Querying MA_r with 'mus.r' would result in an error
                        MA.hip_flex.r   = MA_r(mai(1).mus.l',1);
                        MA.hip_add.r    = MA_r(mai(2).mus.l',2);
                        MA.hip_rot.r    = MA_r(mai(3).mus.l',3);
                        MA.knee.r       = MA_r(mai(4).mus.l',4);
                        MA.ankle.r      = MA_r(mai(5).mus.l',5);
                        MA.subt.r       = MA_r(mai(6).mus.l',6);
                        MA.mtp.r        = MA_r(mai(7).mus.l',7);

                        % Both legs
                        % In MuscleInfo, we first have the right back muscles (44:46)
                        % and then the left back muscles (47:49). We reorganise so that
                        % we first have the left muscles, then the right muscles
                        lMTk_lr = [lMTk_l([1:43,47:49],1);lMTk_r(1:46,1)];
                        vMTk_lr = [vMTk_l([1:43,47:49],1);vMTk_r(1:46,1)];
                        
                        % Moment Arms, MTU-length, MTU-velocity
                        muscleValues.MA.hip_flex.l(:,1) = full(MA.hip_flex.l);
                        muscleValues.MA.hip_flex.r(:,1) = full(MA.hip_flex.r);

                        muscleValues.MA.hip_add.l(:,1) = full(MA.hip_add.l);
                        muscleValues.MA.hip_add.r(:,1) = full(MA.hip_add.r);

                        muscleValues.MA.hip_rot.l(:,1) = full(MA.hip_rot.l);
                        muscleValues.MA.hip_rot.r(:,1) = full(MA.hip_rot.r);

                        muscleValues.MA.knee.l(:,1) = full(MA.knee.l);
                        muscleValues.MA.knee.r(:,1) = full(MA.knee.r);

                        muscleValues.MA.ankle.l(:,1) = full(MA.ankle.l);
                        muscleValues.MA.ankle.r(:,1) = full(MA.ankle.r);

                        muscleValues.MA.subt.l(:,1) = full(MA.subt.l);
                        muscleValues.MA.subt.r(:,1) = full(MA.subt.r);

                        muscleValues.MA.mtp.l(:,1) = full(MA.mtp.l);
                        muscleValues.MA.mtp.r(:,1) = full(MA.mtp.r);

                        muscleValues.MA.trunk_ext(:,1) = full(MA.trunk_ext);
                        muscleValues.MA.trunk_ben(:,1) = full(MA.trunk_ben);
                        muscleValues.MA.trunk_rot(:,1) = full(MA.trunk_rot);

                        muscleValues.lMTk_lr(:,1) = full(lMTk_lr);
                        muscleValues.vMTk_lr(:,1) = full(vMTk_lr);
                        
                        % Get the non-normalised unscaled tendon forces & 
                        % Hill equilibrium evaluation (normalised unscaled)
                        % Outputs are left leg & back muscles, then right leg & back muscles
                        [Hilldiff_ini,FT_ini,Fce_ini,Fpass_ini,Fiso_ini,vMax_ini,lTtilde_ini,lM_ini,lMtilde_ini,FMvtilde_ini,vMtilde_ini,Fpetilde_ini] = ...
                            f_forceEquilibrium_FtildeState_all_tendon_M(actk_nsc_ini,FTtildek_nsc_ini,...
                            dFTtilde_nsc_ini,lMTk_lr,vMTk_lr,tensions,aTendon,shift,oMFL_2_nsc,TSL_2_nsc,oMFL_2_nsc.*vMaxMult);
                        
                        muscleValues.Hilldiff(:,1) = full(Hilldiff_ini); % (Fcetilde+Fpetilde)*cos(alpha)-FTtilde
                        muscleValues.FT(:,1)       = full(FT_ini);       % Tendon Force (Newtons)
                        muscleValues.Fce(:,1)      = full(Fce_ini);      % act*FMltilde*FMvtilde + d*vMtilde (Newtons)
                        muscleValues.Fpass(:,1)    = full(Fpass_ini);    % Passive Muscle Force (Newtons)
                        muscleValues.Fiso(:,1)     = full(Fiso_ini);     % FMltilde - normalised force-length multiplier
                        muscleValues.vMax(:,1)     = full(vMax_ini);     % Max shortening velocity; i.e. 12 opt musc fibre lengths
                        muscleValues.lTtildek(:,1) = full(lTtilde_ini);  % Normalised tendon length
                        muscleValues.lM(:,1)       = full(lM_ini);       % Muscle length
                        muscleValues.lMtilde(:,1)  = full(lMtilde_ini);  % Normalised muscle length
                        muscleValues.FMvtilde(:,1) = full(FMvtilde_ini); % Normalised force-velocity multiplier
                        muscleValues.vMtilde(:,1)  = full(vMtilde_ini);  % Normalised muscle shortening velocity
                        muscleValues.Fpetilde(:,1) = full(Fpetilde_ini); % Normalised passive force

                        % Hip flexion, left
                        % FTk: non-normalised unscaled tendon force
                        Ft_hip_flex_l = FT_ini(mai(1).mus.l',1);
                        moments.muscles.T_hip_flex_l(1,1) = full(calcMoms.f_T27(MA.hip_flex.l,Ft_hip_flex_l));
                        moments.musc_plus_reserve.T_hip_flex_l(1,1) = full(calcMoms.f_T27(MA.hip_flex.l,Ft_hip_flex_l));
                        
                        % Hip flexion, right
                        Ft_hip_flex_r = FT_ini(mai(1).mus.r',1);
                        moments.muscles.T_hip_flex_r(1,1) = full(calcMoms.f_T27(MA.hip_flex.r,Ft_hip_flex_r));
                        moments.musc_plus_reserve.T_hip_flex_r(1,1) = full(calcMoms.f_T27(MA.hip_flex.r,Ft_hip_flex_r));

                        % Hip adduction, left
                        Ft_hip_add_l = FT_ini(mai(2).mus.l',1);
                        moments.muscles.T_hip_add_l(1,1) = full(calcMoms.f_T27(MA.hip_add.l,Ft_hip_add_l));
                        moments.musc_plus_reserve.T_hip_add_l(1,1) = full(calcMoms.f_T27(MA.hip_add.l,Ft_hip_add_l));

                        % Hip adduction, right
                        Ft_hip_add_r = FT_ini(mai(2).mus.r',1);
                        moments.muscles.T_hip_add_r(1,1) = full(calcMoms.f_T27(MA.hip_add.r,Ft_hip_add_r));          
                        moments.musc_plus_reserve.T_hip_add_r(1,1) = full(calcMoms.f_T27(MA.hip_add.r,Ft_hip_add_r));

                        % Hip rotation, left
                        Ft_hip_rot_l = FT_ini(mai(3).mus.l',1);
                        moments.muscles.T_hip_rot_l(1,1) = full(calcMoms.f_T27(MA.hip_rot.l,Ft_hip_rot_l));
                        moments.musc_plus_reserve.T_hip_rot_l(1,1) = full(calcMoms.f_T27(MA.hip_rot.l,Ft_hip_rot_l));

                        % Hip rotation, right
                        Ft_hip_rot_r = FT_ini(mai(3).mus.r',1);
                        moments.muscles.T_hip_rot_r(1,1) = full(calcMoms.f_T27(MA.hip_rot.r,Ft_hip_rot_r));
                        moments.musc_plus_reserve.T_hip_rot_r(1,1) = full(calcMoms.f_T27(MA.hip_rot.r,Ft_hip_rot_r));

                        % Knee, left
                        Ft_knee_l = FT_ini(mai(4).mus.l',1);
                        moments.muscles.T_knee_l(1,1) = full(calcMoms.f_T13(MA.knee.l,Ft_knee_l));
                        moments.musc_plus_reserve.T_knee_l(1,1) = full(calcMoms.f_T13(MA.knee.l,Ft_knee_l));

                        % Knee, right
                        Ft_knee_r = FT_ini(mai(4).mus.r',1);
                        moments.muscles.T_knee_r(1,1) = full(calcMoms.f_T13(MA.knee.r,Ft_knee_r));
                        moments.musc_plus_reserve.T_knee_r(1,1) = full(calcMoms.f_T13(MA.knee.r,Ft_knee_r));

                        % Ankle, left
                        Ft_ankle_l = FT_ini(mai(5).mus.l',1);
                        moments.muscles.T_ankle_l(1,1) = full(calcMoms.f_T12(MA.ankle.l,Ft_ankle_l));
                        moments.musc_plus_reserve.T_ankle_l(1,1) = full(calcMoms.f_T12(MA.ankle.l,Ft_ankle_l));

                        % Ankle, right
                        Ft_ankle_r = FT_ini(mai(5).mus.r',1);
                        moments.muscles.T_ankle_r(1,1) = full(calcMoms.f_T12(MA.ankle.r,Ft_ankle_r));
                        moments.musc_plus_reserve.T_ankle_r(1,1) = full(calcMoms.f_T12(MA.ankle.r,Ft_ankle_r));

                        % Subtalar, left
                        Ft_subt_l = FT_ini(mai(6).mus.l',1);
                        moments.muscles.T_subt_l(1,1) = full(calcMoms.f_T12(MA.subt.l,Ft_subt_l));
                        moments.musc_plus_reserve.T_subt_l(1,1) = full(calcMoms.f_T12(MA.subt.l,Ft_subt_l));

                        % Subtalar, right
                        Ft_subt_r = FT_ini(mai(6).mus.r',1);
                        moments.muscles.T_subt_r(1,1) = full(calcMoms.f_T12(MA.subt.r,Ft_subt_r));
                        moments.musc_plus_reserve.T_subt_r(1,1) = full(calcMoms.f_T12(MA.subt.r,Ft_subt_r));

                        % MTP, left
                        Ft_mtp_l = FT_ini(mai(7).mus.l',1);
                        moments.muscles.T_mtp_l(1,1) = full(calcMoms.f_T4(MA.mtp.l,Ft_mtp_l));
                        moments.musc_plus_reserve.T_mtp_l(1,1) = full(calcMoms.f_T4(MA.mtp.l,Ft_mtp_l)) + uReserves_nsc_ini(1) - Options.MTP_stiff*Xk_nsc(jointi.mtp.l);
                        moments.passive.T_mtp_l(1,1) =  - Options.MTP_stiff*Xk_nsc(jointi.mtp.l);

                        % MTP, right
                        Ft_mtp_r = FT_ini(mai(7).mus.r',1);
                        moments.muscles.T_mtp_r(1,1) = full(calcMoms.f_T4(MA.mtp.r,Ft_mtp_r));
                        moments.musc_plus_reserve.T_mtp_r(1,1) = full(calcMoms.f_T4(MA.mtp.r,Ft_mtp_r)) + uReserves_nsc_ini(2) - Options.MTP_stiff*Xk_nsc(jointi.mtp.r);
                        moments.passive.T_mtp_r(1,1) =  - Options.MTP_stiff*Xk_nsc(jointi.mtp.r);

                        % Lumbar extension
                        Ft_trunk_ext = FT_ini([mai(8).mus.l,mai(8).mus.r]',1);
                        moments.muscles.T_trunk_ext(1,1) = full(calcMoms.f_T6(MA.trunk_ext,Ft_trunk_ext));
                        moments.musc_plus_reserve.T_trunk_ext(1,1) = full(calcMoms.f_T6(MA.trunk_ext,Ft_trunk_ext));

                        % Lumbar bending 
                        Ft_trunk_ben = FT_ini([mai(9).mus.l,mai(9).mus.r]',1);
                        moments.muscles.T_trunk_ben(1,1) = full(calcMoms.f_T6(MA.trunk_ben,Ft_trunk_ben));
                        moments.musc_plus_reserve.T_trunk_ben(1,1) = full(calcMoms.f_T6(MA.trunk_ben,Ft_trunk_ben));

                        % Lumbar rotating
                        Ft_trunk_rot = FT_ini([mai(10).mus.l,mai(10).mus.r]',1);
                        moments.muscles.T_trunk_rot(1,1) = full(calcMoms.f_T6(MA.trunk_rot,Ft_trunk_rot));
                        moments.musc_plus_reserve.T_trunk_rot(1,1) = full(calcMoms.f_T6(MA.trunk_rot,Ft_trunk_rot));
                        
                    end
                    
            for j=1:d
                
                index = j+(k*d)+1; 
                
                % Reorder the skeleton states as q1 u1 q2 u2...
                % For compatibility with F function
                Xkj_nsc_ORD = zeros(nq.all*2,1);
                for i = 0:nq.all-1
                    Xkj_nsc_ORD(i+1*(i+1),1) = Xkj_nsc{j}(i+1,1);
                    Xkj_nsc_ORD(2*(i+1),1)   = Xkj_nsc{j}(nq.all+i+1,1);
                end
                
                % Evaluate F Function 
                % Evaluation returns residuals, joint torques 
                % and GRFs from contact model
                % Output used later
                outputF = F([Xkj_nsc_ORD;uAcckj_nsc{j};contPrms_nsc ]);
                
                % Contact model GRFs; right then left
                xGRFk_r(index,1) = sum(full(outputF(outInd.r_contGRF(1):3:outInd.r_contGRF(end)-2)));
                yGRFk_r(index,1) = sum(full(outputF(outInd.r_contGRF(1)+1:3:outInd.r_contGRF(end)-1))); 
                zGRFk_r(index,1) = sum(full(outputF(outInd.r_contGRF(1)+2:3:outInd.r_contGRF(end))));            

                xGRFk_l(index,1) = sum(full(outputF(outInd.l_contGRF(1):3:outInd.l_contGRF(end)-2)));
                yGRFk_l(index,1) = sum(full(outputF(outInd.l_contGRF(1)+1:3:outInd.l_contGRF(end)-1)));
                zGRFk_l(index,1) = sum(full(outputF(outInd.l_contGRF(1)+2:3:outInd.l_contGRF(end))));

                r_sph_1_G(:,index) = full(outputF(outInd.r_sph_1_G_XYZ));
                r_sph_2_G(:,index) = full(outputF(outInd.r_sph_2_G_XYZ));
                r_sph_3_G(:,index) = full(outputF(outInd.r_sph_3_G_XYZ));
                r_sph_4_G(:,index) = full(outputF(outInd.r_sph_4_G_XYZ));
                r_sph_5_G(:,index) = full(outputF(outInd.r_sph_5_G_XYZ));
                r_sph_6_G(:,index) = full(outputF(outInd.r_sph_6_G_XYZ));
                r_sph_7_G(:,index) = full(outputF(outInd.r_sph_7_G_XYZ));
                l_sph_1_G(:,index) = full(outputF(outInd.l_sph_1_G_XYZ));
                l_sph_2_G(:,index) = full(outputF(outInd.l_sph_2_G_XYZ));
                l_sph_3_G(:,index) = full(outputF(outInd.l_sph_3_G_XYZ));
                l_sph_4_G(:,index) = full(outputF(outInd.l_sph_4_G_XYZ));
                l_sph_5_G(:,index) = full(outputF(outInd.l_sph_5_G_XYZ));
                l_sph_6_G(:,index) = full(outputF(outInd.l_sph_6_G_XYZ));
                l_sph_7_G(:,index) = full(outputF(outInd.l_sph_7_G_XYZ));

                r_sph_F_1_G(:,index) = full(outputF(outInd.r_contGRF(1):outInd.r_contGRF(3)));
                r_sph_F_2_G(:,index) = full(outputF(outInd.r_contGRF(4):outInd.r_contGRF(6)));
                r_sph_F_3_G(:,index) = full(outputF(outInd.r_contGRF(7):outInd.r_contGRF(9)));
                r_sph_F_4_G(:,index) = full(outputF(outInd.r_contGRF(10):outInd.r_contGRF(12)));
                r_sph_F_5_G(:,index) = full(outputF(outInd.r_contGRF(13):outInd.r_contGRF(15)));
                r_sph_F_6_G(:,index) = full(outputF(outInd.r_contGRF(16):outInd.r_contGRF(18)));
                r_sph_F_7_G(:,index) = full(outputF(outInd.r_contGRF(19):outInd.r_contGRF(21)));
                l_sph_F_1_G(:,index) = full(outputF(outInd.l_contGRF(1):outInd.l_contGRF(3)));
                l_sph_F_2_G(:,index) = full(outputF(outInd.l_contGRF(4):outInd.l_contGRF(6)));
                l_sph_F_3_G(:,index) = full(outputF(outInd.l_contGRF(7):outInd.l_contGRF(9)));
                l_sph_F_4_G(:,index) = full(outputF(outInd.l_contGRF(10):outInd.l_contGRF(12)));
                l_sph_F_5_G(:,index) = full(outputF(outInd.l_contGRF(13):outInd.l_contGRF(15)));
                l_sph_F_6_G(:,index) = full(outputF(outInd.l_contGRF(16):outInd.l_contGRF(18)));
                l_sph_F_7_G(:,index) = full(outputF(outInd.l_contGRF(19):outInd.l_contGRF(21)));

                % Counter function
                termsJ.counter(k*d+j) = full(tanh((yGRFk_r(index,1)) * 0.2));
                
                % Pelvis residuals & joint moments
                moments.joints(:,index) = full(outputF(1:nq.all));

                % Calculate joint power
                moments.power(:,index) = moments.joints(:,index).*Xkj_nsc{j}(nq.all+1:end);
            
                % Model CoM Outputs
                modelCOM.pos(:,index) = full(outputF(outInd.posCOM));
                modelCOM.vel(:,index) = full(outputF(outInd.velCOM));

                % Body Angles
                bodyAngles.femur_r(:,index) = full(outputF(outInd.femur_r_XYZ));
                bodyAngles.femur_l(:,index) = full(outputF(outInd.femur_l_XYZ));
                bodyAngles.tibia_r(:,index) = full(outputF(outInd.tibia_r_XYZ));
                bodyAngles.tibia_l(:,index) = full(outputF(outInd.tibia_l_XYZ));
                bodyAngles.torso(:,index)   = full(outputF(outInd.torso_XYZ));
                
                % Angular Momentum
                angMom.femur_r(:,index)     = full(outputF(outInd.femur_r_H_COM));
                angMom.femur_l(:,index)     = full(outputF(outInd.femur_l_H_COM));
                angMom.tibia_r(:,index)     = full(outputF(outInd.tibia_r_H_COM));
                angMom.tibia_l(:,index)     = full(outputF(outInd.tibia_l_H_COM));
                angMom.talus_r(:,index)     = full(outputF(outInd.talus_r_H_COM));
                angMom.talus_l(:,index)     = full(outputF(outInd.talus_l_H_COM));
                angMom.calcn_r(:,index)     = full(outputF(outInd.calcn_r_H_COM));
                angMom.calcn_l(:,index)     = full(outputF(outInd.calcn_l_H_COM));
                angMom.toes_r(:,index)     = full(outputF(outInd.toes_r_H_COM));
                angMom.toes_l(:,index)     = full(outputF(outInd.toes_l_H_COM));
                angMom.humerus_r(:,index)     = full(outputF(outInd.humerus_r_H_COM));
                angMom.humerus_l(:,index)     = full(outputF(outInd.humerus_l_H_COM));
                angMom.radius_r(:,index)     = full(outputF(outInd.radius_r_H_COM));
                angMom.radius_l(:,index)     = full(outputF(outInd.radius_l_H_COM));
                angMom.ulna_r(:,index)     = full(outputF(outInd.ulna_r_H_COM));
                angMom.ulna_l(:,index)     = full(outputF(outInd.ulna_l_H_COM));
                angMom.hand_r(:,index)     = full(outputF(outInd.hand_r_H_COM));
                angMom.hand_l(:,index)     = full(outputF(outInd.hand_l_H_COM));
                angMom.pelvis(:,index)     = full(outputF(outInd.pelvis_H_COM));
                angMom.torso(:,index)     = full(outputF(outInd.torso_H_COM));
                
                % Get MTU lengths, MTU velocities & moment arms
                % First left leg
                qin_l = [Xkj_nsc{j}(jointi.hip_flex.l),...
                    Xkj_nsc{j}(jointi.hip_add.l),...
                    Xkj_nsc{j}(jointi.hip_rot.l),...
                    Xkj_nsc{j}(jointi.knee.l),...
                    Xkj_nsc{j}(jointi.ankle.l),...
                    Xkj_nsc{j}(jointi.subt.l),...
                    Xkj_nsc{j}(jointi.mtp.l),...
                    Xkj_nsc{j}(jointi.trunk.ext),...
                    Xkj_nsc{j}(jointi.trunk.ben),...
                    Xkj_nsc{j}(jointi.trunk.rot)];
                qdotin_l = [Xkj_nsc{j}(jointi.hip_flex.l+nq.all),...
                    Xkj_nsc{j}(jointi.hip_add.l+nq.all),...
                    Xkj_nsc{j}(jointi.hip_rot.l+nq.all),...
                    Xkj_nsc{j}(jointi.knee.l+nq.all),...
                    Xkj_nsc{j}(jointi.ankle.l+nq.all),...
                    Xkj_nsc{j}(jointi.subt.l+nq.all),...
                    Xkj_nsc{j}(jointi.mtp.l+nq.all),...
                    Xkj_nsc{j}(jointi.trunk.ext+nq.all),...
                    Xkj_nsc{j}(jointi.trunk.ben+nq.all),...
                    Xkj_nsc{j}(jointi.trunk.rot+nq.all)];
                [lMTk_l,vMTk_l,MA_l] = f_lMT_vMT_dM(qin_l,qdotin_l);
                MA.hip_flex.l   = MA_l(mai(1).mus.l',1);
                MA.hip_add.l    = MA_l(mai(2).mus.l',2);
                MA.hip_rot.l    = MA_l(mai(3).mus.l',3);
                MA.knee.l       = MA_l(mai(4).mus.l',4);
                MA.ankle.l      = MA_l(mai(5).mus.l',5);
                MA.subt.l       = MA_l(mai(6).mus.l',6);
                MA.mtp.l        = MA_l(mai(7).mus.l',7);

                % For the back muscles, we want left & right together:
                % left first, right second. In MuscleInfo, we first have right
                % back muscles (44:46) & then left back muscles (47:49). Since 
                % back muscles only depend on back DOFs, we do not care if we 
                % extract from "left leg or right leg"; so here we just pick left
                MA.trunk_ext    = MA_l([47:49,mai(8).mus.l]',8);
                MA.trunk_ben    = MA_l([47:49,mai(9).mus.l]',9);
                MA.trunk_rot    = MA_l([47:49,mai(10).mus.l]',10);

                % Second right leg
                qin_r = [Xkj_nsc{j}(jointi.hip_flex.r),...
                    Xkj_nsc{j}(jointi.hip_add.r),...
                    Xkj_nsc{j}(jointi.hip_rot.r),...
                    Xkj_nsc{j}(jointi.knee.r),...
                    Xkj_nsc{j}(jointi.ankle.r),...
                    Xkj_nsc{j}(jointi.subt.r),...
                    Xkj_nsc{j}(jointi.mtp.r),...
                    Xkj_nsc{j}(jointi.trunk.ext),...
                    Xkj_nsc{j}(jointi.trunk.ben),...
                    Xkj_nsc{j}(jointi.trunk.rot)];
                qdotin_r = [Xkj_nsc{j}(jointi.hip_flex.r+nq.all),...
                    Xkj_nsc{j}(jointi.hip_add.r+nq.all),...
                    Xkj_nsc{j}(jointi.hip_rot.r+nq.all),...
                    Xkj_nsc{j}(jointi.knee.r+nq.all),...
                    Xkj_nsc{j}(jointi.ankle.r+nq.all),...
                    Xkj_nsc{j}(jointi.subt.r+nq.all),...
                    Xkj_nsc{j}(jointi.mtp.r+nq.all),...
                    Xkj_nsc{j}(jointi.trunk.ext+nq.all),...
                    Xkj_nsc{j}(jointi.trunk.ben+nq.all),...
                    Xkj_nsc{j}(jointi.trunk.rot+nq.all)];
                [lMTk_r,vMTk_r,MA_r] = f_lMT_vMT_dM(qin_r,qdotin_r);
                % Here we take the indices from left since the vector is 1:49
                % MA_r: 49 x 10
                % mai: is created starting with left muscles then right muscles
                % Querying MA_r with 'mus.r' would result in an error
                MA.hip_flex.r   = MA_r(mai(1).mus.l',1);
                MA.hip_add.r    = MA_r(mai(2).mus.l',2);
                MA.hip_rot.r    = MA_r(mai(3).mus.l',3);
                MA.knee.r       = MA_r(mai(4).mus.l',4);
                MA.ankle.r      = MA_r(mai(5).mus.l',5);
                MA.subt.r       = MA_r(mai(6).mus.l',6);
                MA.mtp.r        = MA_r(mai(7).mus.l',7);

                % Both legs
                % In MuscleInfo, we first have the right back muscles (44:46)
                % and then the left back muscles (47:49). We reorganise so that
                % we first have the left muscles, then the right muscles
                lMTk_lr = [lMTk_l([1:43,47:49],1);lMTk_r(1:46,1)];
                vMTk_lr = [vMTk_l([1:43,47:49],1);vMTk_r(1:46,1)];
                
                % Moment Arms, MTU-length, MTU-velocity
                muscleValues.MA.hip_flex.l(:,index) = full(MA.hip_flex.l);
                muscleValues.MA.hip_flex.r(:,index) = full(MA.hip_flex.r);

                muscleValues.MA.hip_add.l(:,index) = full(MA.hip_add.l);
                muscleValues.MA.hip_add.r(:,index) = full(MA.hip_add.r);

                muscleValues.MA.hip_rot.l(:,index) = full(MA.hip_rot.l);
                muscleValues.MA.hip_rot.r(:,index) = full(MA.hip_rot.r);

                muscleValues.MA.knee.l(:,index) = full(MA.knee.l);
                muscleValues.MA.knee.r(:,index) = full(MA.knee.r);

                muscleValues.MA.ankle.l(:,index) = full(MA.ankle.l);
                muscleValues.MA.ankle.r(:,index) = full(MA.ankle.r);

                muscleValues.MA.subt.l(:,index) = full(MA.subt.l);
                muscleValues.MA.subt.r(:,index) = full(MA.subt.r);

                muscleValues.MA.mtp.l(:,index) = full(MA.mtp.l);
                muscleValues.MA.mtp.r(:,index) = full(MA.mtp.r);

                muscleValues.MA.trunk_ext(:,index) = full(MA.trunk_ext);
                muscleValues.MA.trunk_ben(:,index) = full(MA.trunk_ben);
                muscleValues.MA.trunk_rot(:,index) = full(MA.trunk_rot);

                muscleValues.lMTk_lr(:,index) = full(lMTk_lr);
                muscleValues.vMTk_lr(:,index) = full(vMTk_lr);
                
                [Hilldiffk,FTk,Fcek,Fpassk,Fisok,vMaxk,lTtildek,lMk,lMtildek,FMvtildek,vMtildek,Fpetildek] = ...
                f_forceEquilibrium_FtildeState_all_tendon_M(actkj_nsc{j},FTtildekj_nsc{j},...
                dFTtildekj_nsc{j},lMTk_lr,vMTk_lr,tensions,aTendon,shift,oMFL_2_nsc,TSL_2_nsc,oMFL_2_nsc.*vMaxMult);
            
                muscleValues.Hilldiff(:,index) = full(Hilldiffk); % (Fcetilde+Fpetilde)*cos(alpha)-FTtilde
                muscleValues.FT(:,index)       = full(FTk);       % Tendon Force (Newtons)
                muscleValues.Fce(:,index)      = full(Fcek);      % act*FMltilde*FMvtilde + d*vMtilde (Newtons)
                muscleValues.Fpass(:,index)    = full(Fpassk);    % Passive Muscle Force (Newtons)
                muscleValues.Fiso(:,index)     = full(Fisok);     % FMltilde - normalised force-length multiplier
                muscleValues.vMax(:,index)     = full(vMaxk);     % Max shortening velocity; i.e. 12 opt musc fibre lengths
                muscleValues.lTtildek(:,index) = full(lTtildek);  % Normalised tendon length
                muscleValues.lM(:,index)       = full(lMk);       % Muscle length
                muscleValues.lMtilde(:,index)  = full(lMtildek);  % Normalised muscle length
                muscleValues.FMvtilde(:,index) = full(FMvtildek); % Normalised force-velocity multiplier
                muscleValues.vMtilde(:,index)  = full(vMtildek);  % Normalised muscle shortening velocity
                muscleValues.Fpetilde(:,index) = full(Fpetildek); % Normalised passive force
                
                % Hip flexion, left
                Ft_hip_flex_l = FTk(mai(1).mus.l',1);
                moments.muscles.T_hip_flex_l(1,index) = full(calcMoms.f_T27(MA.hip_flex.l,Ft_hip_flex_l));
                moments.musc_plus_reserve.T_hip_flex_l(1,index) = full(calcMoms.f_T27(MA.hip_flex.l,Ft_hip_flex_l));

                % Hip flexion, right
                Ft_hip_flex_r = FTk(mai(1).mus.r',1);
                moments.muscles.T_hip_flex_r(1,index) = full(calcMoms.f_T27(MA.hip_flex.r,Ft_hip_flex_r));
                moments.musc_plus_reserve.T_hip_flex_r(1,index) = full(calcMoms.f_T27(MA.hip_flex.r,Ft_hip_flex_r));

                % Hip adduction, left
                Ft_hip_add_l = FTk(mai(2).mus.l',1);
                moments.muscles.T_hip_add_l(1,index) = full(calcMoms.f_T27(MA.hip_add.l,Ft_hip_add_l));
                moments.musc_plus_reserve.T_hip_add_l(1,index) = full(calcMoms.f_T27(MA.hip_add.l,Ft_hip_add_l));

                % Hip adduction, right
                Ft_hip_add_r = FTk(mai(2).mus.r',1);
                moments.muscles.T_hip_add_r(1,index) = full(calcMoms.f_T27(MA.hip_add.r,Ft_hip_add_r));          
                moments.musc_plus_reserve.T_hip_add_r(1,index) = full(calcMoms.f_T27(MA.hip_add.r,Ft_hip_add_r));

                % Hip rotation, left
                Ft_hip_rot_l = FTk(mai(3).mus.l',1);
                moments.muscles.T_hip_rot_l(1,index) = full(calcMoms.f_T27(MA.hip_rot.l,Ft_hip_rot_l));
                moments.musc_plus_reserve.T_hip_rot_l(1,index) = full(calcMoms.f_T27(MA.hip_rot.l,Ft_hip_rot_l));

                % Hip rotation, right
                Ft_hip_rot_r = FTk(mai(3).mus.r',1);
                moments.muscles.T_hip_rot_r(1,index) = full(calcMoms.f_T27(MA.hip_rot.r,Ft_hip_rot_r));
                moments.musc_plus_reserve.T_hip_rot_r(1,index) = full(calcMoms.f_T27(MA.hip_rot.r,Ft_hip_rot_r));

                % Knee, left
                Ft_knee_l = FTk(mai(4).mus.l',1);
                moments.muscles.T_knee_l(1,index) = full(calcMoms.f_T13(MA.knee.l,Ft_knee_l));
                moments.musc_plus_reserve.T_knee_l(1,index) = full(calcMoms.f_T13(MA.knee.l,Ft_knee_l));

                % Knee, right
                Ft_knee_r = FTk(mai(4).mus.r',1);
                moments.muscles.T_knee_r(1,index) = full(calcMoms.f_T13(MA.knee.r,Ft_knee_r));
                moments.musc_plus_reserve.T_knee_r(1,index) = full(calcMoms.f_T13(MA.knee.r,Ft_knee_r));

                % Ankle, left
                Ft_ankle_l = FTk(mai(5).mus.l',1);
                moments.muscles.T_ankle_l(1,index) = full(calcMoms.f_T12(MA.ankle.l,Ft_ankle_l));
                moments.musc_plus_reserve.T_ankle_l(1,index) = full(calcMoms.f_T12(MA.ankle.l,Ft_ankle_l));

                % Ankle, right
                Ft_ankle_r = FTk(mai(5).mus.r',1);
                moments.muscles.T_ankle_r(1,index) = full(calcMoms.f_T12(MA.ankle.r,Ft_ankle_r));
                moments.musc_plus_reserve.T_ankle_r(1,index) = full(calcMoms.f_T12(MA.ankle.r,Ft_ankle_r));

                % Subtalar, left
                Ft_subt_l = FTk(mai(6).mus.l',1);
                moments.muscles.T_subt_l(1,index) = full(calcMoms.f_T12(MA.subt.l,Ft_subt_l));
                moments.musc_plus_reserve.T_subt_l(1,index) = full(calcMoms.f_T12(MA.subt.l,Ft_subt_l));

                % Subtalar, right
                Ft_subt_r = FTk(mai(6).mus.r',1);
                moments.muscles.T_subt_r(1,index) = full(calcMoms.f_T12(MA.subt.r,Ft_subt_r));
                moments.musc_plus_reserve.T_subt_r(1,index) = full(calcMoms.f_T12(MA.subt.r,Ft_subt_r));

                % MTP, left
                Ft_mtp_l = FTk(mai(7).mus.l',1);
                moments.muscles.T_mtp_l(1,index) = full(calcMoms.f_T4(MA.mtp.l,Ft_mtp_l));
                moments.musc_plus_reserve.T_mtp_l(1,index) = full(calcMoms.f_T4(MA.mtp.l,Ft_mtp_l)) + uReserveskj_nsc{j}(1) - Options.MTP_stiff*Xkj_nsc{j}(jointi.mtp.l);
                moments.passive.T_mtp_l(1,index) =  - Options.MTP_stiff*Xkj_nsc{j}(jointi.mtp.l);

                % MTP, right
                Ft_mtp_r = FTk(mai(7).mus.r',1);
                moments.muscles.T_mtp_r(1,index) = full(calcMoms.f_T4(MA.mtp.r,Ft_mtp_r));
                moments.musc_plus_reserve.T_mtp_r(1,index) = full(calcMoms.f_T4(MA.mtp.r,Ft_mtp_r)) + uReserveskj_nsc{j}(2) - Options.MTP_stiff*Xkj_nsc{j}(jointi.mtp.r);
                moments.passive.T_mtp_r(1,index) =  - Options.MTP_stiff*Xkj_nsc{j}(jointi.mtp.r);

                % Lumbar extension
                Ft_trunk_ext = FTk([mai(8).mus.l,mai(8).mus.r]',1);
                moments.muscles.T_trunk_ext(1,index) = full(calcMoms.f_T6(MA.trunk_ext,Ft_trunk_ext));
                moments.musc_plus_reserve.T_trunk_ext(1,index) = full(calcMoms.f_T6(MA.trunk_ext,Ft_trunk_ext));

                % Lumbar bending 
                Ft_trunk_ben = FTk([mai(9).mus.l,mai(9).mus.r]',1);
                moments.muscles.T_trunk_ben(1,index) = full(calcMoms.f_T6(MA.trunk_ben,Ft_trunk_ben));
                moments.musc_plus_reserve.T_trunk_ben(1,index) = full(calcMoms.f_T6(MA.trunk_ben,Ft_trunk_ben));

                % Lumbar rotating
                Ft_trunk_rot = FTk([mai(10).mus.l,mai(10).mus.r]',1);
                moments.muscles.T_trunk_rot(1,index) = full(calcMoms.f_T6(MA.trunk_rot,Ft_trunk_rot));
                moments.musc_plus_reserve.T_trunk_rot(1,index) = full(calcMoms.f_T6(MA.trunk_rot,Ft_trunk_rot));
                
                J_pel_q  = J_pel_q  + wJ(1).*B(j+1)*(costFunctions.f_J_gPelOri(statesF.q_aux(k*(d+1)+1+j,1:3),Xkj_nsc{j}(1:3)))*h;
                J_pel_t  = J_pel_t  + wJ(2).*B(j+1)*(costFunctions.f_J_gPelTra(statesF.q_aux(k*(d+1)+1+j,5:6),Xkj_nsc{j}(5:6)))*h;
                J_kin_ll = J_kin_ll + wJ(3).*B(j+1)*(costFunctions.f_J_lljAngs(statesF.q_aux(k*(d+1)+1+j,[7:13,14:20]),Xkj_nsc{j}([7:13,14:20])))*h;
                J_kin_ul = J_kin_ul + wJ(4).*B(j+1)*(costFunctions.f_J_uljAngs(statesF.q_aux(k*(d+1)+1+j,21:end),Xkj_nsc{j}(21:37)))*h;
                J_accs   = J_accs   + wJ(5).*B(j+1)*(costFunctions.f_J_accs(uAcckj_nsc{j}))*h;

                J_act    = J_act    + wJ(6).*B(j+1)*(costFunctions.f_J_muscle_act(actkj_nsc{j}))*h;
                J_dAct   = J_dAct   + wJ(7).*B(j+1)*(costFunctions.f_J_dmuscle_act(uActdotkj_nsc{j}))*h;
                J_ftTil  = J_ftTil  + wJ(8).*B(j+1)*(costFunctions.f_J_FT(FTtildekj_nsc{j}))*h;
                J_dftTil = J_dftTil + wJ(9).*B(j+1)*(costFunctions.f_J_dFT(dFTtildekj_nsc{j}))*h;

                J_reserv = J_reserv + wJ(10).*B(j+1)*(costFunctions.f_J_reserves(uReserveskj_nsc{j}))*h;
                J_arms   = J_arms   + wJ(11).*B(j+1)*(costFunctions.f_J_arms(armActskj_nsc{j}))*h;

                J_work_net_hip_l = J_work_net_hip_l + B(j+1)*moments.joints(14,index).*Xkj_nsc{j}(nq.all+14)*h;
                J_work_net_hip_r = J_work_net_hip_r + B(j+1)*moments.joints(7,index).*Xkj_nsc{j}(nq.all+7)*h;
                J_work_net_knee_l = J_work_net_knee_l + B(j+1)*moments.joints(17,index).*Xkj_nsc{j}(nq.all+17)*h;
                J_work_net_knee_r = J_work_net_knee_r + B(j+1)*moments.joints(10,index).*Xkj_nsc{j}(nq.all+10)*h;
                J_work_net_ankle_l = J_work_net_ankle_l + B(j+1)*moments.joints(18,index).*Xkj_nsc{j}(nq.all+18)*h;
                J_work_net_ankle_r = J_work_net_ankle_r + B(j+1)*moments.joints(11,index).*Xkj_nsc{j}(nq.all+11)*h;
               
                % Smooth function to return ideally 0 or 1; 
                % If the angle is (+) -> 0.5+0.5*tanh(q/0.005)
                % If the angle is (-) -> 0.5+0.5*-tanh(q/0.005)
                % The 0.5's shift the range of the function -> [0,1]
                smooth_output_hip_ext_l = 0.5 + 0.5 * -tanh(Xkj_nsc{j}(14) / 0.005);
                smooth_output_hip_fle_l = 0.5 + 0.5 * tanh(Xkj_nsc{j}(14) / 0.005);
                smooth_output_ankle_ext_l = 0.5 + 0.5 * -tanh(Xkj_nsc{j}(18) / 0.005);
                smooth_output_ankle_fle_l = 0.5 + 0.5 * tanh(Xkj_nsc{j}(18) / 0.005);
                smooth_output_knee_ext_l = 0.5 + 0.5 * tanh(Xkj_nsc{j}(17) / 0.005);
                smooth_output_knee_fle_l = 0.5 + 0.5 * -tanh(Xkj_nsc{j}(17) / 0.005);

                smooth_output_hip_ext_r = 0.5 + 0.5 * -tanh(Xkj_nsc{j}(7) / 0.005);
                smooth_output_hip_fle_r = 0.5 + 0.5 * tanh(Xkj_nsc{j}(7) / 0.005);
                smooth_output_ankle_ext_r = 0.5 + 0.5 * -tanh(Xkj_nsc{j}(11) / 0.005);
                smooth_output_ankle_fle_r = 0.5 + 0.5 * tanh(Xkj_nsc{j}(11) / 0.005);
                smooth_output_knee_ext_r = 0.5 + 0.5 * tanh(Xkj_nsc{j}(10) / 0.005);
                smooth_output_knee_fle_r = 0.5 + 0.5 * -tanh(Xkj_nsc{j}(10) / 0.005);

                % Only positive extensor work
                J_work_net_pos_ext_hip_l = J_work_net_pos_ext_hip_l + B(j+1)*(smooth_output_hip_ext_l*((1/1).*log(1+exp(1*1*((moments.joints(14,index).*Xkj_nsc{j}(nq.all+14))./72.2))))*1)*h;
                J_work_net_neg_ext_hip_l = J_work_net_neg_ext_hip_l + B(j+1)*(smooth_output_hip_ext_l*((1/1).*log(1+exp(1*-1*((moments.joints(14,index).*Xkj_nsc{j}(nq.all+14))./72.2))))*-1)*h;
                J_work_net_pos_fle_hip_l = J_work_net_pos_fle_hip_l + B(j+1)*(smooth_output_hip_fle_l*((1/1).*log(1+exp(1*1*((moments.joints(14,index).*Xkj_nsc{j}(nq.all+14))./72.2))))*1)*h;
                J_work_net_neg_fle_hip_l = J_work_net_neg_fle_hip_l + B(j+1)*(smooth_output_hip_fle_l*((1/1).*log(1+exp(1*-1*((moments.joints(14,index).*Xkj_nsc{j}(nq.all+14))./72.2))))*-1)*h;

                moments.powerJoints.pos_ext_hip_l(index-1) = full(smooth_output_hip_ext_l*((1/1).*log(1+exp(1*1*((moments.joints(14,index).*Xkj_nsc{j}(nq.all+14))./72.2))))*1);
                moments.powerJoints.neg_ext_hip_l(index-1) = full(smooth_output_hip_ext_l*((1/1).*log(1+exp(1*-1*((moments.joints(14,index).*Xkj_nsc{j}(nq.all+14))./72.2))))*-1);
                moments.powerJoints.pos_fle_hip_l(index-1) = full(smooth_output_hip_fle_l*((1/1).*log(1+exp(1*1*((moments.joints(14,index).*Xkj_nsc{j}(nq.all+14))./72.2))))*1);
                moments.powerJoints.neg_fle_hip_l(index-1) = full(smooth_output_hip_fle_l*((1/1).*log(1+exp(1*-1*((moments.joints(14,index).*Xkj_nsc{j}(nq.all+14))./72.2))))*-1);

                J_work_net_pos_ext_hip_r = J_work_net_pos_ext_hip_r + B(j+1)*(smooth_output_hip_ext_r*((1/1).*log(1+exp(1*1*((moments.joints(7,index).*Xkj_nsc{j}(nq.all+7))./72.2))))*1)*h;
                J_work_net_neg_ext_hip_r = J_work_net_neg_ext_hip_r + B(j+1)*(smooth_output_hip_ext_r*((1/1).*log(1+exp(1*-1*((moments.joints(7,index).*Xkj_nsc{j}(nq.all+7))./72.2))))*-1)*h;
                J_work_net_pos_fle_hip_r = J_work_net_pos_fle_hip_r + B(j+1)*(smooth_output_hip_fle_r*((1/1).*log(1+exp(1*1*((moments.joints(7,index).*Xkj_nsc{j}(nq.all+7))./72.2))))*1)*h;
                J_work_net_neg_fle_hip_r = J_work_net_neg_fle_hip_r + B(j+1)*(smooth_output_hip_fle_r*((1/1).*log(1+exp(1*-1*((moments.joints(7,index).*Xkj_nsc{j}(nq.all+7))./72.2))))*-1)*h;

                moments.powerJoints.pos_ext_hip_r(index-1) = full(smooth_output_hip_ext_r*((1/1).*log(1+exp(1*1*((moments.joints(7,index).*Xkj_nsc{j}(nq.all+7))./72.2))))*1);
                moments.powerJoints.neg_ext_hip_r(index-1) = full(smooth_output_hip_ext_r*((1/1).*log(1+exp(1*-1*((moments.joints(7,index).*Xkj_nsc{j}(nq.all+7))./72.2))))*-1);
                moments.powerJoints.pos_fle_hip_r(index-1) = full(smooth_output_hip_fle_r*((1/1).*log(1+exp(1*1*((moments.joints(7,index).*Xkj_nsc{j}(nq.all+7))./72.2))))*1);
                moments.powerJoints.neg_fle_hip_r(index-1) = full(smooth_output_hip_fle_r*((1/1).*log(1+exp(1*-1*((moments.joints(7,index).*Xkj_nsc{j}(nq.all+7))./72.2))))*-1);

                J_work_net_pos_ext_ankle_l = J_work_net_pos_ext_ankle_l + B(j+1)*(smooth_output_ankle_ext_l*((1/1).*log(1+exp(1*1*((moments.joints(18,index).*Xkj_nsc{j}(nq.all+18))./72.2))))*1)*h;
                J_work_net_neg_ext_ankle_l = J_work_net_neg_ext_ankle_l + B(j+1)*(smooth_output_ankle_ext_l*((1/1).*log(1+exp(1*-1*((moments.joints(18,index).*Xkj_nsc{j}(nq.all+18))./72.2))))*-1)*h;
                J_work_net_pos_fle_ankle_l = J_work_net_pos_fle_ankle_l + B(j+1)*(smooth_output_ankle_fle_l*((1/1).*log(1+exp(1*1*((moments.joints(18,index).*Xkj_nsc{j}(nq.all+18))./72.2))))*1)*h;
                J_work_net_neg_fle_ankle_l = J_work_net_neg_fle_ankle_l + B(j+1)*(smooth_output_ankle_fle_l*((1/1).*log(1+exp(1*-1*((moments.joints(18,index).*Xkj_nsc{j}(nq.all+18))./72.2))))*-1)*h;

                moments.powerJoints.pos_ext_ankle_l(index-1) = full(smooth_output_ankle_ext_l*((1/1).*log(1+exp(1*1*((moments.joints(18,index).*Xkj_nsc{j}(nq.all+18))./72.2))))*1);
                moments.powerJoints.neg_ext_ankle_l(index-1) = full(smooth_output_ankle_ext_l*((1/1).*log(1+exp(1*-1*((moments.joints(18,index).*Xkj_nsc{j}(nq.all+18))./72.2))))*-1);
                moments.powerJoints.pos_fle_ankle_l(index-1) = full(smooth_output_ankle_fle_l*((1/1).*log(1+exp(1*1*((moments.joints(18,index).*Xkj_nsc{j}(nq.all+18))./72.2))))*1);
                moments.powerJoints.neg_fle_ankle_l(index-1) = full(smooth_output_ankle_fle_l*((1/1).*log(1+exp(1*-1*((moments.joints(18,index).*Xkj_nsc{j}(nq.all+18))./72.2))))*-1);

                J_work_net_pos_ext_ankle_r = J_work_net_pos_ext_ankle_r + B(j+1)*(smooth_output_ankle_ext_r*((1/1).*log(1+exp(1*1*((moments.joints(11,index).*Xkj_nsc{j}(nq.all+11))./72.2))))*1)*h;
                J_work_net_neg_ext_ankle_r = J_work_net_neg_ext_ankle_r + B(j+1)*(smooth_output_ankle_ext_r*((1/1).*log(1+exp(1*-1*((moments.joints(11,index).*Xkj_nsc{j}(nq.all+11))./72.2))))*-1)*h;
                J_work_net_pos_fle_ankle_r = J_work_net_pos_fle_ankle_r + B(j+1)*(smooth_output_ankle_fle_r*((1/1).*log(1+exp(1*1*((moments.joints(11,index).*Xkj_nsc{j}(nq.all+11))./72.2))))*1)*h;
                J_work_net_neg_fle_ankle_r = J_work_net_neg_fle_ankle_r + B(j+1)*(smooth_output_ankle_fle_r*((1/1).*log(1+exp(1*-1*((moments.joints(11,index).*Xkj_nsc{j}(nq.all+11))./72.2))))*-1)*h;

                moments.powerJoints.pos_ext_ankle_r(index-1) = full(smooth_output_ankle_ext_r*((1/1).*log(1+exp(1*1*((moments.joints(11,index).*Xkj_nsc{j}(nq.all+11))./72.2))))*1);
                moments.powerJoints.neg_ext_ankle_r(index-1) = full(smooth_output_ankle_ext_r*((1/1).*log(1+exp(1*-1*((moments.joints(11,index).*Xkj_nsc{j}(nq.all+11))./72.2))))*-1);
                moments.powerJoints.pos_fle_ankle_r(index-1) = full(smooth_output_ankle_fle_r*((1/1).*log(1+exp(1*1*((moments.joints(11,index).*Xkj_nsc{j}(nq.all+11))./72.2))))*1);
                moments.powerJoints.neg_fle_ankle_r(index-1) = full(smooth_output_ankle_fle_r*((1/1).*log(1+exp(1*-1*((moments.joints(11,index).*Xkj_nsc{j}(nq.all+11))./72.2))))*-1);

                J_work_net_pos_ext_knee_l = J_work_net_pos_ext_knee_l + B(j+1)*(smooth_output_knee_ext_l*((1/1).*log(1+exp(1*1*((moments.joints(17,index).*Xkj_nsc{j}(nq.all+17))./72.2))))*1)*h;
                J_work_net_neg_ext_knee_l = J_work_net_neg_ext_knee_l + B(j+1)*(smooth_output_knee_ext_l*((1/1).*log(1+exp(1*-1*((moments.joints(17,index).*Xkj_nsc{j}(nq.all+17))./72.2))))*-1)*h;
                J_work_net_pos_fle_knee_l = J_work_net_pos_fle_knee_l + B(j+1)*(smooth_output_knee_fle_l*((1/1).*log(1+exp(1*1*((moments.joints(17,index).*Xkj_nsc{j}(nq.all+17))./72.2))))*1)*h;
                J_work_net_neg_fle_knee_l = J_work_net_neg_fle_knee_l + B(j+1)*(smooth_output_knee_fle_l*((1/1).*log(1+exp(1*-1*((moments.joints(17,index).*Xkj_nsc{j}(nq.all+17))./72.2))))*-1)*h;

                moments.powerJoints.pos_ext_knee_l(index-1) = full(smooth_output_knee_ext_l*((1/1).*log(1+exp(1*1*((moments.joints(17,index).*Xkj_nsc{j}(nq.all+17))./72.2))))*1);
                moments.powerJoints.neg_ext_knee_l(index-1) = full(smooth_output_knee_ext_l*((1/1).*log(1+exp(1*-1*((moments.joints(17,index).*Xkj_nsc{j}(nq.all+17))./72.2))))*-1);
                moments.powerJoints.pos_fle_knee_l(index-1) = full(smooth_output_knee_fle_l*((1/1).*log(1+exp(1*1*((moments.joints(17,index).*Xkj_nsc{j}(nq.all+17))./72.2))))*1);
                moments.powerJoints.neg_fle_knee_l(index-1) = full(smooth_output_knee_fle_l*((1/1).*log(1+exp(1*-1*((moments.joints(17,index).*Xkj_nsc{j}(nq.all+17))./72.2))))*-1);

                J_work_net_pos_ext_knee_r = J_work_net_pos_ext_knee_r + B(j+1)*(smooth_output_knee_ext_r*((1/1).*log(1+exp(1*1*((moments.joints(10,index).*Xkj_nsc{j}(nq.all+10))./72.2))))*1)*h;
                J_work_net_neg_ext_knee_r = J_work_net_neg_ext_knee_r + B(j+1)*(smooth_output_knee_ext_r*((1/1).*log(1+exp(1*-1*((moments.joints(10,index).*Xkj_nsc{j}(nq.all+10))./72.2))))*-1)*h;
                J_work_net_pos_fle_knee_r = J_work_net_pos_fle_knee_r + B(j+1)*(smooth_output_knee_fle_r*((1/1).*log(1+exp(1*1*((moments.joints(10,index).*Xkj_nsc{j}(nq.all+10))./72.2))))*1)*h;
                J_work_net_neg_fle_knee_r = J_work_net_neg_fle_knee_r + B(j+1)*(smooth_output_knee_fle_r*((1/1).*log(1+exp(1*-1*((moments.joints(10,index).*Xkj_nsc{j}(nq.all+10))./72.2))))*-1)*h;

                moments.powerJoints.pos_ext_knee_r(index-1) = full(smooth_output_knee_ext_r*((1/1).*log(1+exp(1*1*((moments.joints(10,index).*Xkj_nsc{j}(nq.all+10))./72.2))))*1);
                moments.powerJoints.neg_ext_knee_r(index-1) = full(smooth_output_knee_ext_r*((1/1).*log(1+exp(1*-1*((moments.joints(10,index).*Xkj_nsc{j}(nq.all+10))./72.2))))*-1);
                moments.powerJoints.pos_fle_knee_r(index-1) = full(smooth_output_knee_fle_r*((1/1).*log(1+exp(1*1*((moments.joints(10,index).*Xkj_nsc{j}(nq.all+10))./72.2))))*1);
                moments.powerJoints.neg_fle_knee_r(index-1) = full(smooth_output_knee_fle_r*((1/1).*log(1+exp(1*-1*((moments.joints(10,index).*Xkj_nsc{j}(nq.all+10))./72.2))))*-1);
                
            end

            if k == Options.N - 1
            
                for z = 1:length(termsJ.counter)/3
                    for zz = 1:3
                        time_cont_vec(zz+(z-1)*3) = h * tau_root(zz+1)*termsJ.counter(zz+(z-1)*3);
                    end
                end

                for z = 1:length(termsJ.counter)/3
                    diff_cont(z) = time_cont_vec(z+(z-1)*2) + time_cont_vec(z+(z*2)-1) - time_cont_vec(z+(z-1)*2) + ...
                        time_cont_vec(z*3) - time_cont_vec(z+(z*2)-1);
                end

                termsJ.contact_time = full(sum(diff_cont));

                termsJ.F_ave = full((72.2 * 9.81 * optVars_nsc.totalTime) / termsJ.contact_time);

            end
  
        end
        
        termsJ.total = full(J_pel_q+J_pel_t+J_kin_ll+J_kin_ul+J_accs+J_act+J_dAct+J_ftTil+J_dftTil+J_reserv+J_arms+J_time);
        termsJ.J_pel_q   = full(J_pel_q);
        termsJ.J_pel_t   = full(J_pel_t);
        termsJ.J_kin_ll  = full(J_kin_ll);
        termsJ.J_kin_ul  = full(J_kin_ul);
        termsJ.J_accs    = full(J_accs);
        termsJ.J_act     = full(J_act);
        termsJ.J_dAct    = full(J_dAct);
        termsJ.J_ftTil   = full(J_ftTil);
        termsJ.J_dftTil  = full(J_dftTil);
        termsJ.J_reserv  = full(J_reserv);
        termsJ.J_arms    = full(J_arms);
        termsJ.J_time    = full(J_time);

        termsJ.J_work_net_hip_l = full(J_work_net_hip_l);
        termsJ.J_work_net_pos_ext_hip_l = full(J_work_net_pos_ext_hip_l);
        termsJ.J_work_net_neg_ext_hip_l = full(J_work_net_neg_ext_hip_l);
        termsJ.J_work_net_pos_fle_hip_l = full(J_work_net_pos_fle_hip_l);
        termsJ.J_work_net_neg_fle_hip_l = full(J_work_net_neg_fle_hip_l);
        
        termsJ.J_work_net_hip_r = full(J_work_net_hip_r);
        termsJ.J_work_net_pos_ext_hip_r = full(J_work_net_pos_ext_hip_r);
        termsJ.J_work_net_neg_ext_hip_r = full(J_work_net_neg_ext_hip_r);
        termsJ.J_work_net_pos_fle_hip_r = full(J_work_net_pos_fle_hip_r);
        termsJ.J_work_net_neg_fle_hip_r = full(J_work_net_neg_fle_hip_r);
        
        termsJ.J_work_net_knee_l = full(J_work_net_knee_l);
        termsJ.J_work_net_pos_ext_knee_l = full(J_work_net_pos_ext_knee_l);
        termsJ.J_work_net_neg_ext_knee_l = full(J_work_net_neg_ext_knee_l);
        termsJ.J_work_net_pos_fle_knee_l = full(J_work_net_pos_fle_knee_l);
        termsJ.J_work_net_neg_fle_knee_l = full(J_work_net_neg_fle_knee_l);
        
        termsJ.J_work_net_knee_r = full(J_work_net_knee_r);
        termsJ.J_work_net_pos_ext_knee_r = full(J_work_net_pos_ext_knee_r);
        termsJ.J_work_net_neg_ext_knee_r = full(J_work_net_neg_ext_knee_r);
        termsJ.J_work_net_pos_fle_knee_r = full(J_work_net_pos_fle_knee_r);
        termsJ.J_work_net_neg_fle_knee_r = full(J_work_net_neg_fle_knee_r);

        termsJ.J_work_net_ankle_l = full(J_work_net_ankle_l);
        termsJ.J_work_net_pos_ext_ankle_l = full(J_work_net_pos_ext_ankle_l);
        termsJ.J_work_net_neg_ext_ankle_l = full(J_work_net_neg_ext_ankle_l);
        termsJ.J_work_net_pos_fle_ankle_l = full(J_work_net_pos_fle_ankle_l);
        termsJ.J_work_net_neg_fle_ankle_l = full(J_work_net_neg_fle_ankle_l);
        
        termsJ.J_work_net_ankle_r = full(J_work_net_ankle_r);
        termsJ.J_work_net_pos_ext_ankle_r = full(J_work_net_pos_ext_ankle_r);
        termsJ.J_work_net_neg_ext_ankle_r = full(J_work_net_neg_ext_ankle_r);
        termsJ.J_work_net_pos_fle_ankle_r = full(J_work_net_pos_fle_ankle_r);
        termsJ.J_work_net_neg_fle_ankle_r = full(J_work_net_neg_fle_ankle_r);
        
        GRFs.R = [xGRFk_r yGRFk_r zGRFk_r];
        GRFs.L = [xGRFk_l yGRFk_l zGRFk_l];

        GRF_individual.r_sph_1_G = r_sph_1_G;
        GRF_individual.r_sph_2_G = r_sph_2_G;
        GRF_individual.r_sph_3_G = r_sph_3_G;
        GRF_individual.r_sph_4_G = r_sph_4_G;
        GRF_individual.r_sph_5_G = r_sph_5_G;
        GRF_individual.r_sph_6_G = r_sph_6_G;
        GRF_individual.r_sph_7_G = r_sph_7_G;
        GRF_individual.l_sph_1_G = l_sph_1_G;
        GRF_individual.l_sph_2_G = l_sph_2_G;
        GRF_individual.l_sph_3_G = l_sph_3_G;
        GRF_individual.l_sph_4_G = l_sph_4_G;
        GRF_individual.l_sph_5_G = l_sph_5_G;
        GRF_individual.l_sph_6_G = l_sph_6_G;
        GRF_individual.l_sph_7_G = l_sph_7_G;

        GRF_individual.r_sph_F_1_G = r_sph_F_1_G;
        GRF_individual.r_sph_F_2_G = r_sph_F_2_G;
        GRF_individual.r_sph_F_3_G = r_sph_F_3_G;
        GRF_individual.r_sph_F_4_G = r_sph_F_4_G;
        GRF_individual.r_sph_F_5_G = r_sph_F_5_G;
        GRF_individual.r_sph_F_6_G = r_sph_F_6_G;
        GRF_individual.r_sph_F_7_G = r_sph_F_7_G;
        GRF_individual.l_sph_F_1_G = l_sph_F_1_G;
        GRF_individual.l_sph_F_2_G = l_sph_F_2_G;
        GRF_individual.l_sph_F_3_G = l_sph_F_3_G;
        GRF_individual.l_sph_F_4_G = l_sph_F_4_G;
        GRF_individual.l_sph_F_5_G = l_sph_F_5_G;
        GRF_individual.l_sph_F_6_G = l_sph_F_6_G;
        GRF_individual.l_sph_F_7_G = l_sph_F_7_G;
 
    end

    
    
end