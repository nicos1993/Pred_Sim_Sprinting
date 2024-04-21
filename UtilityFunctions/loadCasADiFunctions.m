function [f_lMT_vMT_dM,f_forceEquilibrium_FtildeState_all_tendon_M,...
    f_J_gPelOri,f_J_gPelTra,f_J_lljAngs,f_J_uljAngs,...
    f_J_gPelOriv,f_J_gPelTrav,f_J_lljAngsv,f_J_uljAngsv,...
    f_J_accs,f_J_resids,f_J_tau,f_J_grf,...
    f_J_muscle_act,f_J_dFT,f_J_dMuscle_act,f_J_FT,...
    f_T27,f_T13,f_T12,f_T4,f_T6] = loadCasADiFunctions(musInd,NMuscle,m_oMFL,m_TSL,m_vmax,pathpolynomial,nq,MTparameters_m)

import casadi.*

%% 1. Muscle model functions

% We load some variables for the polynomial approximations
load([pathpolynomial,'\muscle_spanning_joint_INFO_subject1.mat']);
load([pathpolynomial,'\MuscleINFO_subject1.mat'],'MuscleInfo');
% For the polynomials, we want all independent muscles. So we do not need
% the muscles from both legs, since we assume bilateral symmetry, but want
% all muscles from the back (indices 47:49)
musi_pol = [musInd,47,48,49];
NMuscle_pol = NMuscle/2+3;

% Polynomial Approximation
muscle_spanning_info_m = muscle_spanning_joint_INFO(musi_pol,:);
MuscleInfo_m.muscle    = MuscleInfo.muscle(musi_pol);
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


%% 2. Cost functions

% State scale factors
stateSF = [deg2rad(5) 0.01 deg2rad(5) deg2rad(5) 10.0 0.1 10.0 10.0];

% Global Pelvis Orientations
gpo_e = MX.sym('gpo_e',nq.abs/2); 
gpo_s = MX.sym('gpo_s',nq.abs/2);
gpo_sf = MX.sym('gpo_sf',3);
gPelOri_temp = 0;
for i = 1:length(gpo_e)
    gPelOri_temp = gPelOri_temp + (((gpo_e(i)-gpo_s(i))./stateSF(1,1)).^2)./gpo_sf(i);
end
f_J_gPelOri = Function('f_J_gPelOri',{gpo_e,gpo_s,gpo_sf},{gPelOri_temp});

% Global Pelvis Translations
gpt_e = MX.sym('gpt_e',nq.abs/2);
gpt_s = MX.sym('gpt_s',nq.abs/2);
gpt_sf = MX.sym('gpt_sf',3);
gPelTra_temp = 0;
for i = 1:length(gpt_e)
    gPelTra_temp = gPelTra_temp + (((gpt_e(i)-gpt_s(i))./stateSF(1,2)).^2)./gpt_sf(i);
end 
f_J_gPelTra = Function('f_J_gPelTra',{gpt_e,gpt_s,gpt_sf},{gPelTra_temp});

% Lower-limb joint angles
lljangs_e = MX.sym('lljangs_e',nq.all-nq.abs-nq.trunk-nq.arms);
lljangs_s = MX.sym('lljangs_s',nq.all-nq.abs-nq.trunk-nq.arms);
lljangs_sf = MX.sym('lljangs_sf',nq.all-nq.abs-nq.trunk-nq.arms);
lljAngs_temp = 0;
for i = 1:length(lljangs_e)
    lljAngs_temp = lljAngs_temp + (((lljangs_e(i)-lljangs_s(i))./stateSF(1,3)).^2)./lljangs_sf(i);
end
f_J_lljAngs = Function('f_J_lljAngs',{lljangs_e,lljangs_s,lljangs_sf},{lljAngs_temp});

% Upper-limb & trunk joint angles
uljangs_e = MX.sym('uljangs_e',nq.trunk+nq.arms);
uljangs_s = MX.sym('uljangs_s',nq.trunk+nq.arms);
uljangs_sf = MX.sym('uljangs_sf',nq.trunk+nq.arms);
uljAngs_temp = 0;
for i = 1:length(uljangs_e)
    uljAngs_temp = uljAngs_temp + (((uljangs_e(i)-uljangs_s(i))./stateSF(1,4)).^2)./uljangs_sf(i);
end
f_J_uljAngs = Function('f_J_uljAngs',{uljangs_e,uljangs_s,uljangs_sf},{uljAngs_temp});

% Global Pelvis Orientations Velocity
gpov_e = MX.sym('gpov_e',nq.abs/2); 
gpov_s = MX.sym('gpov_s',nq.abs/2);
gpov_sf = MX.sym('gpov_sf',3);
gPelOriv_temp = 0;
for i = 1:length(gpov_e)
    gPelOriv_temp = gPelOriv_temp + (((gpov_e(i)-gpov_s(i))./stateSF(1,5)).^2)./gpov_sf(i);
end
f_J_gPelOriv = Function('f_J_gPelOriv',{gpov_e,gpov_s,gpov_sf},{gPelOriv_temp});

% Global Pelvis Translations Velocity
gptv_e = MX.sym('gptv_e',nq.abs/2);
gptv_s = MX.sym('gptv_s',nq.abs/2);
gptv_sf = MX.sym('gptv_sf',3);
gPelTrav_temp = 0;
for i = 1:length(gptv_e)
    gPelTrav_temp = gPelTrav_temp + (((gptv_e(i)-gptv_s(i))./stateSF(1,6)).^2)./gptv_sf(i);
end 
f_J_gPelTrav = Function('f_J_gPelTrav',{gptv_e,gptv_s,gptv_sf},{gPelTrav_temp});

% Lower-limb joint angles velocity
lljangsv_e = MX.sym('lljangsv_e',nq.all-nq.abs-nq.arms-nq.trunk);
lljangsv_s = MX.sym('lljangsv_s',nq.all-nq.abs-nq.arms-nq.trunk);
lljangsv_sf = MX.sym('lljangsv_sf',nq.all-nq.abs-nq.arms-nq.trunk);
lljAngsv_temp = 0;
for i = 1:length(lljangsv_e)
    lljAngsv_temp = lljAngsv_temp + (((lljangsv_e(i)-lljangsv_s(i))./stateSF(1,7)).^2)./lljangsv_sf(i);
end
f_J_lljAngsv = Function('f_J_lljAngsv',{lljangsv_e,lljangsv_s,lljangsv_sf},{lljAngsv_temp});

% Upper-limb & trunk joint angles velocity
uljangsv_e = MX.sym('uljangsv_e',nq.arms+nq.trunk);
uljangsv_s = MX.sym('uljangsv_s',nq.arms+nq.trunk);
uljangsv_sf = MX.sym('uljangsv_sf',nq.arms+nq.trunk);
uljAngsv_temp = 0;
for i = 1:length(uljangsv_e)
    uljAngsv_temp = uljAngsv_temp + (((uljangsv_e(i)-uljangsv_s(i))./stateSF(1,8)).^2)./uljangsv_sf(i);
end
f_J_uljAngsv = Function('f_J_uljAngsv',{uljangsv_e,uljangsv_s,uljangsv_sf},{uljAngsv_temp});

% Acceleration scale factor
accSF = 1.0*ones(1,nq.all);

% Accelerations
accs = MX.sym('accs',nq.all);
accs_sf = MX.sym('accs_sf',nq.all);
accs_temp = 0;
for i = 1:length(accs)
    accs_temp = accs_temp + ((accs(i)./accSF(i)).^2)./accs_sf(i);
end
f_J_accs = Function('f_J_accs',{accs,accs_sf},{accs_temp});

% Residuals/Torques scale factor
torSF = [ones(nq.abs,1); 15*ones(nq.all-nq.abs,1)];

% Pelvis Residuals
resids = MX.sym('resids',nq.abs);
resids_sf = MX.sym('resids_sf',1);
resids_temp = 0;
for i = 1:length(resids)
    resids_temp = resids_temp + ((resids(i)./torSF(i)).^2)./resids_sf;
end
f_J_resids = Function('f_J_resids',{resids,resids_sf},{resids_temp});

% Joint Torques
tau_e = MX.sym('tau_e',nq.all-nq.abs);
tau_s = MX.sym('tau_s',nq.all-nq.abs);
tau_sf = MX.sym('tau_sf',1);
tau_temp = 0;
for i = 1:length(tau_e)
    tau_temp = tau_temp + (((tau_e(i)-tau_s(i))./torSF(nq.abs+i)).^2)./tau_sf;
end
f_J_tau = Function('f_J_tau',{tau_e,tau_s,tau_sf},{tau_temp});

% GRFs scale factor
grfSF = [10 20 10];

% GRFs
grf_e = MX.sym('grf_e',3);
grf_s = MX.sym('grf_s',3);
grf_sf = MX.sym('grf_sf',1);
grf_temp = 0;
for i = 1:length(grf_e)
    grf_temp = grf_temp + (((grf_e(i)-grf_s(i))./grfSF(i)).^2)./grf_sf;
end
f_J_grf = Function('f_J_grf',{grf_e,grf_s,grf_sf},{grf_temp});

% Muscle activations
muscle_act = MX.sym('muscle_act',NMuscle);
muscle_act_sf = MX.sym('muscle_act_sf',1);
muscle_act_temp = 0;
for i = 1:length(muscle_act)
    muscle_act_temp = muscle_act_temp + (((muscle_act(i))).^2)./muscle_act_sf;
end
f_J_muscle_act = Function('f_J_muscle_act',{muscle_act,muscle_act_sf},{muscle_act_temp});

% Derivative Tendon Forces Scale Factor
dFT_SF = 1*ones(NMuscle,1);

% Derivative Tendon Forces
dFT = MX.sym('dFT',NMuscle);
dFT_sf = MX.sym('dFT_sf',1);
dFT_temp = 0;
for i = 1:length(dFT)
    dFT_temp = dFT_temp + ((dFT(i)./dFT_SF(i)).^2)./dFT_sf;
end
f_J_dFT = Function('f_J_dFT',{dFT,dFT_sf},{dFT_temp});

% Derivative muscle activations
dMuscle_act_SF = 1*ones(NMuscle,1);
 
% Derivative Muscle activations
dMuscle_act = MX.sym('dMuscle_act',NMuscle);
dMuscle_act_sf = MX.sym('dMuscle_act_sf',1);
dMuscle_act_temp = 0;
for i = 1:length(dMuscle_act)
    dMuscle_act_temp = dMuscle_act_temp + (((dMuscle_act(i)./dMuscle_act_SF(i)).^2))./dMuscle_act_sf;
end
f_J_dMuscle_act = Function('f_J_dMuscle_act',{dMuscle_act,dMuscle_act_sf},{dMuscle_act_temp});

% Tendon Forces Scale Factor
FT_SF = 1*ones(NMuscle,1);

% Tendon Forces
FT = MX.sym('FT',NMuscle);
FT_sf = MX.sym('FT_sf',1);
FT_temp = 0;
for i = 1:length(FT)
    FT_temp = FT_temp + (((FT(i)./FT_SF(i)).^2))./FT_sf;
end
f_J_FT = Function('f_J_FT',{FT,FT_sf},{FT_temp});


%% 3. Muscle moment calculation functions

% Muscle-driven joint torques for the lower limbs and the trunk
% Define several CasADi functions for immediate use
% Function for 27 elements 
ma_temp27 = SX.sym('ma_temp27',27);
ft_temp27 = SX.sym('ft_temp27',27);
J_sptemp27 = 0;
for i=1:length(ma_temp27)
    J_sptemp27 = J_sptemp27 + ma_temp27(i,1)*ft_temp27(i,1);    
end
f_T27 = Function('f_T27',{ma_temp27,ft_temp27},{J_sptemp27});

% Function for 13 elements 
ma_temp13 = SX.sym('ma_temp13',13);
ft_temp13 = SX.sym('ft_temp13',13);
J_sptemp13 = 0;
for i=1:length(ma_temp13)
    J_sptemp13 = J_sptemp13 + ma_temp13(i,1)*ft_temp13(i,1);    
end
f_T13 = Function('f_T13',{ma_temp13,ft_temp13},{J_sptemp13});

% Function for 12 elements 
ma_temp12 = SX.sym('ma_temp12',12);
ft_temp12 = SX.sym('ft_temp12',12);
J_sptemp12 = 0;
for i=1:length(ma_temp12)
    J_sptemp12 = J_sptemp12 + ma_temp12(i,1)*ft_temp12(i,1);    
end
f_T12 = Function('f_T12',{ma_temp12,ft_temp12},{J_sptemp12});

% Function for 4 elements 
ma_temp4 = SX.sym('ma_temp4',4);
ft_temp4 = SX.sym('ft_temp4',4);
J_sptemp4 = 0;
for i=1:length(ma_temp4)
    J_sptemp4 = J_sptemp4 + ma_temp4(i,1)*ft_temp4(i,1);    
end
f_T4 = Function('f_T4',{ma_temp4,ft_temp4},{J_sptemp4});

% Function for 6 elements 
ma_temp6 = SX.sym('ma_temp6',6);
ft_temp6 = SX.sym('ft_temp6',6);
J_sptemp6 = 0;
for i=1:length(ma_temp6)
    J_sptemp6 = J_sptemp6 + ma_temp6(i,1)*ft_temp6(i,1);    
end
f_T6 = Function('f_T6',{ma_temp6,ft_temp6},{J_sptemp6});

end