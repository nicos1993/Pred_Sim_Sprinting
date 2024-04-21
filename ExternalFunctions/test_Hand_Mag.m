clc
clear

import casadi.*

F_cont_v21 = external('F_cont_v21','Spr_Imp_GRFs_ownCont_V21.dll');

F_cont_ana21 = external('F_cont_ana21','Spr_Imp_GRFs_ownCont_V21an.dll');

hand_r_pelvis_mag   = 123;
hand_l_pelvis_mag   = 124;

% % % x = zeros(74,1);
% % % 
% % % x(24) = deg2rad(-47); % shoulder flex
% % % x(27) = deg2rad(107); % elbow flex
% % % 
% % % x_ord = zeros(74,1);
% % % for i = 0:37-1
% % %     x_ord(i+1*(i+1),1) = x(i+1);
% % %     x_ord(2*(i+1),1) = x(37+i+1);
% % % end
% % % 
% % % u = zeros(37,1);
% % % c = zeros(24,1);
% % % 
% % % output = F_cont_v21([x_ord;u;c]);
% % % 
% % % dist_r = full(output(hand_r_pelvis_mag));
% % % dist_l = full(output(hand_l_pelvis_mag));

prevSol = load('IC_pred_Sprinting_optimum_p02_maxVel_01_24-October-2023__11-40-59___Nominal_CONS_BS_PUSH.mat');

n = length(prevSol.optimumOutput.timeGrid);

u = zeros(37,1);
c = zeros(24,1);

for i = 1:n

    x = zeros(74,1);
    %x(1:37) = prevSol.optimumOutput.optVars_nsc.q(:,i);
    x(1:37) = prevSol.optimumOutput.states.q_aux(i,:)';

    x_ord = zeros(74,1);
    for ii = 0:37-1
        x_ord(ii+1*(ii+1),1) = x(ii+1);
        x_ord(2*(ii+1),1) = x(37+ii+1);
    end
    
    output = F_cont_v21([x_ord;u;c]);

    dist_r(i,1) = full(output(hand_r_pelvis_mag));
    dist_l(i,1) = full(output(hand_l_pelvis_mag));

end
