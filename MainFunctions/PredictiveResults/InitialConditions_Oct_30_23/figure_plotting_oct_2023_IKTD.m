% Plotting of optimal simulation together with +/- 6 cm Horizontal
% touchdown distance simulations (worse performing)


%% Joint Moments
figure;

% Define the dimensions for the subplotsx
subplot1_dims = [0.1, 0.6, 0.25, 0.25];  % [left, bottom, width, height]
subplot2_dims = [0.4, 0.6, 0.25, 0.25];
subplot3_dims = [0.7, 0.6, 0.25, 0.25];
subplot4_dims = [0.1, 0.25, 0.25, 0.25];
subplot5_dims = [0.4, 0.25, 0.25, 0.25];
subplot6_dims = [0.7, 0.25, 0.25, 0.25];

subplot('Position', subplot1_dims);
plot(pred_15.optimumOutput.timeNodes-0.056,pred_15.optimumOutput.muscleMoments.joints(11,:),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeNodes-0.056,pred_15_006_ik_p.optimumOutput.muscleMoments.joints(11,:),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeNodes-0.056,pred_15_006_ik_m.optimumOutput.muscleMoments.joints(11,:),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
ylabel({'Moment (N.m)'},'fontweight','bold')
title('R Ankle Dorsiflexion (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xticklabels([])
xlim([0.0 0.212])
legend({'Optimal','+6 cm','-6 cm'},'Location','southeast','FontSize',14)
legend('boxoff')

subplot('Position', subplot2_dims);
plot(pred_15.optimumOutput.timeNodes-0.056,pred_15.optimumOutput.muscleMoments.joints(10,:),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeNodes-0.056,pred_15_006_ik_p.optimumOutput.muscleMoments.joints(10,:),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeNodes-0.056,pred_15_006_ik_m.optimumOutput.muscleMoments.joints(10,:),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
%ylabel({'Moment (N.m)'},'fontweight','bold')
title('R Knee Extension (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xticklabels([])
xlim([0.0 0.212])

subplot('Position', subplot3_dims);
plot(pred_15.optimumOutput.timeNodes-0.056,pred_15.optimumOutput.muscleMoments.joints(7,:),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeNodes-0.056,pred_15_006_ik_p.optimumOutput.muscleMoments.joints(7,:),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeNodes-0.056,pred_15_006_ik_m.optimumOutput.muscleMoments.joints(7,:),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
%ylabel({'Moment (N.m)'},'fontweight','bold')
title('R Hip Flexion (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xticklabels([])
xlim([0.0 0.212])

subplot('Position', subplot4_dims);
plot(pred_15.optimumOutput.timeNodes-0.056,pred_15.optimumOutput.muscleMoments.joints(18,:),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeNodes-0.056,pred_15_006_ik_p.optimumOutput.muscleMoments.joints(18,:),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeNodes-0.056,pred_15_006_ik_m.optimumOutput.muscleMoments.joints(18,:),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
ylabel({'Moment (N.m)'},'fontweight','bold')
title('L Ankle Dorsiflexion (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xlim([0.0 0.212])
xlabel('Step time (s)','fontweight','bold')

subplot('Position', subplot5_dims);
plot(pred_15.optimumOutput.timeNodes-0.056,pred_15.optimumOutput.muscleMoments.joints(17,:),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeNodes-0.056,pred_15_006_ik_p.optimumOutput.muscleMoments.joints(17,:),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeNodes-0.056,pred_15_006_ik_m.optimumOutput.muscleMoments.joints(17,:),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
%ylabel({'Moment (N.m)'},'fontweight','bold')
title('L Knee Extension (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xlim([0.0 0.212])
xlabel('Step time (s)','fontweight','bold')

subplot('Position', subplot6_dims);
plot(pred_15.optimumOutput.timeNodes-0.056,pred_15.optimumOutput.muscleMoments.joints(14,:),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeNodes-0.056,pred_15_006_ik_p.optimumOutput.muscleMoments.joints(14,:),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeNodes-0.056,pred_15_006_ik_m.optimumOutput.muscleMoments.joints(14,:),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
%ylabel({'Moment (N.m)'},'fontweight','bold')
title('L Hip Flexion (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xlim([0.0 0.212])
xlabel('Step time (s)','fontweight','bold')



%% Joint Kinematics
% Lower-limb
figure;

subplot('Position', subplot1_dims);
plot(pred_15.optimumOutput.timeGrid-0.056,rad2deg(pred_15.optimumOutput.optVars_nsc.q(11,:)),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_p.optimumOutput.optVars_nsc.q(11,:)),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_m.optimumOutput.optVars_nsc.q(11,:)),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
title('R Ankle Dorsiflexion (+)')
ylabel({'Angle (\circ)'},'fontweight','bold')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xticklabels([])
xlim([0.0 0.212])

subplot('Position', subplot2_dims);
plot(pred_15.optimumOutput.timeGrid-0.056,rad2deg(pred_15.optimumOutput.optVars_nsc.q(10,:)),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_p.optimumOutput.optVars_nsc.q(10,:)),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_m.optimumOutput.optVars_nsc.q(10,:)),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
title('R Knee Extension (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xticklabels([])
xlim([0.0 0.212])

subplot('Position', subplot3_dims);
plot(pred_15.optimumOutput.timeGrid-0.056,rad2deg(pred_15.optimumOutput.optVars_nsc.q(7,:)),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_p.optimumOutput.optVars_nsc.q(7,:)),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_m.optimumOutput.optVars_nsc.q(7,:)),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
title('R Hip Flexion (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xticklabels([])
xlim([0.0 0.212])

subplot('Position', subplot4_dims);
plot(pred_15.optimumOutput.timeGrid-0.056,rad2deg(pred_15.optimumOutput.optVars_nsc.q(18,:)),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_p.optimumOutput.optVars_nsc.q(18,:)),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_m.optimumOutput.optVars_nsc.q(18,:)),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
title('L Ankle Dorsiflexion (+)')
ylabel({'Angle (\circ)'},'fontweight','bold')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xlim([0.0 0.212])
xlabel('Step time (s)','fontweight','bold')

subplot('Position', subplot5_dims);
plot(pred_15.optimumOutput.timeGrid-0.056,rad2deg(pred_15.optimumOutput.optVars_nsc.q(17,:)),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_p.optimumOutput.optVars_nsc.q(17,:)),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_m.optimumOutput.optVars_nsc.q(17,:)),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
title('L Knee Extension (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xlim([0.0 0.212])
xlabel('Step time (s)','fontweight','bold')
legend({'Optimal','+6 cm','-6 cm'},'Location','southeast','FontSize',14)
legend('boxoff')

subplot('Position', subplot6_dims);
plot(pred_15.optimumOutput.timeGrid-0.056,rad2deg(pred_15.optimumOutput.optVars_nsc.q(14,:)),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_p.optimumOutput.optVars_nsc.q(14,:)),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_m.optimumOutput.optVars_nsc.q(14,:)),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
title('L Hip Flexion (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xlim([0.0 0.212])
xlabel('Step time (s)','fontweight','bold')

%% GRFs
figure;

subplot('Position', subplot1_dims);
plot(pred_15.optimumOutput.timeNodes-0.056,pred_15.optimumOutput.GRFs.R(:,1),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeNodes-0.056,pred_15_006_ik_p.optimumOutput.GRFs.R(:,1),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeNodes-0.056,pred_15_006_ik_m.optimumOutput.GRFs.R(:,1),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
ylabel({'Ground Reaction Force (N)'},'fontweight','bold')
title('Anterior-Posterior')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xlim([0.0 0.212])
xlabel('Step time (s)','fontweight','bold')

subplot('Position', subplot2_dims);
plot(pred_15.optimumOutput.timeNodes-0.056,pred_15.optimumOutput.GRFs.R(:,2),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeNodes-0.056,pred_15_006_ik_p.optimumOutput.GRFs.R(:,2),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeNodes-0.056,pred_15_006_ik_m.optimumOutput.GRFs.R(:,2),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
%ylabel({'Ground Reaction Force (N)'},'fontweight','bold')
title('Vertical')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xlim([0.0 0.212])
xlabel('Step time (s)','fontweight','bold')
legend({'Optimal','+6 cm','-6 cm'},'Location','southeast','FontSize',14)
legend('boxoff')

%% Kinematics
% Pelvis & Trunk
figure;

subplot('Position', subplot1_dims);
plot(pred_15.optimumOutput.timeGrid-0.056,rad2deg(pred_15.optimumOutput.optVars_nsc.q(1,:)),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_p.optimumOutput.optVars_nsc.q(1,:)),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_m.optimumOutput.optVars_nsc.q(1,:)),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
title('Pelvis (+)')
ylabel({'Angle (\circ)'},'fontweight','bold')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xticklabels([])
xlim([0.0 0.212])

subplot('Position', subplot2_dims);
plot(pred_15.optimumOutput.timeGrid-0.056,rad2deg(pred_15.optimumOutput.optVars_nsc.q(2,:)),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_p.optimumOutput.optVars_nsc.q(2,:)),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_m.optimumOutput.optVars_nsc.q(2,:)),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
title('Pelvis (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xticklabels([])
xlim([0.0 0.212])

subplot('Position', subplot3_dims);
plot(pred_15.optimumOutput.timeGrid-0.056,rad2deg(pred_15.optimumOutput.optVars_nsc.q(3,:)),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_p.optimumOutput.optVars_nsc.q(3,:)),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_m.optimumOutput.optVars_nsc.q(3,:)),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
title('Pelvis (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xticklabels([])
xlim([0.0 0.212])

subplot('Position', subplot4_dims);
plot(pred_15.optimumOutput.timeGrid-0.056,rad2deg(pred_15.optimumOutput.optVars_nsc.q(21,:)),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_p.optimumOutput.optVars_nsc.q(21,:)),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_m.optimumOutput.optVars_nsc.q(21,:)),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
title('Trunk (+)')
ylabel({'Angle (\circ)'},'fontweight','bold')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xlim([0.0 0.212])
xlabel('Step time (s)','fontweight','bold')

subplot('Position', subplot5_dims);
plot(pred_15.optimumOutput.timeGrid-0.056,rad2deg(pred_15.optimumOutput.optVars_nsc.q(22,:)),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_p.optimumOutput.optVars_nsc.q(22,:)),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_m.optimumOutput.optVars_nsc.q(22,:)),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
title('Trunk (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xlim([0.0 0.212])
xlabel('Step time (s)','fontweight','bold')
legend({'Optimal','+6 cm','-6 cm'},'Location','southeast','FontSize',14)
legend('boxoff')

subplot('Position', subplot6_dims);
plot(pred_15.optimumOutput.timeGrid-0.056,rad2deg(pred_15.optimumOutput.optVars_nsc.q(23,:)),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_p.optimumOutput.optVars_nsc.q(23,:)),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeGrid-0.056,rad2deg(pred_15_006_ik_m.optimumOutput.optVars_nsc.q(23,:)),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
title('Trunk (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xlim([0.0 0.212])
xlabel('Step time (s)','fontweight','bold')


%% Joint Powers
figure;

% Define the dimensions for the subplots
subplot1_dims = [0.1, 0.6, 0.25, 0.25];  % [left, bottom, width, height]
subplot2_dims = [0.4, 0.6, 0.25, 0.25];
subplot3_dims = [0.7, 0.6, 0.25, 0.25];
subplot4_dims = [0.1, 0.25, 0.25, 0.25];
subplot5_dims = [0.4, 0.25, 0.25, 0.25];
subplot6_dims = [0.7, 0.25, 0.25, 0.25];

subplot('Position', subplot1_dims);
plot(pred_15.optimumOutput.timeNodes-0.056,pred_15_power(11,:),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeNodes-0.056,pred_15_006_ik_p_power(11,:),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeNodes-0.056,pred_15_006_ik_m_power(11,:),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
ylabel({'Moment (N.m)'},'fontweight','bold')
title('R Ankle Dorsiflexion (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xticklabels([])
xlim([0.0 0.212])
legend({'Optimal','+6 cm','-6 cm'},'Location','southeast','FontSize',14)
legend('boxoff')

subplot('Position', subplot2_dims);
plot(pred_15.optimumOutput.timeNodes-0.056,pred_15_power(10,:),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeNodes-0.056,pred_15_006_ik_p_power(10,:),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeNodes-0.056,pred_15_006_ik_m_power(10,:),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
%ylabel({'Moment (N.m)'},'fontweight','bold')
title('R Knee Extension (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xticklabels([])
xlim([0.0 0.212])

subplot('Position', subplot3_dims);
plot(pred_15.optimumOutput.timeNodes-0.056,pred_15_power(7,:),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeNodes-0.056,pred_15_006_ik_p_power(7,:),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeNodes-0.056,pred_15_006_ik_m_power(7,:),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
%ylabel({'Moment (N.m)'},'fontweight','bold')
title('R Hip Flexion (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xticklabels([])
xlim([0.0 0.212])

subplot('Position', subplot4_dims);
plot(pred_15.optimumOutput.timeNodes-0.056,pred_15_power(18,:),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeNodes-0.056,pred_15_006_ik_p_power(18,:),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeNodes-0.056,pred_15_006_ik_m_power(18,:),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
ylabel({'Moment (N.m)'},'fontweight','bold')
title('L Ankle Dorsiflexion (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xlim([0.0 0.212])
xlabel('Step time (s)','fontweight','bold')

subplot('Position', subplot5_dims);
plot(pred_15.optimumOutput.timeNodes-0.056,pred_15_power(17,:),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeNodes-0.056,pred_15_006_ik_p_power(17,:),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeNodes-0.056,pred_15_006_ik_m_power(17,:),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
%ylabel({'Moment (N.m)'},'fontweight','bold')
title('L Knee Extension (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xlim([0.0 0.212])
xlabel('Step time (s)','fontweight','bold')

subplot('Position', subplot6_dims);
plot(pred_15.optimumOutput.timeNodes-0.056,pred_15_power(14,:),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]);
hold on
plot(pred_15_006_ik_p.optimumOutput.timeNodes-0.056,pred_15_006_ik_p_power(14,:),'LineWidth',3,'Color',[0.663, 0.431, 0.808 1]);
plot(pred_15_006_ik_m.optimumOutput.timeNodes-0.056,pred_15_006_ik_m_power(14,:),'LineWidth',3,'Color',[0.396, 0.741, 0.380 1]);
%ylabel({'Moment (N.m)'},'fontweight','bold')
title('L Hip Flexion (+)')
xline(pred_15_to_i-0.056,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
xline(pred_15_006_ik_p_to_i-0.056,'--','LineWidth',1.5,'Color',[0.663, 0.431, 0.808],'Alpha',1);
xline(pred_15_006_ik_m_to_i-0.056,'--','LineWidth',1.5,'Color',[0.396, 0.741, 0.380],'Alpha',1);
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
%xticks([0.0 0.05 0.1 0.15 0.2 0.25])
xlim([0.0 0.212])
xlabel('Step time (s)','fontweight','bold')



