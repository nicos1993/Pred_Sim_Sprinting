% Plotting of optimal simulation speed together with +/- 6 cm Horizontal
% touchdown distance and +/- 6 cm IKTD

figure;

% Define the dimensions for the subplots
subplot1_dims = [0.1, 0.6, 0.25, 0.25];  % [left, bottom, width, height]
subplot4_dims = [0.1, 0.2, 0.25, 0.25];


td_d_pert = 100.*[pred_15_006_td_p_td pred_15_004_td_p_td pred_15_002_td_p_td pred_15_td pred_15_002_td_m_td pred_15_004_td_m_td pred_15_006_td_m_td];
spe_td_d_pert = [pred_15_006_td_p.optimumOutput.ave_speed pred_15_004_td_p.optimumOutput.ave_speed pred_15_002_td_p.optimumOutput.ave_speed pred_15.optimumOutput.ave_speed pred_15_002_td_m.optimumOutput.ave_speed pred_15_004_td_m.optimumOutput.ave_speed pred_15_006_td_m.optimumOutput.ave_speed];

subplot('Position', subplot1_dims);
scatter(td_d_pert,spe_td_d_pert,'filled','SizeData', 200,'MarkerFaceColor', [0.996, 0.298, 0.247])
hold on
scatter(td_d_pert(4),spe_td_d_pert(4),'filled','SizeData',200,'MarkerFaceColor',[0.663, 0.431, 0.808]);
%scatter(pred_1_5_td*100,pred_1_5_spe,'filled','SizeData',500,'MarkerFaceColor',[0.396, 0.741, 0.380]);
xlabel('Horizontal touchdown distance (cm)','fontweight','bold')
ylabel('Speed (m/s)','fontweight','bold');
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
xlim([100*pred_15_td-8 100*pred_15_td+8])
ylim([10.8 12.0])
yticks([[10.8:0.3:12.0]])
xticks([[round(100*pred_15_td-8):3:round(100*pred_15_td+8)]])
title('a')
box on

ik_d_pert = 100.*[pred_15_006_ik_p_ik pred_15_004_ik_p_ik pred_15_002_ik_p_ik pred_15_ik pred_15_002_ik_m_ik pred_15_004_ik_m_ik pred_15_006_ik_m_ik];
spe_ik_d_pert = [pred_15_006_ik_p.optimumOutput.ave_speed pred_15_004_ik_p.optimumOutput.ave_speed pred_15_002_ik_p.optimumOutput.ave_speed pred_15.optimumOutput.ave_speed...
                 pred_15_002_ik_m.optimumOutput.ave_speed pred_15_004_ik_m.optimumOutput.ave_speed pred_15_006_ik_m.optimumOutput.ave_speed];

subplot('Position', subplot4_dims);
scatter(ik_d_pert,spe_ik_d_pert,'filled','SizeData', 200,'MarkerFaceColor', [0.996, 0.298, 0.247])
hold on
scatter(ik_d_pert(4),spe_ik_d_pert(4),'filled','SizeData',200,'MarkerFaceColor',[0.663, 0.431, 0.808]);

%scatter(pred_1_5_ik*100,pred_1_5_spe,'filled','SizeData',500,'MarkerFaceColor',[0.396, 0.741, 0.380]);
xlabel('Inter-knee touchdown distance (cm)','fontweight','bold')
ylabel('Speed (m/s)','fontweight','bold');
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
xlim([floor(100*pred_15_ik-8) floor(100*pred_15_ik+8)])
ylim([10.8 12.0])
yticks([[10.8:0.3:12.0]])
xticks([[floor(100*pred_15_ik-8):3:floor(100*pred_15_ik+8)]])
title('b')
box on
