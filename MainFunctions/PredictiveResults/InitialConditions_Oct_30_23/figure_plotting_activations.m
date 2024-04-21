% Plotting stride activations together with experimental EMG data

acts = load('pred_15_symmetric_activations_data.mat');

acts_time = acts.actData.data(:,1);
acts_time([4:4:end]) = [];
acts_time(end+1,1) = acts.actData.data(end,1);

acts_data = acts.actData.data(:,2:end);
acts_data([4:4:end],:) = [];
acts_data(end+1,:) = acts.actData.data(end,2:end);

acts_t = linspace(acts_time(1),acts_time(end),length(acts_time))';
acts_tn = linspace(0,100,length(acts_time))';

acts_n = interp1(acts_time,acts_data,acts_t,'spline');

takeoff_p = 18.6;

ham1x = [0 0 23 23]; ham1y = [0 1 1 0];
ham2x = [72 72 100 100]; ham2y = [0 1 1 0];

ham1x_sd = [23 30]; ham1y_sd = [0.5 0.5];
ham2x_sd = [67 72]; ham2y_sd = [0.5 0.5];

rf1x = [0 0 16 16]; rf1y = [0 1 1 0];
rf2x = [36 36 63 63]; rf2y = [0 1 1 0];
rf3x = [88 88 100 100]; rf3y = [0 1 1 0];

rf2x_sd = [28 36]; rf2y_sd = [0.5 0.5];
rf3x_sd = [63 65.5]; rf3y_sd = [0.5 0.5];
rf4x_sd = [86 88]; rf4y_sd = [0.5 0.5];

vl1x = [0 0 23.5 23.5]; vl1y = [0 1 1 0];
vl2x = [79 79 100 100]; vl2y = [0 1 1 0];

vl1x_sd = [23.5 32.5]; vl1y_sd = [0.5 0.5];
vl2x_sd = [66 79]; vl2y_sd = [0.5 0.5];

gm1x = [0 0 15 15]; gm1y = [0 1 1 0];
gm2x = [77 77 100 100]; gm2y = [0 1 1 0];

g1x = [0 0 19 19]; g1y = [0 1 1 0];
g2x = [77 77 100 100]; g2y = [0 1 1 0];

g1x_sd = [19 23.5]; g1y_sd = [0.5 0.5];
g2x_sd = [71 77]; g2y_sd = [0.5 0.5];

s1x = [0 0 28 28]; s1y = [0 1 1 0];
s2x = [81 81 100 100]; s2y = [0 1 1 0];

s1x_sd = [28 30.5]; s1y_sd = [0.5 0.5];
s2x_sd = [76 81]; s2y_sd = [0.5 0.5];

ta1x = [0 0 16 16]; ta1y = [0 1 1 0];
ta2x = [52 52 100 100]; ta2y = [0 1 1 0];

ta1x_sd = [16 18.5]; ta1y_sd = [0.5 0.5];
ta2x_sd = [30.5 52]; ta2y_sd = [0.5 0.5];

figure;

% Define the dimensions for the subplots
subplot1_dims = [0.1, 0.7, 0.25, 0.25];  % [left, bottom, width, height]
subplot2_dims = [0.4, 0.7, 0.25, 0.25];
subplot3_dims = [0.7, 0.7, 0.25, 0.25];
subplot4_dims = [0.1, 0.4, 0.25, 0.25];
subplot5_dims = [0.4, 0.4, 0.25, 0.25];
subplot6_dims = [0.7, 0.4, 0.25, 0.25];
subplot7_dims = [0.1, 0.1, 0.25, 0.25];


subplot('Position', subplot1_dims);
plot(acts_tn,acts_n(:,53),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]) % Right Semimem
hold on
xline(takeoff_p,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
fill(ham1x,ham1y,'k','FaceAlpha',0.1,'EdgeColor','none');
fill(ham2x,ham2y,'k','FaceAlpha',0.1,'EdgeColor','none');
line(ham1x_sd,ham1y_sd,'LineStyle','-','Color','black','LineWidth',2);
line([ham1x_sd(2) ham1x_sd(2)],[0.47 0.53],'LineStyle','-','Color','black','LineWidth',2);
line(ham2x_sd,ham2y_sd,'LineStyle','-','Color','black','LineWidth',2);
line([ham2x_sd(1) ham2x_sd(1)],[0.47 0.53],'LineStyle','-','Color','black','LineWidth',2);
ylabel({'Activation (-)'},'fontweight','bold');
title('Hamstring');
ylim([0 1])
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
xticklabels([])

subplot('Position', subplot2_dims);
plot(acts_tn,acts_n(:,74),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]) % Right RF
hold on
xline(takeoff_p,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
fill(rf1x,rf1y,'k','FaceAlpha',0.1,'EdgeColor','none');
fill(rf2x,rf2y,'k','FaceAlpha',0.1,'EdgeColor','none');
fill(rf3x,rf3y,'k','FaceAlpha',0.1,'EdgeColor','none');
line(rf2x_sd,rf2y_sd,'LineStyle','-','Color','black','LineWidth',2);
line([rf2x_sd(1) rf2x_sd(1)],[0.47 0.53],'LineStyle','-','Color','black','LineWidth',2);
line(rf3x_sd,rf3y_sd,'LineStyle','-','Color','black','LineWidth',2);
line([rf3x_sd(2) rf3x_sd(2)],[0.47 0.53],'LineStyle','-','Color','black','LineWidth',2);
line(rf4x_sd,rf4y_sd,'LineStyle','-','Color','black','LineWidth',2);
line([rf4x_sd(1) rf4x_sd(1)],[0.47 0.53],'LineStyle','-','Color','black','LineWidth',2);
title('Rectus Femoris');
ylim([0 1])
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
xticklabels([])

subplot('Position', subplot3_dims);
plot(acts_tn,acts_n(:,77),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]) % Right VL
hold on
xline(takeoff_p,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
fill(vl1x,vl1y,'k','FaceAlpha',0.1,'EdgeColor','none');
fill(vl2x,vl2y,'k','FaceAlpha',0.1,'EdgeColor','none');
line(vl1x_sd,vl1y_sd,'LineStyle','-','Color','black','LineWidth',2);
line([vl1x_sd(2) vl1x_sd(2)],[0.47 0.53],'LineStyle','-','Color','black','LineWidth',2);
line(vl2x_sd,vl2y_sd,'LineStyle','-','Color','black','LineWidth',2);
line([vl2x_sd(1) vl2x_sd(1)],[0.47 0.53],'LineStyle','-','Color','black','LineWidth',2);
title('Vastus Lateralis');
ylim([0 1])
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
xticklabels([])

subplot('Position', subplot4_dims);
plot(acts_tn,mean([acts_n(:,66) acts_n(:,67) acts_n(:,68)],2),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]) % Right GMAX
hold on
xline(takeoff_p,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
fill(gm1x,gm1y,'k','FaceAlpha',0.1,'EdgeColor','none');
fill(gm2x,gm2y,'k','FaceAlpha',0.1,'EdgeColor','none');
ylabel({'Activation (-)'},'fontweight','bold');
title('Gluteus Maximus');
ylim([0 1])
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
xticklabels([])

subplot('Position', subplot5_dims);
plot(acts_tn,mean([acts_n(:,78) acts_n(:,79)],2),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]) % Right GAS
hold on
xline(takeoff_p,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
fill(g1x,g1y,'k','FaceAlpha',0.1,'EdgeColor','none');
fill(g2x,g2y,'k','FaceAlpha',0.1,'EdgeColor','none');
line(g1x_sd,g1y_sd,'LineStyle','-','Color','black','LineWidth',2);
line([g1x_sd(2) g1x_sd(2)],[0.47 0.53],'LineStyle','-','Color','black','LineWidth',2);
line(g2x_sd,g2y_sd,'LineStyle','-','Color','black','LineWidth',2);
line([g2x_sd(1) g2x_sd(1)],[0.47 0.53],'LineStyle','-','Color','black','LineWidth',2);
title('Gastrocnemius');
ylim([0 1])
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
xlim([0.0 100])
xlabel('Stride (%)','fontweight','bold')

subplot('Position', subplot6_dims);
plot(acts_tn,acts_n(:,80),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]) % Right SOL
hold on
xline(takeoff_p,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
fill(s1x,s1y,'k','FaceAlpha',0.1,'EdgeColor','none');
fill(s2x,s2y,'k','FaceAlpha',0.1,'EdgeColor','none');
line(s1x_sd,s1y_sd,'LineStyle','-','Color','black','LineWidth',2);
line([s1x_sd(2) s1x_sd(2)],[0.47 0.53],'LineStyle','-','Color','black','LineWidth',2);
line(s2x_sd,s2y_sd,'LineStyle','-','Color','black','LineWidth',2);
line([s2x_sd(1) s2x_sd(1)],[0.47 0.53],'LineStyle','-','Color','black','LineWidth',2);
title('Soleus');
ylim([0 1])
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
xlim([0.0 100])
xlabel('Stride (%)','fontweight','bold')

subplot('Position', subplot7_dims);
plot(acts_tn,acts_n(:,84),'LineWidth',3,'Color',[0.996, 0.298, 0.247 1]) % Right TA
hold on
xline(takeoff_p,'--','LineWidth',1.5,'Color',[0.996, 0.298, 0.247],'Alpha',1);
fill(ta1x,ta1y,'k','FaceAlpha',0.1,'EdgeColor','none');
fill(ta2x,ta2y,'k','FaceAlpha',0.1,'EdgeColor','none');
line(ta1x_sd,ta1y_sd,'LineStyle','-','Color','black','LineWidth',2);
line([ta1x_sd(2) ta1x_sd(2)],[0.47 0.53],'LineStyle','-','Color','black','LineWidth',2);
line(ta2x_sd,ta2y_sd,'LineStyle','-','Color','black','LineWidth',2);
line([ta2x_sd(1) ta2x_sd(1)],[0.47 0.53],'LineStyle','-','Color','black','LineWidth',2);
ylabel({'Activation (-)'},'fontweight','bold');
title('Tibialis Anterior');
ylim([0 1])
set(gca,'fontname','arial')
set(gca,'fontsize',18.0)
set(gca,'linewidth',2)
xlim([0.0 100])
xlabel('Stride (%)','fontweight','bold')