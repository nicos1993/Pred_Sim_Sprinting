function [] = exploreMuscleModel()

% Parameters of force-length-velocity curves
load Fvparam.mat Fvparam % Velocity
load Fpparam.mat Fpparam % Passive
load Faparam.mat Faparam % Active

ATendon = 35;
c1 = 0.2;
c2 = 0.995;
c3 = 0.25;

act = 1;
lTtilde = linspace(0.2,1.8,500);
lMtilde = linspace(0.2,1.8,500);
vMtilde = linspace(-1,1,500);

for i = 1:length(lTtilde)
   for j = 1:length(vMtilde)
       [fTtilde(i),fAct(i),fPass(i),fV(j,i),Fcetilde(j,i),fComb(j,i),FMtilde1(i),FMtilde2(i),FMtilde3(i)] = evalMuscleModel(lTtilde(i),lMtilde(i),vMtilde(j),act);
   end
end

subplot(2,3,1)
plot(lTtilde,fTtilde);
xlim([0.99 1.05])
xlabel('norm tendon length');
ylabel('norm tendon force');
subplot(2,3,2)
plot(lMtilde,fAct);
xlabel('norm muscle fibre length');
ylabel('norm active muscle fibre force');
xlim([0.5 1.6])

subplot(2,3,3)
plot(lMtilde,fPass);
xlabel('norm muscle fibre length');
ylabel('norm passive muscle fibre force');
xlim([0.5 1.6])

close all

figure
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 21.0, 29.7], 'PaperUnits', 'centimeters', 'PaperSize', [21.0, 29.7])

Axes1 = axes;
set(Axes1, 'Units', 'centimeters', 'Position', [2, 16+5, 3.5, 2.5]);

plot(lTtilde,fTtilde,'-','LineWidth',1.5,'Color',[0 0 0]);
ylim([-0.048662884798974 1.1]);
xlim([0.995190380761523 1.05]);

set(Axes1,'FontSize',8);
set(Axes1,'LineWidth',1.0);
%set(Axes1,'Box','off');
set(Axes1,'TickDir','in');
set(Axes1,'TickLength',[0.015 0.1]);
set(Axes1,'xtick',[1.0 1.05])
%set(Axes1,'xticklabel',[])
%set(Axes1,'ytick',[1.0 1.05])
%set(Axes1,'yticklabel',[])
set(Axes1,'FontName','Times');

ylab1 = ylabel(['F_{SE}'],'fontweight','bold');
set(ylab1, 'Units', 'centimeters');
ylab1.Position(1) = -0.65;
xlabel(['L_{SE}'],'fontweight','bold');


Axes2 = axes;
set(Axes2, 'Units', 'centimeters', 'Position', [7, 16+5, 3.5, 2.5]);

plot(lMtilde,fAct,'-','LineWidth',1.5,'Color',[0 0 0]);
hold on
plot(lMtilde,fPass,':','LineWidth',1.5,'Color',[0 0 0]);
ylim([0 1.1]);
xlim([0.4 1.6]);

set(Axes2,'FontSize',8);
set(Axes2,'LineWidth',1.0);
%set(Axes1,'Box','off');
set(Axes2,'TickDir','in');
set(Axes2,'TickLength',[0.015 0.1]);
set(Axes2,'xtick',[0.5 1 1.5])
%set(Axes1,'xticklabel',[])
set(Axes2,'ytick',[0 0.5 1.0])
%set(Axes1,'yticklabel',[])
set(Axes2,'FontName','Times');

ylab2 = ylabel(['F_{CE}, F_{PE}'],'fontweight','bold');
set(ylab2, 'Units', 'centimeters');
ylab2.Position(1) = -0.65;
xlabel(['L_{CE}'],'fontweight','bold');


Axes3 = axes;
set(Axes3, 'Units', 'centimeters', 'Position', [12, 16+5, 3.5, 2.5]);

plot(vMtilde,fV(:,1),'-','LineWidth',1.5,'Color',[0 0 0]);
hold on
plot([0 0],[0 1.9],'-','LineWidth',1.0,'Color',[0 0 0]);
ylim([0 1.9]);
xlim([-1 1]);

set(Axes3,'FontSize',8);
set(Axes3,'LineWidth',1.0);
%set(Axes1,'Box','off');
set(Axes3,'TickDir','in');
set(Axes3,'TickLength',[0.015 0.1]);
set(Axes3,'xtick',[-1.0 0 1.0])
%set(Axes1,'xticklabel',[])
set(Axes3,'ytick',[0 0.5 1 1.5])
%set(Axes1,'yticklabel',[])
set(Axes3,'FontName','Times');

ylab3 = ylabel(['F-V_{CE}'],'fontweight','bold');
set(ylab3, 'Units', 'centimeters');
ylab3.Position(1) = -0.65;
xlabel(['V_{CE}'],'fontweight','bold');

% Activation Dynamics
time = linspace(0,1,201);
ta = 0.015;
td = 0.06;

t2 = 1/td;
t1 = 1/(ta-t2);

exct = zeros(201,1);
exct(51:121) = 1;
act = zeros(201,1);

for i = 1:length(time)-1
    act(i+1) = act(i) + (time(3)-time(2))*((exct(i)-act(i))*(t1*exct(i)+t2));
end


% % % % figure
% % % % set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 21.0, 29.7], 'PaperUnits', 'centimeters', 'PaperSize', [21.0, 29.7])

Axes4 = axes;
set(Axes4, 'Units', 'centimeters', 'Position', [7, 16+1, 3.5, 2.5]);

plot(time,exct,':','LineWidth',1.5,'Color',[0 0 0]);
hold on
plot(time,act,'-','LineWidth',1.5,'Color',[0 0 0]);
ylim([0 1]);
xlim([0 1]);

set(Axes4,'FontSize',8);
set(Axes4,'LineWidth',1.0);
%set(Axes1,'Box','off');
set(Axes4,'TickDir','in');
set(Axes4,'TickLength',[0.015 0.1]);
set(Axes4,'xtick',[0 0.5 1])
%set(Axes1,'xticklabel',[])
set(Axes4,'ytick',[0 0.5 1])
%set(Axes1,'yticklabel',[])
set(Axes4,'FontName','Times');

ylab1 = ylabel(['Excit, Act'],'fontweight','bold');
set(ylab1, 'Units', 'centimeters');
ylab1.Position(1) = -0.65;
xlabel(['Time (s)'],'fontweight','bold');

print(gcf,'muscle_plots.png','-dpng','-r600');


colors = [1 0 0; 0.9 0 0.1; 0.8 0 0.2; 0.7 0 0.3; 0.6 0 0.4; 0.5 0 0.5;...
    0.4 0 0.6; 0.3 0 0.7; 0.2 0 0.8; 0.1 0 0.9]; 

subplot(2,3,4)
for j = 1:10
    plot(lMtilde,Fcetilde((j-1)*50+1,:),'color',colors(j,:));
    hold on
end
annotation('textbox',[0.47 0.3 0.1 0.1],'string',{'norm shortening velocity = -1'},'edgecolor',[1 1 1],'color',[1 0 0]);
annotation('textbox',[0.47 0.27 0.1 0.1],'string',{'norm lengthening velocity = +1'},'edgecolor',[1 1 1],'color',[0.1 0 0.9]);

xlabel('norm muscle fibre length');
ylabel('norm active force-length & - velocity curve');
ylim([0.5 1.8])
hold off

subplot(2,3,5)
for j = 1:10
    plot(lMtilde,fComb((j-1)*50+1,:),'color',colors(j,:));
    hold on
end
xlabel('norm muscle fibre length');
ylabel('norm active force-length & - velocity curve plus passive');
ylim([0.5 1.8]);
hold off

subplot(2,3,6)
plot(vMtilde,fV);
xlabel('norm muscle fibre velocity');
ylabel('norm active force-velocity curve');
pause();


    function [fTtilde,fAct,fPass,fV,Fcetilde,fComb,FMtilde1,FMtilde2,FMtilde3] = evalMuscleModel(lTtilde,lMtilde,vMtilde,act)
        
        fTtilde = c1*exp(ATendon*(lTtilde-c2))-c3;
        
        b11 = Faparam(1);
        b21 = Faparam(2);
        b31 = Faparam(3);
        b41 = Faparam(4);
        b12 = Faparam(5);
        b22 = Faparam(6);
        b32 = Faparam(7);
        b42 = Faparam(8);
        b13 = 0.1;
        b23 = 1;
        b33 = 0.5*sqrt(0.5);
        b43 = 0;
        num3 = lMtilde-b23;
        den3 = b33+b43*lMtilde;
        FMtilde3 = b13*exp(-0.5*num3.^2./den3.^2);
        num1 = lMtilde-b21;
        den1 = b31+b41*lMtilde;
        FMtilde1 = b11*exp(-0.5*num1.^2./den1.^2);
        num2 = lMtilde-b22;
        den2 = b32+b42*lMtilde;
        FMtilde2 = b12*exp(-0.5*num2.^2./den2.^2);
        FMltilde = FMtilde1+FMtilde2+FMtilde3;
        fAct = FMltilde;
        
        e1 = Fvparam(1);
        e2 = Fvparam(2);
        e3 = Fvparam(3);
        e4 = Fvparam(4);
        fV = e1*log((e2*vMtilde+e3)+sqrt((e2*vMtilde+e3).^2+1))+e4;
        
        % Active muscle force
        d = 0.01; % damping coefficient
        Fcetilde = act.*fAct.*fV +d*vMtilde;
        
        e0 = 0.6;
        kpe = 4;
        t5 = exp(kpe * (lMtilde - 0.10e1) / e0);
        % Passive muscle force
        fPass = ((t5 - 0.10e1) - Fpparam(1)) / Fpparam(2);
        
        fComb = Fcetilde + fPass;
        
    end

end