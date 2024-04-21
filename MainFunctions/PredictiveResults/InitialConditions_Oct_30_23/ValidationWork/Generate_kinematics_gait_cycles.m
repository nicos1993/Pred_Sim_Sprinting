percent = linspace(0,100,101);

cf = 20;
        
% Filter coefficients
[b,a] = butter(2,(cf/(250/2)),'low');

i1_1 = find(m1(:,3+9) > 40); % Left foot contact

m1_1 = m1(i1_1,3+9);
t1_1 = m1(i1_1,1);



i1_2 = find(m1(:,3+27) > 40); % Right foot contact

m1_2 = m1(i1_2,3+27);
t1_2 = m1(i1_2,1);

t1_step = m1(i1_2(1):i1_1(1),1);
[~,m1_k_ind1] = min(abs(m1_k(:,1)-t1_step(1)));
[~,m1_k_ind2] = min(abs(m1_k(:,1)-t1_step(end)));

m1_k_filt = paddon(m1_k(:,2:end));
m1_k_filt = filtfilt(b,a,m1_k_filt);
m1_k_filt = paddoff(m1_k_filt);

y_m1_k = interp1(m1_k(m1_k_ind1:m1_k_ind2,1),m1_k_filt(m1_k_ind1:m1_k_ind2,:),t1_step,'spline');
t11_step = t1_step - t1_step(1);
y_m1_k_step = interp1((t11_step./t11_step(end))*100,y_m1_k,percent,'spline');

i2_1 = find(m2(:,3+9) > 40); % Right foot contact

m2_1 = m2(i2_1,3+9);
t2_1 = m2(i2_1,1);




i2_2 = find(m2(:,3+36) > 40); % Left foot contact

m2_2 = m2(i2_2,3+36);
t2_2 = m2(i2_2,1);

t2_step = m2(i2_2(1):i2_1(1),1);
[~,m2_k_ind1] = min(abs(m2_k(:,1)-t2_step(1)));
[~,m2_k_ind2] = min(abs(m2_k(:,1)-t2_step(end)));

m2_k_filt = paddon(m2_k(:,2:end));
m2_k_filt = filtfilt(b,a,m2_k_filt);
m2_k_filt = paddoff(m2_k_filt);

y_m2_k = interp1(m2_k(m2_k_ind1:m2_k_ind2,1),m2_k_filt(m2_k_ind1:m2_k_ind2,:),t2_step,'spline');
t22_step = t2_step - t2_step(1);
y_m2_k_step = interp1((t22_step./t22_step(end))*100,y_m2_k,percent,'spline');


opt_time = optimumOutput.timeGrid;
opt_time([4:4:end]) = [];
opt_time(end+1) = optimumOutput.timeGrid(end);
opt_time = opt_time - opt_time(1);

opt_q = optimumOutput.optVars_nsc.q';
opt_q([4:4:end],:) = [];
opt_q(end+1,:) = optimumOutput.optVars_nsc.q(:,end)';
opt_q(:,[1:3,7:end]) = rad2deg(opt_q(:,[1:3,7:end]));

y_opt_q = interp1((opt_time./opt_time(end))*100,opt_q,percent,'spline');




