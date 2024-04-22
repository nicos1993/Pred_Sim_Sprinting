
percent = linspace(0,100,101);

i1_1 = find(m1(:,3+9) > 40);

m1_1 = m1(i1_1,3+9);
t1_1 = m1(i1_1,1);
t1_1 = t1_1 - t1_1(1);

y1_1 = interp1((t1_1/t1_1(end))*100,m1_1,percent,'spline');




i1_2 = find(m1(:,3+27) > 40);

m1_2 = m1(i1_2,3+27);
t1_2 = m1(i1_2,1);
t1_2 = t1_2 - t1_2(1);

y1_2 = interp1((t1_2/t1_2(end))*100,m1_2,percent,'spline');





i2_1 = find(m2(:,3+9) > 40);

m2_1 = m2(i2_1,3+9);
t2_1 = m2(i2_1,1);
t2_1 = t2_1 - t2_1(1);

y2_1 = interp1((t2_1/t2_1(end))*100,m2_1,percent,'spline');




i2_2 = find(m2(:,3+36) > 40);

m2_2 = m2(i2_2,3+36);
t2_2 = m2(i2_2,1);
t2_2 = t2_2 - t2_2(1);

y2_2 = interp1((t2_2/t2_2(end))*100,m2_2,percent,'spline');


opt_grf = optimumOutput.GRFs.R(:,2);

opt_time = optimumOutput.timeNodes(find(opt_grf > 39.9),1);
opt_time = opt_time - opt_time(1);

opt_grf = opt_grf(find(opt_grf > 39.9),1);

y_opt_grf = interp1(opt_time./opt_time(end)*100,opt_grf,percent,'spline');


