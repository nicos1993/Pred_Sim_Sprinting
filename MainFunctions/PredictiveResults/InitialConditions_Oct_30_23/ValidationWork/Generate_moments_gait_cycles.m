percent = linspace(0,100,101);


t1 = file1.timeNodes';
t1 = t1 - t1(1);

njm1 = file1.momentsID;

y_njm1 = interp1((t1./t1(end))*100,njm1',percent,'spline');



t2 = file2.timeNodes';
t2 = t2 - t2(1);

njm2 = file2.momentsID;

y_njm2 = interp1((t2./t2(end))*100,njm2',percent,'spline');



t_opt = pred_15.optimumOutput.timeNodes(1:56);
t_opt = t_opt - t_opt(1);

njm_opt = pred_15.optimumOutput.muscleMoments.joints(:,1:56);

y_njm_opt = interp1((t_opt/t_opt(end))*100,njm_opt',percent,'spline');