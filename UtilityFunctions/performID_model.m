clear all

import casadi.*
d = 3; % degree of interpolating polynomial
method = 'radau'; % collocation method
[tau_root,C,D,B] = CollocationScheme(d,method);


load('pred_Sprinting_maxSpeed_50_meshInts_optimum_STRp02_maxVel_01_24-May-2022__08-52-15.mat');

q = optimumOutput.states.q_aux;
qdot = optimumOutput.states.qdot_aux;
qddot = optimumOutput.states.qddot_aux;

% load grfs
nom_grf_fname = 'p02_m_01_labelled_grf.mot';
nom_grf_f = readMOT(nom_grf_fname);
nom_grf_f = nom_grf_f.data;

% some filtering of grfs
[b,a] = butter(2,(20/(2000/2)),'low');
padGRF = paddon(nom_grf_f);

plate = 4;

filtGRF = zeros(length(padGRF(:,1)),9);

for i = 1:3
    filtGRF(:,i) = filtfilt(b,a,padGRF(:,9*(plate-1)+1+i)); % force
    filtGRF(:,i+3) = filtfilt(b,a,padGRF(:,9*(plate-1)+4+i)); % cop
    filtGRF(:,i+6) = filtfilt(b,a,padGRF(:,9*(plate-1)+7+i)); % moment
end

filtGRF = paddoff(filtGRF);

ti = find(nom_grf_f(:,1) == 0.056,1);
tf = find(nom_grf_f(:,1) == 0.2845,1);

h = (0.2845-0.056)/optimumOutput.options.N;
timeNodes = linspace(0.056,0.2845,optimumOutput.options.N+1);

for i = 1:optimumOutput.options.N
    for j = 1:d
        timeGrid_dummy(i,j) = timeNodes(i) + h*tau_root(j+1);
    end
end

timeGrid_dummy = timeGrid_dummy';
timeGrid_dummy = timeGrid_dummy(:);

timeGrid = [timeNodes(1); timeGrid_dummy];

intGRF = zeros(length(timeGrid),9);
for i = 1:3
    intGRF(:,i) = interp1(nom_grf_f(ti:tf,1),filtGRF(ti:tf,i),timeGrid); % force
    intGRF(:,i+3) = interp1(nom_grf_f(ti:tf,1),filtGRF(ti:tf,i+6),timeGrid); % moment
    intGRF(:,i+6) = interp1(nom_grf_f(ti:tf,1),filtGRF(ti:tf,i+3),timeGrid); % cop
end

% apply smoothing - remove negative vertical GRF & set to zeros
for i = 1:9
    if i == 2
        aux1 = intGRF(:,i) - (0.5+0.5.*(tanh(-intGRF(:,2)))).*intGRF(:,i);
    else
        intGRF(:,i) = intGRF(:,i) - (0.5+0.5.*(tanh(-intGRF(:,2)))).*intGRF(:,i);
    end
end
intGRF(:,2) = aux1;


intGRF(:,[4,6,8]) = 0;

% find the .dll to perform ID with model
FF = external('FF','C:\Users\Nicos\Desktop\Gil_twoSteps\ExternalFunctions\Running_ID.dll');

for i = 1:optimumOutput.options.N
   
    q_dummy((i-1)*3+1:i*3,:) = q((i-1)*3+1+i:i*3+i,:);
    qdot_dummy((i-1)*3+1:i*3,:) = qdot((i-1)*3+1+i:i*3+i,:);
    qddot_dummy((i-1)*3+1:i*3,:) = qddot((i-1)*3+1+i:i*3+i,:);
    
end
q_new = [q(1,:); q_dummy];
qdot_new = [qdot(1,:); qdot_dummy];
qddot_new = [qddot(1,:); qddot_dummy];

for i = 0:37-1
   states(:,i+1*(i+1)) = q_new(:,i+1);
   states(:,2*(i+1))   = qdot_new(:,i+1);
end

for i = 1:optimumOutput.options.N*3+1
    
    torques_ID(i,:) = full(evalf(FF(MX([states(i,:) qddot_new(i,:) intGRF(i,:)]'))));
    
end

torques_ID = torques_ID';


