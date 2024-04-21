% Function to extract:
%   Maximum Isometric Force
%   Optimum Fibre Length
%   Tendon Slack Length
%   Optimal Pennation Angle
%   Maximum Contract Velocity - x10 Optimum Fibre Length
% From .osim file

function muscProperties = extractMuscProperties(modelName,mNames)

import org.opensim.modeling.*

model = Model(modelName);
model.initSystem();

muscles = model.getMuscles();
nMuscles = muscles.getSize()/2;

muscProperties = zeros(5,nMuscles+3);

for i = 1:nMuscles
    muscle = muscles.get(mNames(i));
    f      = muscle.getMaxIsometricForce();
    l      = muscle.getOptimalFiberLength();
    t      = muscle.getTendonSlackLength();
    p      = muscle.getPennationAngleAtOptimalFiberLength();
    %m      = muscle.getMaxContractionVelocity();
    muscProperties(1,i) = f;
    muscProperties(2,i) = l;
    muscProperties(3,i) = t;
    muscProperties(4,i) = p;
    muscProperties(5,i) = l*10;
end

muscProperties(:,end-2) = muscProperties(:,end-5);
muscProperties(:,end-1) = muscProperties(:,end-4);
muscProperties(:,end)   = muscProperties(:,end-3);
end
