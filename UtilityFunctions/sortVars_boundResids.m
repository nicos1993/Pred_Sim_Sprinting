function [outVars_sc,outVars_nsc] = sortVars(w0,n,scaling)

% outVars_sc: scaled output variables structure
% outVars_nsc: non-scaled output variables structure

% w0: column vector of variables
% n:  number of mesh intervals
% scaling: structure with scaling factors 

nq       = 37;
nMuscle  = 92;
nGRF     = 18;
nArms    = 14;
%nPelvis  = 6;

d = 3;

x        = zeros(nq*2,(d+1)*n+1);
act      = zeros(nMuscle,(d+1)*n+1);
FTtilde  = zeros(nMuscle,(d+1)*n+1);

x(1:nq*2,1)           = w0(1:nq*2);
act(1:nMuscle,1)      = w0(nq*2+1:nq*2+nMuscle);
FTtilde(1:nMuscle,1)  = w0(nq*2+nMuscle+1:nq*2+nMuscle*2);

dFTtilde = zeros(nMuscle,n);
uArms    = zeros(nArms,n);
%uResids  = zeros(nPelvis,n);
uActdot  = zeros(nMuscle,n);
uAcc     = zeros(nq,n);
uGRF     = zeros(nGRF,n);

for i = 0:n-1
    dFTtilde(:,i+1) = w0((i+1)*(nq*2+nMuscle*2)+i*(nMuscle+nArms+nMuscle+nq+nGRF+d*(nq*2+nMuscle*2))+1:...
        (i+1)*(nq*2+nMuscle*2)+i*(nMuscle+nArms+nMuscle+nq+nGRF+d*(nq*2+nMuscle*2))+nMuscle);
    
    uArms(:,i+1)    = w0((i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle)+i*(nArms+nMuscle+nq+nGRF+d*(nq*2+nMuscle*2))+1:...
        (i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle)+i*(nArms+nMuscle+nq+nGRF+d*(nq*2+nMuscle*2))+nArms);
    
    %uResids(:,i+1)    = w0((i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms)+i*(nPelvis+nMuscle+nq+nGRF+d*(nq*2+nMuscle*2))+1:...
     %   (i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms)+i*(nPelvis+nMuscle+nq+nGRF+d*(nq*2+nMuscle*2)));
    
    uActdot(:,i+1)    = w0((i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms)+i*(nMuscle+nq+nGRF+d*(nq*2+nMuscle*2))+1:...
        (i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms)+i*(nMuscle+nq+nGRF+d*(nq*2+nMuscle*2))+nMuscle);
    
    uAcc(:,i+1)       = w0((i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms+nMuscle)+i*(nq+nGRF+d*(nq*2+nMuscle*2))+1:...
        (i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms+nMuscle)+i*(nq+nGRF+d*(nq*2+nMuscle*2))+nq);
    
    uGRF(:,i+1)       = w0((i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms+nMuscle+nq)+i*(nGRF+d*(nq*2+nMuscle*2))+1:...
        (i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms+nMuscle+nq)+i*(nGRF+d*(nq*2+nMuscle*2))+nGRF);
    
    for r = 1:d
        x(:,r+1+i*(d+1)) = w0((i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms+nMuscle+nq+nGRF)+i*(d*(nq*2+nMuscle*2))+(r-1)*(nq*2)+1:...
            (i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms+nMuscle+nq+nGRF)+i*(d*(nq*2+nMuscle*2))+(r-1)*(nq*2)+nq*2);
    
        act(:,r+1+i*(d+1)) = w0((i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms+nMuscle+nq+nGRF)+i*(d*(nq*2+nMuscle*2))+(r-1)*(nMuscle)+d*(nq*2)+1:...
            (i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms+nMuscle+nq+nGRF)+i*(d*(nq*2+nMuscle*2))+(r-1)*(nMuscle)+d*(nq*2)+nMuscle);

        FTtilde(:,r+1+i*(d+1)) = w0((i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms+nMuscle+nq+nGRF)+i*(d*(nq*2+nMuscle*2))+(r-1)*(nMuscle)+d*(nq*2+nMuscle)+1:...
            (i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms+nMuscle+nq+nGRF)+i*(d*(nq*2+nMuscle*2))+(r-1)*(nMuscle)+d*(nq*2+nMuscle)+nMuscle);
    end

    x(:,(i+1)*(r+1)+1)       = w0((i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms+nMuscle+nq+nGRF)+(i+1)*((nq*2+nMuscle*2)*d)+1:...
        (i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms+nMuscle+nq+nGRF)+(i+1)*((nq*2+nMuscle*2)*d)+nq*2);
    
    act(:,(i+1)*(r+1)+1)     = w0((i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms+nMuscle+nq+nGRF)+(i+1)*((nq*2+nMuscle*2)*d)+nq*2+1:...
        (i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms+nMuscle+nq+nGRF)+(i+1)*((nq*2+nMuscle*2)*d)+nq*2+nMuscle);
    
    FTtilde(:,(i+1)*(r+1)+1) = w0((i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms+nMuscle+nq+nGRF)+(i+1)*((nq*2+nMuscle*2)*d)+nq*2+nMuscle+1:...
        (i+1)*(nq*2+nMuscle*2)+(i+1)*(nMuscle+nArms+nMuscle+nq+nGRF)+(i+1)*((nq*2+nMuscle*2)*d)+nq*2+nMuscle+nMuscle); 
end

contPrms = zeros(20,1);
contPrms = w0(end-19:end);

outVars_sc.q         = x(1:nq,:)';
outVars_sc.qdot      = x(nq+1:end,:)';
outVars_sc.act       = act;
outVars_sc.FTtilde   = FTtilde;
outVars_sc.dFTtilde  = dFTtilde;
outVars_sc.uTarms    = uArms;
%outVars_sc.uTresids  = uResids;
outVars_sc.uActdot   = uActdot;
outVars_sc.uAcc      = uAcc';
outVars_sc.uGRF      = uGRF;
outVars_sc.contPrms  = contPrms';

outVars_nsc.q        = outVars_sc.q.*scaling.q;
outVars_nsc.qdot     = outVars_sc.qdot.*scaling.qdot;
outVars_nsc.act      = outVars_sc.act.*scaling.act;
outVars_nsc.FTtilde  = outVars_sc.FTtilde.*scaling.FTtilde;
outVars_nsc.dFTtilde = outVars_sc.dFTtilde.*scaling.dFTtilde;
outVars_nsc.uTarms   = outVars_sc.uTarms.*scaling.uTarms;
%outVars_nsc.uTresids = outVars_sc.uTresids.*scaling.uTresids;
outVars_nsc.uActdot  = outVars_sc.uActdot.*scaling.uActdot;
outVars_nsc.uAcc     = outVars_sc.uAcc.*scaling.uAcc;
outVars_nsc.uGRF     = outVars_sc.uGRF.*scaling.uGRF;
outVars_nsc.contPrms = outVars_sc.contPrms.*scaling.contPrms';

    

