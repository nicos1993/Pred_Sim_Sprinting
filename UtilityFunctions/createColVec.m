function vecVars_nsc = createColVec(outVars_nsc,n)

vecVars_nsc = [];

q=outVars_nsc.q;
qdot=outVars_nsc.qdot;
act=outVars_nsc.act;
FTtilde=outVars_nsc.FTtilde;
dFTtilde=outVars_nsc.dFTtilde;
uTarms=outVars_nsc.uTarms;
uTresids=outVars_nsc.uTresids;
uActdot=outVars_nsc.uActdot;
uAcc=outVars_nsc.uAcc;
uGRF=outVars_nsc.uGRF;
contPrms=outVars_nsc.contPrms;

d=3;

vecVars_nsc = [vecVars_nsc; q(1,:) qdot(1,:)];

vecVars_nsc = [vecVars_nsc act(:,1)'];

vecVars_nsc = [vecVars_nsc FTtilde(:,1)'];

for k = 0:n-1

    vecVars_nsc = [vecVars_nsc dFTtilde(:,k+1)'];
    vecVars_nsc = [vecVars_nsc uTarms(:,k+1)'];
    vecVars_nsc = [vecVars_nsc uTresids(:,k+1)'];
    vecVars_nsc = [vecVars_nsc uActdot(:,k+1)'];
    vecVars_nsc = [vecVars_nsc uAcc(k+1,:)];
    vecVars_nsc = [vecVars_nsc uGRF(:,k+1)'];
    
    for j=1:d
        vecVars_nsc = [vecVars_nsc q(k*(d+1)+1+j,:) qdot(k*(d+1)+1+j,:)];
    end
    
    for j=1:d
        vecVars_nsc = [vecVars_nsc act(:,k*(d+1)+1+j)'];
    end
    
    for j=1:d
        vecVars_nsc = [vecVars_nsc FTtilde(:,k*(d+1)+1+j)'];
    end
    
    vecVars_nsc = [vecVars_nsc q((k+1)*(d+1)+1,:) qdot((k+1)*(d+1)+1,:)];
    vecVars_nsc = [vecVars_nsc act(:,(k+1)*(d+1)+1)'];
    vecVars_nsc = [vecVars_nsc FTtilde(:,(k+1)*(d+1)+1)'];

    
end

vecVars_nsc = [vecVars_nsc contPrms];

end
