% Assumes an optimum file has already been loaded

q = optimumOutput.optVars_nsc.q;
qdot = optimumOutput.optVars_nsc.qdot;
uAcc = optimumOutput.optVars_nsc.uAcc;

FTtilde = optimumOutput.optVars_nsc.FTtilde;
dFTtilde = optimumOutput.optVars_nsc.dFTtilde;
armActs = optimumOutput.optVars_nsc.armActs;
armExcts = optimumOutput.optVars_nsc.armExcts;
act = optimumOutput.optVars_nsc.act;
uActdot = optimumOutput.optVars_nsc.uActdot;
uReserves = optimumOutput.optVars_nsc.uReserves;

q_pelvis_check = abs(q(1:6,1)) - abs(q(1:6,end));
q_legs_check_1 = abs(q(7:13,1)) - abs(q(14:20,end));
q_legs_check_2 = abs(q(14:20,1)) - abs(q(7:13,end));
q_torso_check = abs(q(21:23,1)) - abs(q(21:23,end));
q_arms_check_1 = abs(q(24:30,1)) - abs(q(31:37,end));
q_arms_check_2 = abs(q(24:30,end)) - abs(q(31:37,1));

qdot_pelvis_check = abs(qdot(1:6,1)) - abs(qdot(1:6,end));
qdot_legs_check_1 = abs(qdot(7:13,1)) - abs(qdot(14:20,end));
qdot_legs_check_2 = abs(qdot(14:20,1)) - abs(qdot(7:13,end));
qdot_torso_check = abs(qdot(21:23,1)) - abs(qdot(21:23,end));
qdot_arms_check_1 = abs(qdot(24:30,1)) - abs(qdot(31:37,end));
qdot_arms_check_2 = abs(qdot(24:30,end)) - abs(qdot(31:37,1));

uAcc_pelvis_check = abs(uAcc(1:6,1)) - abs(uAcc(1:6,end));
uAcc_legs_check_1 = abs(uAcc(7:13,1)) - abs(uAcc(14:20,end));
uAcc_legs_check_2 = abs(uAcc(14:20,1)) - abs(uAcc(7:13,end));
uAcc_torso_check = abs(uAcc(21:23,1)) - abs(uAcc(21:23,end));
uAcc_arms_check_1 = abs(uAcc(24:30,1)) - abs(uAcc(31:37,end));
uAcc_arms_check_2 = abs(uAcc(24:30,end)) - abs(uAcc(31:37,1));


FTtilde_check_1 = FTtilde(1:46,1)-FTtilde(47:end,end);
FTtilde_check_2 = FTtilde(1:46,end)-FTtilde(47:end,1);

dFTtilde_check_1 = dFTtilde(1:46,1)-dFTtilde(47:end,end);
dFTtilde_check_2 = dFTtilde(1:46,end)-dFTtilde(47:end,1);

armActs_check_1 = armActs(1:7,1)-armActs(8:14,end);
armActs_check_2 = armActs(1:7,end)-armActs(8:14,1);

armExcts_check_1 = armExcts(1:7,1)-armExcts(8:14,end);
armExcts_check_2 = armExcts(1:7,end)-armExcts(8:14,1);

act_check_1 = act(1:46,1)-act(47:end,end);
act_check_2 = act(1:46,end)-act(47:end,1);

uActdot_check_1 = uActdot(1:46,1)-uActdot(47:end,end);
uActdot_check_2 = uActdot(1:46,end)-uActdot(47:end,1);

uReserves_check_1 = uReserves(1,1)-uReserves(2,end);
uReserves_check_2 = uReserves(1,end)-uReserves(2,1);

