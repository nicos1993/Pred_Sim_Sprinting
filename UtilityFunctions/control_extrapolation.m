function D_control = control_extrapolation(tau_roots)

%tau_roots = [0.333 0.6666 0.999];

D_control = zeros(3,1);

lag_poly = 1;

tau = 0; % Get coefficients of polynomial at this point.

for j = 1:3
    
    for r = 1:3
        
        if r~= j
            
            lag_poly = ((tau-tau_roots(r))/(tau_roots(j)-tau_roots(r))).*lag_poly; 
            
        end
        
    end 
    
    D_control(j) = lag_poly;
    
    lag_poly = 1;
    
end

%control_vec = [5 10 15];

%extra_control = control_vec*D_control;