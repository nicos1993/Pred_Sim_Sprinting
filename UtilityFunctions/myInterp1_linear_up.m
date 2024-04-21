function yint = myInterp1_linear_up(x,y,X)
import casadi.*

if isa(y,'double')
    yint = zeros(length(X),1);
else
    yint = MX(length(X),1);
end

for i = 1:length(X)
    if i == length(X)
        q = find(x == X(i),1);
    else
        q = find(X(i) < x,1);
    end

    if q == 1
        q = 2;
    end
    
    if isempty(q)
        q = length(x);
    end
    
    x0 = x(q-1);
    x1 = x(q);
    y0 = y(q-1);
    y1 = y(q);
    yint(i,1) = y0 + (X(i)-x0)*((y1-y0)/(x1-x0));
    
end