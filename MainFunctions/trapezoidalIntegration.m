function [summed,pos_summed,neg_summed,test] = trapezoidalIntegration(time,variable)

size_time = size(time);
size_variable = size(variable);

if size_time(2) > 1
    time = time';
end

if size_variable(2) > 1
    variable = variable';
end

ind_parts = zeros(length(time)-1,1);

for i = 1:length(ind_parts)
    ind_parts(i) = (variable(i+1,1)+variable(i,1))*0.5*(time(i+1,1)-time(i,1));
end

summed = sum(ind_parts);
pos_summed = sum(ind_parts(find(ind_parts > 0)));
neg_summed = sum(ind_parts(find(ind_parts < 0)));

test = pos_summed+neg_summed;

end