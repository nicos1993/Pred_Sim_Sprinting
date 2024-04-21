% Resample Data
% Tim Dorn
% 20/7/2006

% Use this to resample a set of data into a certain number
% of frames...
% ---------------------------------------------------------

function [sampled,coeff] = resamp3(raw, tSamp,outputt)
if nargin==2
    outputt=1;
end
[m,n] = size(raw);
sampled= zeros(m,length(tSamp));
%sampled(1,:) = tSamp;
%for i = 2:m
%    coeff(i-1)=pchip(raw(1,:),raw(i,:));
%    sampled(i,:) = ppval(coeff(i-1), tSamp);
%end
    coeff=pchip(raw(1,:),raw(2:end,:));
    sampled = ppval(coeff, tSamp);
    if outputt==1
        sampled=[tSamp; sampled]';
    end



