function [gtp, vop, gtt, vot, gtvn, vovn, voo] = alignGT(gt, vo)
% get overlap of gt with vo
i = 1; % index of gt
j = 1; % index of vo
k = 1; % index of vo
while(i<=size(gt,1) && gt(i,1) < vo(j,1))
    i = i+1;
end
while i<=size(gt,1)
    while(j<=size(vo,1) && gt(i,1) > vo(j,1))
        j = j+1;
    end
    if(j>size(vo,1))
        break;
    else
        vo_n(k,1) = gt(i,1);
        if(gt(i,1) ~= vo(j,1))
            inc = (vo(j,2:4) - vo(j-1,2:4))/(vo(j,1) - vo(j-1,1));
            vo_n(k,2:4) = vo(j-1,2:4) + (gt(i,1) - vo(j-1,1))*inc;
        else
            vo_n(k,2:4) = vo(j,2:4);
        end
    end
    i = i+1;
    k = k+1;
end
% use range of idx
idx = 1:size(vo_n,1);
% idx = [size(gt,1):-1:size(gt,1)-200];
vo = vo_n(idx, :);
gt = gt(idx, :);

%% align two set of points
% cg = mean(gt(:,2:4));
% cv = mean(vo(:,2:4));
cg = gt(1,2:4);
cv = vo(1,2:4);
H = zeros(3,3);
ali_length = length(gt);
% ali_length = 1;
% while gt(ali_length,1)-gt(1,1)<5
%     ali_length = ali_length + 1;
% end
for i=1:ali_length
    H = H + (gt(i,2:4)-cg)'*(vo(i,2:4)-cv);
end
[U,~,V] = svd(H);
R = U*V';
for i=1:size(vo,1)
    p = R*vo(i,2:4)';
    vo(i,2:4) = p';
end
% cg = mean(gt(:,2:4));
% cv = mean(vo(:,2:4));
cg = gt(1,2:4);
cv = vo(1,2:4);
for i=size(vo,1):-1:1
    vo(i,2:4) = vo(i,2:4) - cv;
end
for i=size(gt,1):-1:1
    gt(i,2:4) = gt(i,2:4) - cg;
end

%% results
step = 1; 
step = floor(length(vo) / (vo(end,1)-vo(1,1))); % step of 1 sec

% calculate vo translation
for i=1+step:size(vo,1)
    vo(i, 5:7) = (vo(i, 2:4) - vo(i-step, 2:4)) / (vo(i, 1) - vo(i-step, 1));
end
vo = vo(1+step:end, :);

% calculate gt translation
for i=1+step:size(gt,1)
    gt(i, 5:7) = (gt(i, 2:4) - gt(i-step, 2:4)) / (gt(i, 1) - gt(i-step, 1));
end
gt = gt(1+step:end, :);
 
% time axis
gtt = gt(:,1)-gt(1,1);
vot = vo(:,1)-vo(1,1);

% position
gtp = gt(:,2:4);
vop = vo(:,2:4);

% velocity scale
gtvn = vecnorm(gt(:,5:7)')';
vovn = vecnorm(vo(:,5:7)')';

% velocity orientation
gtvd = gt(:,5:7) ./ gtvn;
vovd = vo(:,5:7) ./ vovn;
voo = vot;
for i=2:size(vo,1)
    voo(i) = acos( gtvd(i,:) * vovd(i,:)')/3.14159*180;
end

end
