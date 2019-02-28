% gt_no_rot = true;
% dir = '/home/jiawei/Dropbox/IROS/eval/euroc/mh';
% tests = 5;
% 
gt_no_rot = false;
dir = '/home/jiawei/Dropbox/IROS/eval/euroc/vr';
tests = 6;

% gt_no_rot = true;
% dir = '/home/jiawei/Dropbox/IROS/eval/euroc_stereo/mh';
% tests = 5;

% gt_no_rot = false;
% dir = '/home/jiawei/Dropbox/IROS/eval/euroc_stereo/vr';
% tests = 6;

gts_p_dir = strcat(dir, '/poses');
vos_dir = strcat(dir, '/results/poses');
load('T_euroc.mat');

out_dir = strcat(dir, '/output');
system(['rm -rf ', out_dir]);
mkdir(out_dir)
out_gts_dir = strcat(out_dir, '/ground_truths');
mkdir(out_gts_dir)
out_vos_dir = strcat(out_dir, '/results');
mkdir(out_vos_dir)
out_formatSpec = '%f %f %f %f %f %f %f %f %f %f %f %f %f\n';

for i=01:tests
    gt_file = strcat(gts_p_dir, sprintf('/%02d.csv',i));
    vo_file = strcat(vos_dir, sprintf('/%02d.txt',i));
    gt_raw = csvread(gt_file);
    gt_raw(:,1) = gt_raw(:,1) / 1e9;
    vo_raw_ = load(vo_file);
    if(length(vo_raw_)<2) 
        vo_raw = gt_raw;
    else
        vo_raw = vo_raw_(:,[1:4,8,5:7]);
    end
    
    rsi = 1; 
    while(vo_raw(rsi,1)<gt_raw(1,1))
        rsi = rsi+1;
    end
    rei = size(vo_raw,1); 
    while(vo_raw(rei,1)>gt_raw(end,1))
        rei = rei-1;
    end
    vo = vo_raw(rsi:rei, :);
    
    gt = zeros(size(vo));
    ri=1;
    for gi=2:size(gt_raw,1)
        if(ri>size(vo,1))
            break;
        end
        if(gt_raw(gi,1)<vo(ri,1)) 
           continue;
        end

        f = (vo(ri,1)-gt_raw(gi-1,1))/(gt_raw(gi,1)-gt_raw(gi-1,1));
        g_p = (1-f)*gt_raw(gi-1,2:4) + f*gt_raw(gi,2:4);
        g_q = slerp(quaternion(gt_raw(gi-1,5:8)), quaternion(gt_raw(gi,5:8)), f);
        gt(ri,1) = vo(ri,1);
        gt(ri,2:4) = g_p;
        [w,x,y,z] = parts(g_q);
        gt(ri,5:8) = [w,x,y,z];
        ri = ri+1;
    end
    
    R = align(gt(:,2:4), vo(:,2:4));
    
    gt = tum2kitti(gt, eye(3), eye(4));
    vo = tum2kitti(vo, R, inv(T_b));
    
    if(gt_no_rot)
        vo(:,[2:4,6:8,10:12]) = gt(:,[2:4,6:8,10:12]);
    end
    
    gt(:,[5,9,13]) = gt(:,[5,9,13])-mean(gt(:,[5,9,13]));
    vo(:,[5,9,13]) = vo(:,[5,9,13])-mean(vo(:,[5,9,13]));
    
    out_gt_file = fopen(strcat(out_gts_dir, sprintf('/%02d.txt',i)), 'w');
    out_vos_file = fopen(strcat(out_vos_dir, sprintf('/%02d.txt',i)), 'w');
    fprintf(out_gt_file, out_formatSpec, gt');
    fprintf(out_vos_file, out_formatSpec, vo');
    fclose(out_gt_file);
    fclose(out_vos_file);
end


function D_K = tum2kitti(D, R, T_b)
D_K = zeros(length(D), 12);
for i=1:length(D)
    T = R*[quat2rotm(D(i,5:8)), D(i,2:4)']*T_b;
    T = T';
    D_K(i, :)  = T(:);
end
D_K = [D(:,1), D_K];
end



function R = align(gt_p, vo_p)
cg = mean(gt_p);
cv = mean(vo_p);
H = zeros(3,3);
for i=1:length(gt_p)
    H = H + (gt_p(i,:)-cg)'*(vo_p(i,:)-cv);
end
[U,~,V] = svd(H);
R = U*V';
end