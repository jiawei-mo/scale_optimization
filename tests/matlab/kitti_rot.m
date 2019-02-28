dir = '/home/jiawei/Dropbox/IROS/eval/kitti';
gts_dir = strcat(dir, '/poses');
vos_dir = strcat(dir, '/results');

out_dir = strcat(dir, '/output');
system(['rm -rf ', out_dir]);
mkdir(out_dir)
out_vos_dir = strcat(out_dir, '/results');
mkdir(out_vos_dir)
out_formatSpec = '%f %f %f %f %f %f %f %f %f %f %f %f\n';

for i=0:10
    gt_file = strcat(gts_dir, sprintf('/%02d.txt',i));
    vo_file = strcat(vos_dir, sprintf('/%02d.txt',i));
    gt = load(gt_file);
    vo = load(vo_file);
    vo = vo(:,[1:4,8,5:7]);
    
    R = align(gt(:,[4,8,12]), vo(:,2:4));
    vo = tum2kitti(vo, R);
    
    vo(:,[4,8,12]) = vo(:,[4,8,12])-vo(1,[4,8,12]);

    out_vos_file = fopen(strcat(out_vos_dir, sprintf('/%02d.txt',i)), 'w');
    fprintf(out_vos_file, out_formatSpec, vo');
    fclose(out_vos_file);
end


function D_K = tum2kitti(D, R)
D_K = zeros(length(D), 12);
for i=1:length(D)
    T = R*[quat2rotm(D(i,5:8)), D(i,2:4)'];
    T = T';
    D_K(i, :)  = T(:);
end
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