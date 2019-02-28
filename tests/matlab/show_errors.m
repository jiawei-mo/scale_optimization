scale_dir = '/home/jiawei/Dropbox/IROS/eval/kitti';
stereo_dir = '/home/jiawei/Dropbox/IROS/eval/kitti_stereo';
test_start = 0;
test_end = 10;
% 
% scale_dir = '/home/jiawei/Dropbox/IROS/eval/euroc/mh';
% stereo_dir = '/home/jiawei/Dropbox/IROS/eval/euroc_stereo/mh';
% test_start = 1;
% test_end = 5;

% scale_dir = '/home/jiawei/Dropbox/IROS/eval/euroc/vr';
% stereo_dir = '/home/jiawei/Dropbox/IROS/eval/euroc_stereo/vr';
% test_start = 1;
% test_end = 6;

[scale_tl_vec, scale_rl_vec, scale_s_vec, scale_ba_vec, scale_p_vec, scale_fps_vec] = error_cal(scale_dir, test_start, test_end);
[stereo_tl_vec, stereo_rl_vec, stereo_s_vec, stereo_ba_vec, stereo_p_vec, stereo_fps_vec] = error_cal(stereo_dir, test_start, test_end);
tl = [scale_tl_vec, stereo_tl_vec];
rl = [scale_rl_vec, stereo_rl_vec];
scale_stereo = [scale_s_vec, stereo_s_vec];
BA = [scale_ba_vec, stereo_ba_vec];
fps = [scale_fps_vec, stereo_fps_vec];
pts = [scale_p_vec, stereo_p_vec];

error_table = table(tl, rl, scale_stereo, BA, pts, fps)

function [tl_vec, rl_vec, s_vec, ba_vec, p_vec, fps_vec] = error_cal(dir, test_start, test_end)
errors_dir = strcat(dir, '/output/results/errors');
ba_dir = strcat(dir, '/results/ba_times');
s_dir = strcat(dir, '/results/s_times');
fps_dir = strcat(dir, '/results/fps_times');

tl_vec = zeros((test_end-test_start+1),1);
rl_vec = zeros((test_end-test_start+1),1);
s_vec = zeros((test_end-test_start+1),1);
ba_vec = zeros((test_end-test_start+1),1);
fps_vec = zeros((test_end-test_start+1),1);
p_vec = zeros((test_end-test_start+1),1);
for i=test_start:test_end
    error_file = strcat(errors_dir, sprintf('/%02d.txt',i));
    err = load(error_file);
    if(size(err,1)<2) 
        continue;
    end
    tl_vec(i-test_start+1) = 100*mean(err(:,3));
    rl_vec(i-test_start+1) = 100*57.3*mean(err(:,2));
    
    ba_file = strcat(ba_dir, sprintf('/%02d.txt',i));
    s_file = strcat(s_dir, sprintf('/%02d.txt',i));
    fps_file = strcat(fps_dir, sprintf('/%02d.txt',i));
    ba_t = load(ba_file);
    fps_t = load(fps_file);
    s_t = load(s_file);
    s_vec(i-test_start+1) = 1000*mean(s_t);
    ba_vec(i-test_start+1) = 1000*mean(ba_t(:,2));
    fps_vec(i-test_start+1) = 1000*mean(fps_t);
    p_vec(i-test_start+1) = mean(ba_t(:,1));
%     fprintf('Stereo Mean: %.2f ms\n', 1000*mean(s_t));
%     fprintf('BA Mean: %.2f ms\n', 1000*mean(ba_t(:,2)));
%     fprintf('Pts Mean: %.2f\n', mean(ba_t(:,1)));
end
end