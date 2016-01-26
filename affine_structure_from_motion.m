
% get matrix size
[hmat wmat] = size(Xs); 

% mean of each row
for ycounter = 1:hmat
    
    Xs(ycounter,:) = Xs(ycounter,:) - mean(Xs(ycounter,:));
    Ys(ycounter,:) = Ys(ycounter,:) - mean(Ys(ycounter,:));

end


W = [Xs; Ys];

[U D V] = svd(W);

mht = U(:, 1:3)*sqrt(D(1:3, 1:3)); 
sht = sqrt(D(1:3, 1:3))*V(:, 1:3)';

%% Calculate Q... the metric constraints
Is = mht(1:hmat, :);
Js = mht(hmat+1:end, :);


gfun = @(a, b)[ a(1)*b(1), a(1)*b(2)+a(2)*b(1), a(1)*b(3)+a(3)*b(1), a(2)*b(2), a(2)*b(3)+a(3)*b(2), a(3)*b(3)] ;

% Calculate G          
G = zeros(3*hmat, 6);
for f = 1:3*hmat
    if f <= hmat
        G(f, :) = gfun(Is(f,:), Is(f,:));
    elseif f <= 2*hmat
        G(f, :) = gfun(Js(mod(f, hmat+1)+1, :), Js(mod(f, hmat+1)+1, :));
    else
        G(f, :) = gfun(Is(mod(f, 2*hmat),:), Js(mod(f, 2*hmat),:));
    end
end

c = [ones(2*hmat, 1); zeros(hmat, 1)];

% solve Gl by SVD
[U S V] = svd(G);
hatl = U'*c;
y = [hatl(1)/S(1,1); hatl(2)/S(2,2); hatl(3)/S(3,3); hatl(4)/S(4,4); hatl(5)/S(5,5); hatl(6)/S(6,6)];
l = V*y;

L = [l(1) l(2) l(3); l(2) l(4) l(5); l(3) l(5) l(6)];

% !!!!!!!!!!
Q = chol(L);
%!!!!!!!!!!!

M = mht*Q;
S = Q\sht;

%IMAGE_DIR = 'C:/MYWORKSPACE/HomeWorks/ComputerVision/HW3/data/images/';
%DATA_DIR = 'data/';   

%imgPath1 = sprintf('%s%s', IMAGE_DIR, 'hotel.seq0.png');
%im = im2double(imread(imgPath1));
    

%% plot of 3D points
figure;
plot3(sht(1, :), sht(2,:), sht(3,:), 'k.'); hold on;

plot3(S(1, :), S(2,:), S(3,:), 'b.');
plot3(0, 0, 0, 'gs');
grid on;
title('3D points from tracked points: affine ambiguity');
legend('before','after', 'origin');


cam_pos = zeros(hmat, 3);
for f = 1:hmat
    kf = cross(M(f,:), M(f+hmat, :));
    cam_pos(f,:) = kf/norm(kf); % in unit norm
    %disp(f);
    %imgPath = sprintf('%shotel.seq%d.png', IMAGE_DIR, f-1);
    %im = im2double(imread(imgPath));
    %imagesc(im);
    %plot(cam_pos(:, 1), cam_pos(:, 2));
end

cposX = cam_pos(:, 1);
cposY = cam_pos(:, 2);
cposZ = cam_pos(:, 3);

% camera 3d plot
%figure;
figure; plot3(cposX, cposY, cposZ,'.-');
suptitle('The camera position for each frame - 3D View');
grid on;

figure;
%subplot(131);
plot(cposX, cposY, '.-');
grid on;
title('XY axis');
xlabel('X');
ylabel('Y');
suptitle('The camera position for each frame - XY axis');

figure;
%subplot(132);
plot(cposX, cposZ,'.-');
grid on;
title('XZ axis');
xlabel('X');
ylabel('Z');
suptitle('The camera position for each frame - XZ axis');

figure;
%subplot(133);
plot(cposY, cposZ,'.-');
grid on;
title('YZ axis');
xlabel('Y');
ylabel('Z');
suptitle('The camera position for each frame - YZ axis');

%figure;
%plot(cposX);
%figure;
%plot(cposY);
%figure;
%plot(cposZ);


%% Other 3D plots...
%X = S(1, :);
%Y = S(2, :);
%Z = S(3, :);

%figure; 
%imagesc(im); colormap('gray'); hold on;
%plot(Y+320, X+250, 'y.');
%title('first frame overlayed with keypoints');

%figure; trimesh(tri, X,Y,Z);
%plot3(X, Y, Z);

