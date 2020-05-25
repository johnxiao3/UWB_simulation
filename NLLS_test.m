%% plot the env
syms x; syms y;

xo = [0.15,-0.15,0];
yo = [0.0,0.0,0.15];
disRR12 = sqrt((xo(1)-xo(2))^2+(yo(1)^2-yo(2))^2);
disRR13 = sqrt((xo(1)-xo(3))^2+(yo(1)^2-yo(3))^2);
disRR23 = sqrt((xo(3)-xo(2))^2+(yo(3)^2-yo(2))^2);

No = size(xo,2);
xx = 10; yy = 10;
%rescan
clc
% for xx = 1:49
%     for yy = 1:49
disp([num2str(xx) ',' num2str(yy)])
% hold off;
for i = 1:No
    d(i) = norm([xo(i) yo(i)]-[xx yy]);    
%     fimplicit(@(x,y) (x-xo(i)).^2+(y-yo(i)).^2-d(i).^2,'b');hold on;
end
% plot
% hold off;
% plot(xo,yo,'bo','MarkerSize',8,'LineWidth',2);hold on;axis equal;
% xlim([-10 60]);ylim([-10 60])
% plot(xx,yy,'ro','MarkerSize',8,'LineWidth',2);

% give random err
mean_err = [];
X_m = [];
Y_m = [];
X_m0 = [];
Y_m0 = [];
delta_X1 = [];
delta_Y1 = [];
delta_X2 = [];
delta_Y2 = [];
count = 0;
while(count<2000)
    count = count+1;
    for i=1:No
        rand_err(i) = normrnd(0,0.018);
        d_m(i) = d(i)+rand_err(i);
    end
    
    %  method 1 LLS
    gamma1 = d_m(2)^2-d_m(1)^2-(xo(2)^2-xo(1)^2+yo(2)^2-yo(1)^2);
    gamma2 = d_m(3)^2-d_m(1)^2-(xo(3)^2-xo(1)^2+yo(3)^2-yo(1)^2);
    A = 2*[xo(1)-xo(2) yo(1)-yo(2)
       xo(1)-xo(3) yo(1)-yo(3)];
    tag_m = inv(A)*[gamma1;gamma2];
    x_m0 = tag_m(1);
    y_m0 = tag_m(2);
    
    delta_X1 = [delta_X0;x_m0-xx];
    delta_Y1 = [delta_Y0;y_m0-yy];
    
    %  method 2 NLLS
    R = [x_m0;y_m0]
    for n = 1:6
        f1 = sqrt((R(1)-xo(1))^2+(R(2)-yo(1))^2)-d_m(1);
        f2 = sqrt((R(1)-xo(2))^2+(R(2)-yo(2))^2)-d_m(2);
        f3 = sqrt((R(1)-xo(3))^2+(R(2)-yo(3))^2)-d_m(3);
        J11 = (R(1)-xo(1))^2/(f1+d_m(1))^2 + ...
                (R(1)-xo(2))^2/(f2+d_m(2))^2+ ...
                (R(1)-xo(3))^2/(f3+d_m(3))^2;
        J12 = (R(1)-xo(1))*(R(2)-yo(1))/(f1+d_m(1))^2 + ...
                (R(1)-xo(2))*(R(2)-yo(2))/(f2+d_m(2))^2+ ...
                (R(1)-xo(3))*(R(2)-yo(3))/(f3+d_m(3))^2;
        J22 = (R(2)-yo(1))^2/(f1+d_m(1))^2 + ...
                (R(2)-yo(2))^2/(f2+d_m(2))^2+ ...
                (R(2)-yo(3))^2/(f3+d_m(3))^2;
        JTJ = [J11 J12;
                J12 J22];
        JTf1 = (R(1)-xo(1))*f1/(f1+d_m(1))+ ...
                (R(1)-xo(2))*f2/(f2+d_m(2))+ ...
                (R(1)-xo(3))*f3/(f3+d_m(3));
        JTf2 = (R(2)-yo(1))*f1/(f1+d_m(1))+ ...
                (R(2)-yo(2))*f2/(f2+d_m(2))+ ...
                (R(2)-yo(3))*f3/(f3+d_m(3));
        JTf = [JTf1
                JTf2];
        R = R - inv(JTJ)*JTf
    end
    delta_X2 = [delta_X2;R(1)-xx];
    delta_Y2 = [delta_Y2;R(2)-yy];
    %
   

cla;
for i = 1:No
    fimplicit(@(x,y) (x-xo(i)).^2+(y-yo(i)).^2-d_m(i).^2,'b');hold on;
end
axis equal;
title(['likelyhood','(',num2str(xx),',',num2str(yy),')'])
plot(delta_X1(end)+xx,delta_Y1(end)+yy,'.k','MarkerSize',25);
plot(delta_X2(end)+xx,delta_Y2(end)+yy,'.r','MarkerSize',25);

% plot(i1x,i1y,'.c','MarkerSize',15);
% plot(i2x,i2y,'.c','MarkerSize',15);
plot(xo(mincircle)+d_m(mincircle)*cos(phi(1)),...
     yo(mincircle)+d_m(mincircle)*sin(phi(1)),'v','MarkerSize',10);
plot(xo(mincircle)+d_m(mincircle)*cos(phi(2)),...
     yo(mincircle)+d_m(mincircle)*sin(phi(2)),'>','MarkerSize',10);
plot(xo(mincircle)+d_m(mincircle)*cos(phi(3)),...
     yo(mincircle)+d_m(mincircle)*sin(phi(3)),'^','MarkerSize',10);
plot(xo(mincircle)+d_m(mincircle)*cos(phi(4)),...
     yo(mincircle)+d_m(mincircle)*sin(phi(4)),'<','MarkerSize',10);
 
fimplicit(@(x,y) (x-xx).^2+(y-yy).^2-0.8.^2,'r');hold on;
% xlim([xx-5 xx+5]);ylim([yy-5 yy+5]);
xlim([-18 18]);ylim([-18 18]);
drawnow;
1
    %disp(num2str(count))
end

