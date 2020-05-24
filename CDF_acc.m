

%% plot the env
syms x; syms y;

xo = [0.15,-0.15,0];
yo = [0.0,0.0,0.15];
No = size(xo,2);
%rescan
clc
% for xx = 1:49
%     for yy = 1:49
xx = 10; yy = 0;
disp([num2str(xx) ',' num2str(yy)])
subplot(121);
hold off;
for i = 1:No
    d(i) = norm([xo(i) yo(i)]-[xx yy]);    
    fimplicit(@(x,y) (x-xo(i)).^2+(y-yo(i)).^2-d(i).^2,'b');hold on;
end
hold on;
subplot(122)
hold off;
for i = 1:No
    d(i) = norm([xo(i) yo(i)]-[xx yy]);    
    fimplicit(@(x,y) (x-xo(i)).^2+(y-yo(i)).^2-d(i).^2,'b');hold on;
end
hold on;

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
delta_X = [];
delta_Y = [];
delta_X0 = [];
delta_Y0 = [];
count = 0;
while(count<2000)
    count = count+1;
    for i=1:No
        rand_err(i) = normrnd(0,0.018);
        d_m(i) = d(i)+rand_err(i);
    end
    if min(d_m) == d_m(1)
       [i1x i1y] = circcirc(xo(1),yo(1),d_m(1),xo(2),yo(2),d_m(2));
       if isnan(i1x(1))
           i1x(1) = xo(2)*(sqrt(xo(1)^2+yo(1)^2)+d_m(1))/sqrt(xo(2)^2+yo(2)^2);
           i1x(2) = xo(2)*(sqrt(xo(1)^2+yo(1)^2)-d_m(1))/sqrt(xo(2)^2+yo(2)^2);
           i1y(1) = yo(2)*(sqrt(xo(1)^2+yo(1)^2)+d_m(1))/sqrt(xo(2)^2+yo(2)^2);
           i1y(2) = yo(2)*(sqrt(xo(1)^2+yo(1)^2)-d_m(1))/sqrt(xo(2)^2+yo(2)^2);
       end
       [i2x i2y] = circcirc(xo(1),yo(1),d_m(1),xo(3),yo(3),d_m(3));
       if isnan(i2x(1))
           i2x(1) = xo(3)*(sqrt(xo(1)^2+yo(1)^2)+d_m(1))/sqrt(xo(3)^2+yo(3)^2);
           i2x(2) = xo(3)*(sqrt(xo(1)^2+yo(1)^2)-d_m(1))/sqrt(xo(3)^2+yo(3)^2);
           i2y(1) = yo(3)*(sqrt(xo(1)^2+yo(1)^2)+d_m(1))/sqrt(xo(3)^2+yo(3)^2);
           i2y(2) = yo(3)*(sqrt(xo(1)^2+yo(1)^2)-d_m(1))/sqrt(xo(3)^2+yo(3)^2);
       end
       p1 = d_m(2);
       p2 = d_m(3);
    elseif min(d_m) == d_m(2)
       [i1x i1y] = circcirc(xo(2),yo(2),d_m(2),xo(1),yo(1),d_m(1));
       if isnan(i1x(1))
           i1x(1) = xo(1)*(sqrt(xo(2)^2+yo(2)^2)+d_m(2))/sqrt(xo(1)^2+yo(1)^2);
           i1x(2) = xo(1)*(sqrt(xo(2)^2+yo(2)^2)-d_m(2))/sqrt(xo(1)^2+yo(1)^2);
           i1y(1) = yo(1)*(sqrt(xo(2)^2+yo(2)^2)+d_m(2))/sqrt(xo(1)^2+yo(1)^2);
           i1y(2) = yo(1)*(sqrt(xo(2)^2+yo(2)^2)-d_m(2))/sqrt(xo(1)^2+yo(1)^2);
       end
       [i2x i2y] = circcirc(xo(2),yo(2),d_m(2),xo(3),yo(3),d_m(3));
       if isnan(i2x(1))
           i2x(1) = xo(3)*(sqrt(xo(2)^2+yo(2)^2)+d_m(2))/sqrt(xo(3)^2+yo(3)^2);
           i2x(2) = xo(3)*(sqrt(xo(2)^2+yo(2)^2)-d_m(2))/sqrt(xo(3)^2+yo(3)^2);
           i2y(1) = yo(3)*(sqrt(xo(2)^2+yo(2)^2)+d_m(2))/sqrt(xo(3)^2+yo(3)^2);
           i2y(2) = yo(3)*(sqrt(xo(2)^2+yo(2)^2)-d_m(2))/sqrt(xo(3)^2+yo(3)^2);
       end
       p1 = d_m(1);
       p2 = d_m(3);
    elseif min(d_m) == d_m(3)
       [i1x i1y] = circcirc(xo(3),yo(3),d_m(3),xo(2),yo(2),d_m(2));
       if isnan(i1x(1))
           i1x(1) = xo(2)*(sqrt(xo(3)^2+yo(3)^2)+d_m(3))/sqrt(xo(2)^2+yo(2)^2);
           i1x(2) = xo(2)*(sqrt(xo(3)^2+yo(3)^2)-d_m(3))/sqrt(xo(2)^2+yo(2)^2);
           i1y(1) = yo(2)*(sqrt(xo(3)^2+yo(3)^2)+d_m(3))/sqrt(xo(2)^2+yo(2)^2);
           i1y(2) = yo(2)*(sqrt(xo(3)^2+yo(3)^2)-d_m(3))/sqrt(xo(2)^2+yo(2)^2);
       end
       [i2x i2y] = circcirc(xo(3),yo(3),d_m(3),xo(1),yo(1),d_m(1));
       if isnan(i2x(1))
           i2x(1) = xo(1)*(sqrt(xo(3)^2+yo(3)^2)+d_m(3))/sqrt(xo(1)^2+yo(1)^2);
           i2x(2) = xo(1)*(sqrt(xo(3)^2+yo(3)^2)-d_m(3))/sqrt(xo(1)^2+yo(1)^2);
           i2y(1) = yo(1)*(sqrt(xo(3)^2+yo(3)^2)+d_m(3))/sqrt(xo(1)^2+yo(1)^2);
           i2y(2) = yo(1)*(sqrt(xo(3)^2+yo(3)^2)-d_m(3))/sqrt(xo(1)^2+yo(1)^2);
       end
       p1 = d_m(2);
       p2 = d_m(1);
    end
    part1 = p1^5/(p1^5+p2^5);
    part2 = p2^5/(p1^5+p2^5);
    dis1 = (i1x(1)-i2x(1))^2+(i1y(1)-i2y(1))^2;
    dis2 = (i1x(1)-i2x(2))^2+(i1y(1)-i2y(2))^2;
    dis3 = (i1x(2)-i2x(1))^2+(i1y(2)-i2y(1))^2;
    dis4 = (i1x(2)-i2x(2))^2+(i1y(2)-i2y(2))^2;
    allmin = min([dis1,dis2,dis3,dis4]);
    if allmin == dis1
        x_m = (part2*i1x(1)+part1*i2x(1));
        y_m = (part2*i1y(1)+part1*i2y(1));
    elseif allmin == dis2
        x_m = (part2*i1x(1)+part1*i2x(2));
        y_m = (part2*i1y(1)+part1*i2y(2));
    elseif allmin == dis3
        x_m = (part2*i1x(2)+part1*i2x(1));
        y_m = (part2*i1y(2)+part1*i2y(1));
    elseif allmin == dis4
        x_m = (part2*i1x(2)+part1*i2x(2));
        y_m = (part2*i1y(2)+part1*i2y(2));
    end
    delta_X = [delta_X;x_m-xx];
    delta_Y = [delta_Y;y_m-yy];
    %  method 2 LLS
    gamma1 = d_m(2)^2-d_m(1)^2-(xo(2)^2-xo(1)^2+yo(2)^2-yo(1)^2);
    gamma2 = d_m(3)^2-d_m(1)^2-(xo(3)^2-xo(1)^2+yo(3)^2-yo(1)^2);
    A = 2*[xo(1)-xo(2) yo(1)-yo(2)
       xo(1)-xo(3) yo(1)-yo(3)];
    tag_m = inv(A)*[gamma1;gamma2];
    x_m0 = tag_m(1);
    y_m0 = tag_m(2);
    
    delta_X0 = [delta_X0;x_m0-xx];
    delta_Y0 = [delta_Y0;y_m0-yy];
    
    %disp(num2str(count))
    
    % improve process
    dis_1v2 =(delta_X(end)-delta_X0(end))^2+(delta_Y(end)-delta_Y0(end))^2; 
    if dis_1v2>3*allmin
        newx = xo(mincircle)+d_m(mincircle)*(delta_X0(end)+xx-xo(mincircle))/...
            sqrt((delta_X0(end)+xx-xo(mincircle))^2+(delta_Y0(end)+yy-yo(mincircle))^2);
        newy = yo(mincircle)+d_m(mincircle)*(delta_Y0(end)+yy-yo(mincircle))/...
            sqrt((delta_X0(end)+xx-xo(mincircle))^2+(delta_Y0(end)+yy-yo(mincircle))^2);
        delta_X(end) = newx-xx;
        delta_Y(end) = newy-yy;
    end
    
end
% plot
close all;clc;
subplot(241);
hold on;axis equal;box on;
title(['likelyhood','(',num2str(xx),',',num2str(yy),')'])
plot(delta_X+xx,delta_Y+yy,'.k','MarkerSize',5);
fimplicit(@(x,y) (x-xx).^2+(y-yy).^2-0.8.^2,'r');hold on;
xlim([xx-5 xx+5]);ylim([yy-5 yy+5]);

subplot(242);
hold on;axis equal;box on;
title(['regular: ',num2str(count)])
plot(delta_X0+xx,delta_Y0+yy,'.k','MarkerSize',5);
fimplicit(@(x,y) (x-xx).^2+(y-yy).^2-0.8.^2,'r');hold on;
xlim([xx-5 xx+5]);ylim([yy-5 yy+5]);

subplot(245);hold off;
histogram(delta_X,20);hold on;
title(['muX=',num2str(mean(delta_X),2),'  sigX=',num2str(std(delta_X),2) char(10) ...
    'muY=',num2str(mean(delta_Y),2),'  sigY=',num2str(std(delta_Y),2)])
axis square;
histogram(delta_Y,20);

subplot(246);hold off;
histogram(delta_X0,20);hold on;
title(['muX=',num2str(mean(delta_X0),2),'  sigX=',num2str(std(delta_X0),2) char(10) ...
    'muY=',num2str(mean(delta_Y0),2),'  sigY=',num2str(std(delta_Y0),2)])
axis square;
histogram(delta_Y0,20);

subplot(2,4,[3,4,7,8]);
axis square;hold on;box on;
Acc1 = sqrt(delta_X.^2+delta_Y.^2);
RMSE1 = sqrt(sum(Acc1.^2)/count);
[h1 bins1] = hist(Acc1,100);
F1 = cumsum(h1)/count;
plot(bins1,F1);
xlabel('Acc');ylabel('F(Acc)');
Acc2 = sqrt(delta_X0.^2+delta_Y0.^2);
RMSE2 = sqrt(sum(Acc2.^2)/count);
[h2 bins2] = hist(Acc2,100);
F2 = cumsum(h2)/count;
plot(bins2,F2);
legend(['RMSE1:',num2str(RMSE1,2)],['RMSE2:',num2str(RMSE2,2)]);

sn = ['likelyhood/' num2str(xx) '-' num2str(yy) '.jpg'];
% saveas(gca,sn);
%     end
% end

