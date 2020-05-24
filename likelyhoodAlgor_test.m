

%% plot the env
syms x; syms y;

xo = [0.15,-0.15,0];
yo = [0.0,0.0,0.15];
No = size(xo,2);
xx = 10; yy = 0;
%rescan
clc
% for xx = 1:49
%     for yy = 1:49
disp([num2str(xx) ',' num2str(yy)])
hold off;
for i = 1:No
    d(i) = norm([xo(i) yo(i)]-[xx yy]);    
    fimplicit(@(x,y) (x-xo(i)).^2+(y-yo(i)).^2-d(i).^2,'b');hold on;
end
% plot
% hold off;
plot(xo,yo,'bo','MarkerSize',8,'LineWidth',2);hold on;axis equal;
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
       mincircle = 1;
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
       mincircle = 2;
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
       mincircle = 3;
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
        other_dis = [dis1,dis3,dis4];
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
    
    if min(other_dis)<1.8*allmin
        if allmin==dis2 && min(other_dis)==dis4
            delta_X(end) = i2x(2)
            delta_Y(end) = i2y(2)
        end
    end
        
    
cla;
for i = 1:No
    fimplicit(@(x,y) (x-xo(i)).^2+(y-yo(i)).^2-d_m(i).^2,'b');hold on;
end
title(['likelyhood','(',num2str(xx),',',num2str(yy),')'])
plot(delta_X(end)+xx,delta_Y(end)+yy,'.k','MarkerSize',25);axis equal;
plot(delta_X0(end)+xx,delta_Y0(end)+yy,'.r','MarkerSize',25);axis equal;

plot(i1x,i1y,'.c','MarkerSize',15);
plot(i2x,i2y,'.c','MarkerSize',15);

fimplicit(@(x,y) (x-xx).^2+(y-yy).^2-0.8.^2,'r');hold on;
% xlim([xx-5 xx+5]);ylim([yy-5 yy+5]);
% xlim([-10 10]);ylim([-13 13]);
drawnow;
1
    %disp(num2str(count))
end

