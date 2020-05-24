%% plot the env
syms x; syms y;

xo = [1,50,50];
yo = [1,0,50];
No = size(xo,2);
xx = 0; yy = 15;

figure;hold on;axis equal;
xlim([-10 60]);ylim([-10 60])
plot(xo,yo,'bo','MarkerSize',8,'LineWidth',2);
plot(xx,yy,'ro','MarkerSize',8,'LineWidth',2);

for i = 1:No
    d(i) = norm([xo(i) yo(i)]-[xx yy]);    
    fimplicit(@(x,y) (x-xo(i)).^2+(y-yo(i)).^2-d(i).^2,'b');
end

% plot
hold off;
plot(xo,yo,'bo','MarkerSize',8,'LineWidth',2);hold on;axis equal;
xlim([-10 60]);ylim([-10 60])
plot(xx,yy,'ro','MarkerSize',8,'LineWidth',2);
for i = 1:No
    fimplicit(@(x,y) (x-xo(i)).^2+(y-yo(i)).^2-d_m(i).^2,'b');
end
% give random err
mean_err = [];
delta_X = [];
delta_Y = [];
count = 0;
while(count<300)
    count = count+1;
    for i=1:No
        rand_err(i) = normrnd(0,0.1*0.2856*d(i));
        d_m(i) = d(i)+rand_err(i);
    end
    gamma1 = xo(2)^2-xo(3)^2+yo(2)^2-yo(3)^2+d_m(3)^2-d_m(2)^2;
    gamma2 = xo(1)^2-xo(2)^2+yo(1)^2-yo(2)^2+d_m(2)^2-d_m(1)^2;
    x_m = ((yo(2)-yo(1))*gamma1+(yo(2)-yo(3))*gamma2)/...
         2/((xo(2)-xo(3))*(yo(2)-yo(1))+(xo(1)-xo(2))*(yo(2)-yo(3)));
    y_m = ((xo(2)-xo(1))*gamma1+(xo(2)-xo(3))*gamma2)/...
         2/((xo(2)-xo(1))*(yo(2)-yo(3))+(xo(2)-xo(3))*(yo(1)-yo(2))); 
    Err1 = sqrt((x_m-xx)^2+(y_m-yy)^2);
    % LLS % this method is equavalent to above
    % A = [xo(2)-xo(3) yo(3)-yo(2)
    %      xo(1)-xo(2) yo(2)-yo(1)];
    % b = [-gamma1/2
    %      gamma2/2];
    % tag_m = inv(A)*b;
    % Err2 = sqrt((tag_m(1)-xx)^2+(tag_m(2)-yy)^2);
    
    plot(x_m,y_m,'.k');drawnow;
    %mean_err = [mean_err;Err1];
    delta_X = [delta_X;x_m-xx];
    delta_Y = [delta_Y;y_m-yy];
    disp(num2str(mean(delta_X)))
end
% plot hisgram with map
[N,b] = hist3([delta_X, delta_Y],[50 50]); 
figure;
imagesc(b{1}([1 end]),b{2}([1 end]),flipud(N),[1 50]);
title(['(xx,yy)=(',num2str(xx),',',num2str(yy),')'])
colormap('jet')
colorbar
axis equal tight

%% plot the env
syms x; syms y;

xo = [0.5,50,50];
yo = [0.5,0.5,50];
No = size(xo,2);
xx = 40; yy = 35;

figure;hold on;axis equal;
xlim([-10 60]);ylim([-10 60])
plot(xo,yo,'bo','MarkerSize',8,'LineWidth',2);
plot(xx,yy,'ro','MarkerSize',8,'LineWidth',2);

%% rescan
clc
for xx = 1:49
    for yy = 1:49
% xx = 11; yy = 20;
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
while(count<300)
    count = count+1;
    for i=1:No
        rand_err(i) = normrnd(0,0.05*0.2856*d(i));
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
end
subplot(221);
hold on;axis equal;
title(['likelyhood','(',num2str(xx),',',num2str(yy),')'])
plot(delta_X+xx,delta_Y+yy,'.k','MarkerSize',5);
fimplicit(@(x,y) (x-xx).^2+(y-yy).^2-0.8.^2,'r');hold on;
xlim([xx-5 xx+5]);ylim([yy-5 yy+5]);
%drawnow;
subplot(222);
hold on;axis equal;title(['regular: ',num2str(count)])
'  sigY=',num2str(std(delta_Y),2)])
axis square;
histogram(delta_Y,20);
subplot(224);hold off;
histogram(delta_X0,20);hold on;
title(['muX=',num2str(mean(delta_X0),2),'  sigX=',num2str(std(delta_X0),2) char(10) ...
    'muY=',num2str(mean(delta_Y0),2),'  sigY=',num2str(std(delta_Y0),2)])
axis square;
histogram(delta_Y0,20);
drawnow;
sn = ['likelyhood/' num2str(xx) '-' num2str(yy) '.jpg'];

plot(delta_X0+xx,delta_Y0+yy,'.k','MarkerSize',5);
saveas(gca,sn);
    end
end
% % plot hisgram with map
% [N,b] = hist3([delta_X, delta_Y],[50 50]); 
% figure;
% imagesc(b{1}([1 end]),b{2}([1 end]),flipud(N),[1 50]);
% title(['(xx,yy)=(',num2str(xx),',',num2str(yy),')'])
% colormap('jet')
% colorbar
% axis equal tight

