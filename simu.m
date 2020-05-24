%% simulate random number
collection = [];
figure;
while(1)
    rnd = rand;
    collection = [collection,rnd];
    edges = [0:0.1:1];
    h = histogram(collection,edges);
    drawnow;
    pause(0.1);
end

%% simulate gauss random number
rng('default');
collection = [];
figure(1);
while(1)
    rnd = normrnd(0.5,0.1);
    collection = [collection,rnd];
    edges = [0:0.05:1];
    h = histogram(collection,edges);
    drawnow;
    % pause(0.1);
end

%% plot the env
syms x; syms y;

xo = [0,50,50];
yo = [0,0,50];
No = size(xo,2);
xx = 23; yy = 20;

figure;hold on;axis equal;
xlim([-10 60]);ylim([-10 60])
plot(xo,yo,'bo','MarkerSize',8,'LineWidth',2);
plot(xx,yy,'ro','MarkerSize',8,'LineWidth',2);

for i = 1:No
    d(i) = norm([xo(i) yo(i)]-[xx yy]);    
    fimplicit(@(x,y) (x-xo(i)).^2+(y-yo(i)).^2-d(i).^2,'b');
end

%% plot
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
while(1)
    for i=1:No
        rand_err(i) = normrnd(0,0.3);
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

%% plot:
hold off;
plot(xo,yo,'bo','MarkerSize',8,'LineWidth',2);hold on;axis equal;
xlim([-10 60]);ylim([-10 60])
plot(xx,yy,'ro','MarkerSize',8,'LineWidth',2);
for i = 1:No
    fimplicit(@(x,y) (x-xo(i)).^2+(y-yo(i)).^2-d_m(i).^2,'b');
end
%xlim([15 25]);ylim([15 25])

%% plot hisgram with map
[N,b] = hist3([delta_X, delta_Y],[50 50]); 
figure
imagesc(b{1}([1 end]),b{2}([1 end]),flipud(N),[1 50]);
colormap('jet')
colorbar
axis equal tight
