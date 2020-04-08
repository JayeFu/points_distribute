% 用于产生三个区域的边界

clear;
% 边界1
x1 = 0:0.1:2.5;
z1 = -0.75:0.1:0.75;
[xx1, zz1] = meshgrid(x1, z1);
yy1 = zeros(size(xx1)); % 边界1的y值为0
surf(xx1, yy1, zz1);
xlabel('$-2.5\le x\le 2.5$', 'interpreter', 'latex');
ylabel('$-1.5\le y\le 1.5$', 'interpreter', 'latex');
zlabel('$-0.75\le x\le 0.75$', 'interpreter', 'latex');
text(-1.5, 0, 0,'区域1');
text(1.5, -0.75, 0,'区域2');
text(1.5, 0.75, 0,'区域3');
text(1.5, 0, 1, '边界1', 'Color', 'Red');
text(-0.5, 0.5*sqrt(3), 1, '边界2', 'Color', 'Red');
text(-0.5, -0.5*sqrt(3), 1, '边界3', 'Color', 'Red');
text(0.1, -0.1, 0.1, '原点', 'Color', 'Red');
hold on;
plot3(0,0,0,'rs','LineWidth',5,'MarkerSize',10);

% 边界2
x2 = -0.9:0.1:0;
z2 = -0.75:0.1:0.75;
[xx2, zz2] = meshgrid(x2, z2);
yy2 = -sqrt(3)*xx2;
surf(xx2, yy2, zz2);

% 边界3
x3 = -0.9:0.1:0;
z3 = -0.75:0.1:0.75;
[xx3, zz3] = meshgrid(x3, z3);
yy3 = sqrt(3)*xx3;
surf(xx3, yy3, zz3);

% 左
yl = -1.5:0.1:1.5;
zl = -0.75:0.1:0.75;
[yyl, zzl] = meshgrid(yl, zl);
xxl = -2.5*ones(size(yyl));
mesh(xxl, yyl, zzl);

% 右
yr = -1.5:0.1:1.5;
zr = -0.75:0.1:0.75;
[yyr, zzr] = meshgrid(yr, zr);
xxr = 2.5*ones(size(yyr));
mesh(xxr, yyr, zzr);

% 前
xf = -2.5:0.1:2.5;
zf = -0.75:0.1:0.75;
[xxf, zzf] = meshgrid(xf, zf);
yyf = 1.5*ones(size(xxf));
mesh(xxf, yyf, zzf);

% 后
xb = -2.5:0.1:2.5;
zb = -0.75:0.1:0.75;
[xxb, zzb] = meshgrid(xb, zb);
yyb = -1.5*ones(size(xxb));
mesh(xxb, yyb, zzb);