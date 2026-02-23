clc
clear
close("all")
set(0, 'defaultfigurecolor', 'w')
%测试路径点
pos=load("testPath.txt");
plot3(pos(:,1),pos(:,2),pos(:,3),'-ro')
title("测试路径")
xlabel("x")
ylabel("y")
zlabel("z")
grid on
axis equal
grpNum=4;
%插补轨迹点
for i=1:grpNum    
    file=['inpFile',num2str(i-1),'.txt'];
    pos=load(file);
    if(isempty(pos))
        figure
        plot3(pos,pos ,pos ,'-r.');
        axis equal
        grid on
        title(['轴组',num2str(i-1)]);
        xlabel("x")
        ylabel("y")
        zlabel("z")
    else
        figure
        plot3(pos(:,2),pos(:,3),pos(:,4),'-r.');
        axis equal
        grid on
        title(['轴组',num2str(i-1)]);
        xlabel("x")
        ylabel("y")
        zlabel("z")
    end

end
%笛卡尔轴坐标
coodDim=3;
colorStr={"-r","-b","-g","-k","-m","-y","--r","--b","--g","--k","--m","--y"};
legends = {}; % 保存图例文本
handles = []; % 保存曲线句柄
Ts=0.001;
for i=1:grpNum
    file=['inpFile',num2str(i-1),'.txt'];
    pos=load(file);
    
    if(isempty(pos))
       continue;
    else
        coodDim=length(pos(1,:))-1;
        figure
        subplot(3,1,1)
        legends = {}; % 保存图例文本
        handles = []; % 保存曲线句柄
        for j=1:coodDim
            h=plot(pos(:,1+j),colorStr{j});
            handles=[handles, h];
            legends{end+1}=['cart-',num2str(j)];
            grid on
            hold on 
        end
        title(['笛卡尔轴坐标,轴组',num2str(i-1)]);
        xlabel("ms")
        legend(handles, legends);

        subplot(3,1,2)
        legends = {}; % 保存图例文本
        handles = []; % 保存曲线句柄

        for j=1:coodDim
     
            y=pos(:,1+j);
            v=(y(2:end)-y(1:end-1))/Ts;
            h=plot(v,colorStr{j});
            handles=[handles, h];
            legends{end+1}=['cart-',num2str(j)];
            grid on
            hold on            
        end
        title('笛卡尔轴速度');
        xlabel("ms")
        legend(handles, legends); 

        subplot(3,1,3)
        legends = {}; % 保存图例文本
        handles = []; % 保存曲线句柄
        for j=1:coodDim
            a=(v(2:end)-v(1:end-1))/Ts;
            h=plot(a,colorStr{j});
            handles=[handles, h];
            legends{end+1}=['cart-',num2str(j)];
            grid on
            hold on            
        end
        title('笛卡尔轴加速度');
        xlabel("ms")
        legend(handles, legends);
        savepng(h,"axis")
    end
end

