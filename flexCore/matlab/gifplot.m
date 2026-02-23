
function  gifplot(fileName,axisId,data,row,col,dx)

% 生成文件名
path=datetime('now', 'Format', 'yyyyMMdd');
path=['E:/gogsRepo/AdaptiveX文档/',char(path)];
mkdir(char(path));
currentTime = datetime('now', 'Format', 'yyyyMMdd_HHmmss');
fname = sprintf('%s/%s_%s.gif', char(path),fileName,char(currentTime));

set(0, 'defaultfigurecolor', 'w')
colorStr={'-r','-b','-g','-k','-m','-y','--r','--b','--g','--k','--m','--y'};
labels={'position','velocity','acceralation','jerk'};
legends = {}; % 保存图例文本
handles = []; % 保存曲线句柄
img=figure;


xmin=0;
xmax=data{1}(end,1);
for i=1:row
    for j=1:col
        hid=(i-1)*col+j;%子图subplot(row,col,hid)
        ymax=0;
        ymin=0;
        fig=subplot(row,col,hid);             
        for k=1:length(data)
            
            if(isempty(data{k}))
                continue;
            end
            axId=axisId(k);
            dataId=k;
            
            handles(hid,dataId)=plot(NaN, NaN,colorStr{dataId});
            legends{dataId}=['axis-',num2str(axId)];
            ymax=max([data{dataId}(:,hid+1);ymax]);
            ymin=min([data{dataId}(:,hid+1);ymin]);
            xlim([xmin xmax]);
            ylim([ymin,ymax]);
        end
        box off;
        grid on;
        legend(fig,handles(i,:),legends);
        ylabel(labels{hid});
    end
    xlabel('time(s)')
end

dataNum=0;
for i=1:length(data)
    if(isempty(data{i}))
        continue;
    end
    dataNum=max(length(data{i}(:,1)),dataNum);
end

ndx=0;
m=1;
while(m<dataNum)
    m=m+ndx*dx;
    ndx=ndx+1;
    if(m>dataNum)
       m=dataNum;
    end
    % 生成数据并绘图
    for i=1:row
        for j=1:col
            hid=(i-1)*col+j;%子图subplot(row,col,hid)                    
            for k=1:length(data)        
                if(isempty(data{k}))
                    continue;
                end
                dataId=k;                    
                x=data{dataId}(1:m,1);
                y=data{dataId}(1:m,hid+1);
                set(handles(hid,dataId), 'XData', x, 'YData', y);  
            end
        end
    end
    % 捕获帧并转换颜色空间
    frame = getframe(img);
    [im, cm] = rgb2ind(frame.cdata, 256); % 关键修正点
    
    % 写入GIF时传递正确的cm参数
    if m == 1
        imwrite(im, cm, fname, 'gif', ...
                'LoopCount', inf, 'DelayTime', 0.01*dx);
    else
        imwrite(im, cm, fname, 'gif', ...
                'WriteMode', 'append', 'DelayTime', 0.01*dx);
    end
end
fprintf("gif plot finish\n");