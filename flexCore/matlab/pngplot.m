function pngplot(fileName,axisId,data,row,col)

set(0, 'defaultfigurecolor', 'w')
colorStr={'-r','-b','-g','-k','-m','-y','--r','--b','--g','--k','--m','--y'};
labels={'position','velocity','acceralation','jerk'};
legends = {}; % 保存图例文本
handles = []; % 保存曲线句柄
img=figure;
for i=1:row
    for j=1:col
        hid=(i-1)*col+j;%子图subplot(row,col,hid)
        fig=subplot(row,col,hid);             
        for k=1:length(data)
            dataId=axisId(k)+1;%轴id对应的数据id
            axId=axisId(k);
            if(isempty(data{dataId}))
                continue;
            end
            x=data{dataId}(:,1);
            y=data{dataId}(:,hid+1);
            handles(dataId)=plot(x,y,colorStr{dataId});
            legends{dataId}=['axis-',num2str(axId)];
             
        end
        box off;
        grid on;
        legend(fig,handles,legends);
        ylabel(labels{hid});
    end
    xlabel('time(s)')
end

% 生成文件名
path=datetime('now', 'Format', 'yyyyMMdd');
path=['E:/gogsRepo/AdaptiveX文档/',char(path)];
mkdir(char(path));
currentTime = datetime('now', 'Format', 'yyyyMMdd_HHmmss');
fname = sprintf('%s/%s_%s.png', char(path),fileName,char(currentTime));
% 保存为高分辨率PNG
exportgraphics(img, fname, ...
'Resolution', 300, ...       % 分辨率（默认150）
'BackgroundColor', 'white'); % 背景颜色
