clc
clear
close("all")
set(0, 'defaultfigurecolor', 'w')
%物理轴
axisId=[0;1;2;3];
colorStr={"-r","-b","-g","-k","-m","-y","--r","--b","--g","--k","--m","--y"};
%导入数据
data={};
for i=1:length(axisId)
    file=['E:\gogsRepo\adxlib\debug_windows\drv',num2str(axisId(i)),'.txt'];
    data{i}=load(file);
end
fileName="test_GantryMove";
img=figure;
grid on
hold on   
legends = {}; % 保存图例文本
handles = []; % 保存曲线句柄
for i=1:length(axisId)
    h=plot(data{i},colorStr{i});
    handles=[handles, h];
    legends{end+1}=['axis-',num2str(i)];

end
xlabel('time(ms)')
ylabel('pos(mm)')
legend(handles,legends);
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