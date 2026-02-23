clc
clear
close("all")
set(0, 'defaultfigurecolor', 'w')
%物理轴
axisId=[0;1;2;3];
%导入数据
data={};
for i=1:length(axisId)
    file=['E:\gogsRepo\adxlib\debug_windows\drv',num2str(axisId(i)),'.txt'];
    data{i}=load(file);
end
fileName="test_GantryMove";
row=3;
col=1;
pngplot(fileName,axisId,data,row,col)
gifplot(fileName,axisId,data,row,col,10);