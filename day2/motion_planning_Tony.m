%% edit by Tonywang to accomplish 人工势场法
%代码参考 https://blog.csdn.net/qq_28697571/article/details/84313443，不完全一样；
clc;
clear all;
close all;

%% 初始化车的参数
x0=[0 0]; %起点位置
k=15; %计算引力需要的增益系数
K=0;
m=5;
P0=2.5;
n=7;
a=0.5;
l=0.2;
J=200;
%end

%给出障碍和目标信息
Xsum=[10 10;1 1.2;3 2.5;4 4.5;3 6;6 2;5.5 5.5;8 8.5];
Xj=x0;%j=1循环开始，将车的起始坐标赋给xj
%***********************初始化结束**********************

%% 主题循环
for j=1:J
    Goal(j,1)=Xj(1);
    Goal(j,2)=Xj(2);
    %计算角度
    Theta=compute_angle(Xj,Xsum,n);%计算车与障碍、目标点之间的x轴的夹角，统一规定角度为逆时针方向为正；
    %计算引力
    Angle=Theta(1);
    Anlge_at=Theta(1);
    [Yatx,Yaty]=compute_Attract(Xj,Xsum,k,Angle);
    %计算斥力
    for i=1:n
        angle_re(i)=Theta(i+1);
    end
    [Yrerxx,Yreryy,Yataxx,Yatayy]=compute_repulsion(Xj,Xsum,m,Anlge_at,angle_re,n,P0,a);
    %计算合力，x方向+y方向
    Fsumxj=Yatx+Yrerxx+Yataxx;
    Fsumyj=Yaty+Yreryy+Yatayy;
    position_angle(j)=atan(Fsumyj/Fsumxj);
    %计算下一步的位置
    Xnext(1)=Xj(1)+cos(position_angle(j));
    Xnext(2)=Xj(2)+cos(position_angle(j));
    %保存车的位置
    Xj=Xnext;
    K=j;
    Goal(K,1)=Xj(1,1);
	Goal(K,2)=Xj(1,2);
    %判断是否到达终点
    if(Xj(1)-Xsum(1,1)>0&&Xj(2)-Xsum(1,2)>0)
        break;
    end
end
Goal(K+1,1)=Xsum(1,1);
Goal(K+1,2)=Xsum(1,2);
%画图
X=Goal(:,1);
Y=Goal(:,2);
x=Xsum(2:length(Xsum),1);
y=Xsum(2:length(Xsum),2);
plot(x,y,'o',10,10,'v',0,0,'ms',X,Y,'.r');


%% 计算车与障碍、目标点之间的夹角
function Y = compute_angle(Xj,Xsum,n)
    for i=1:n+1
        deltaX(i)=Xsum(i,1)-Xj(1,1);
        deltaY(i)=Xsum(i,2)-Xj(1,2);
        r(i)=sqrt(deltaX(i)^2+deltaY(i)^2);
        %if deltaX(i)>0
        theta=acos(deltaX(i)/r(i));
        %else
        %    theta=pi-acos(deltaX(i)/r(i)); %不知道为什么要减？
        %end
        angle=theta;
        Y(i)=angle; 
    end
end

%% 计算引力模块
function [Yatx,Yaty] = compute_Attract(X,Xsum,k,angle)
    R=(X(1)-Xsum(1,1))^2+(X(2)-Xsum(1,2))^2;
    r=sqrt(R);
    Yatx=k*r*cos(angle);
    Yaty=k*r*sin(angle);
end

%% 计算斥力模块
function [Yrerxx,Yreryy,Yataxx,Yatayy] = compute_repulsion(X,Xsum,m,Angle_at,angle_re,n,P0,a)
Rat=(X(1)-Xsum(1,1))^2+(X(2)-Xsum(1,2))^2;
rat=sqrt(Rat);
for i=1:n
    Rrei(i)=(X(1)-Xsum(i+1,1))^2+(X(2)-Xsum(i+1,2))^2;
    rrei(i)=sqrt(Rrei(i));
    if rrei(i)>P0
        Yrerx(i)=0;
        Yrery(i)=0;
        Yatax(i)=0;
        Yatay(i)=0;
    else
        if rrei(i)<P0/2
            Yrer(i)=m*(1/rrei(i)-1/P0)*(1/Rrei(i))*(rat^a);
            Yata(i)=a*m*((1/rrei(i)-1/P0)^2)*(rat^(a-1))/2;
            Yrerx(i)=(1+0.1)*Yrer(i)*cos(angle_re(i));
            Yrery(i)=(1-0.1)*Yrer(i)*sin(angle_re(i));
            Yatax(i)=Yata(i)*cos(Angle_at);
            Yatay(i)=Yata(i)*sin(Angle_at);
        else
            Yrer(i)=m*(1/rrei(i)-1/P0)*(1/Rrei(i))*(rat^2);%Rat=rat^2
            Yata(i)=m*((1/rrei(i)-1/P0)^2)*(rat);
            Yrerx(i)=Yrer(i)*cos(angle_re(i));
            Yrery(i)=Yrer(i)*sin(angle_re(i));
            Yatax(i)=Yata(i)*cos(Angle_at);
            Yatay(i)=Yata(i)*sin(Angle_at);
        end
    end
end
Yrerxx=sum(Yrerx);
Yreryy=sum(Yrery);
Yataxx=sum(Yatax);
Yatayy=sum(Yatay);
end
