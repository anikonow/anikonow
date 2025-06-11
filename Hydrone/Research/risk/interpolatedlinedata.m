%% Ref Depth Plot%%
clc
clear

data = readtable("krigged_ref_line.csv");
data1 = readtable("echotxt12bit.txt");
plot(data.Distance_Degree_,data.Line1_Band_1_SurfaceValue_,'-',LineStyle="--")
P=max(data.Distance_Degree_);
O=size(data1.Corr_Depth1,1);
I=P/(O-1);
i=0:I:P;
hold on
plot(i,data1.Corr_Depth1,'-')
plot(i,data1.Corr_Depth2)


xlabel('Degrees')
ylabel('Depth (ft)')
legend('Ref','Low Frequency Depth','High Frequency Depth')
title('Output type Echotxt 12bit Profile')
hold off
