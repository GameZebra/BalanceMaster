clear

data = readmatrix("static_graph_1.csv");
Td = 0.002;
time = 0:Td:(length(data)-1)*Td;
Rspeed = data(:,2);
Lspeed = data(:,3);
Rcontrol = data(:,4);
Lcontrol = data(:,5);


plot(time, Rspeed);


