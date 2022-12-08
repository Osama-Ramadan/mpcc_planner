fileName = 'acc_air.json'; % filename in JSON extension
fid = fopen(fileName); % Opening the file
raw = fread(fid,inf); % Reading the contents
str = char(raw'); % Transformation
fclose(fid); % Closing the file
data = jsondecode(str); % Using the jsondecode function to parse JSON from string

data = struct2cell(data);
stamptime = [];
steeringangle = [];
steeringcommand = [];
for i = 1:length(data)
    datapoint = data(i);
    stamptime = [stamptime ; datapoint{1}.recTime];
    steeringangle = [steeringangle ; datapoint{1}.steerAngle_sp];
    steeringcommand = [steeringcommand ; datapoint{1}.steerAngle];
end
stamptime(1:end) = stamptime(1:end)-stamptime(1);
steeringangle = steeringangle*(180/pi);

figure(51)
plot(stamptime, steeringangle, "LineWidth", 1.5);hold on
title("Steering System Response")
xlabel("Time (ms)")
ylabel("Steering Angle (deg)")
legend( "Steering Response")