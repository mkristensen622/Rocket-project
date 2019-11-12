%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% This script takes sensor data from the VTS-machines in ASCII format     %
% from a student rocket launched from Andøya Space Center, and converts   %
% it into the actual units of the sensors.                                %
%                                                                         %
% NAROM AS                                                                %
% Last modified: 01.07.2015                                               %   
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% -------------- import data from datafile ------
close all;
clear all;
filename = uigetfile('*.txt'); %
if ischar(filename)
    file = filename;
end
dataset = importdata(file);
ldata = size(dataset.data,1);
rawdata = dataset.data;
raw_text = dataset.textdata; 

%% Define the different channels in use and sort the data 
A0 = rawdata(1:end,4); %Current
A1 = rawdata(1:end,8); %Lys
A2 = rawdata(1:end,2); %Acc-X
A3 = rawdata(1:end,3); %Acc-Y
A4 = rawdata(1:end,11); %MagY
A5 = rawdata(1:end,12); %MUX
A6 = rawdata(1:end,9); %MagX
A7 = rawdata(1:end,10); %Trykk
D0 = rawdata(1:end,5); 
D1 = rawdata(1:end,6); 
FrameCounter = rawdata(1:end,7); 
time = rawdata(1:end,1);

%% ------ If a MUX is used -----------
ind0 = D0 == 252; %
ind1 = D0 == 253; %
ind2 = D0 == 254; %
ind3 = D0 == 255; %
time0 = time(ind0);
time1 = time(ind1);
time2 = time(ind2);
time3 = time(ind3);
A0_0 = rawdata(ind0,12);
A0_1 = rawdata(ind1,12);
A0_2 = rawdata(ind2,12);
A0_3 = rawdata(ind3,12);

% Multiplexer1  rawoutput
RawMUX1 = A5; 

figure;             
plot(time,RawMUX1);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Multiplexer 1');   %Plot title


% Multiplexer0 output
MUX0 = A0_0; %Converts data to volts (4.3 is the multiplier for the voltage divider)

figure;             
plot(time0,MUX0);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Multiplexer Ch0');   %Plot title

% Multiplexer1 output
MUX1 = A0_1; %Converts data to volts (4.3 is the multiplier for the voltage divider)

figure;             
plot(time1,MUX1);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Multiplexer Ch1');   %Plot title

% Multiplexer2 output
MUX2 = A0_2; %Converts data to volts (4.3 is the multiplier for the voltage divider)

figure;             
plot(time2,MUX2);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Multiplexer Ch2');   %Plot title

% Multiplexer2 output
MUX3 = A0_3; %Converts data to volts (4.3 is the multiplier for the voltage divider)

figure;             
plot(time3,MUX3);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Multiplexer Ch3');   %Plot title

%D0
figure;             
plot(D0);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Digital 0');   %Plot title

%D1
figure;             
plot(D1);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Digital 1');   %Plot title

%-----------------------------------

%Check where you are getting data from in the equations.


%% FrameCounter 
figure;             
plot(time,FrameCounter);
ylabel('Frame count'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Frame Counter [0 - 65534]');   %Plot title


%% PCB Temperature sensor 
Analog0_1 = A0_1*5/255;  %Converts data to volts
PcbTempGain=10.2;
Analog0_1 = Analog0_1/(0.01*PcbTempGain) ;  %Converts to Celsius

figure;             
plot(time1,Analog0_1);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Internal Temperature');   %Plot title

%% Nose Temperature sensor 
Analog0_0 = A0_0*5/255; %Converts data to volts
ExtTempGain=4.9;
Analog0_0 = Analog0_0/(0.01*ExtTempGain); %Converts to Celsius

figure;             
plot(time0,Analog0_0);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Nose Temperature');   %Plot title

%% Antenna Temperature sensor 
Analog0_2 = A0_2*5/255;  %Converts data to volts
PcbTempGain=4.9;
Analog0_2 = Analog0_2/(0.01*PcbTempGain) ;  %Converts to Celsius

figure;             
plot(time2,Analog0_2);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Antenna Temperature');   %Plot title

%% Structure Temperature sensor 
Analog0_3 = A0_3*5/255; %Converts data to volts
ExtTempGain=4.9;
Analog0_3 = Analog0_3/(0.01*ExtTempGain); %Converts to Celsius

figure;             
plot(time3,Analog0_3);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Structure Temperature');   %Plot title
%% Magnetic field sensor X
Analog3 = A6*5/255 ; %Converts data to volts

figure;             
plot(time,Analog3);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Magnetic X');   %Plot title

%% Magnetic field sensor Y 
Analog4 = A4*5/255 ; %Converts data to volts

figure;             
plot(time,Analog4);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Magnetic');   %Plot title

%% Air Pressure Sensor
Analog6 = A7*5/255; %Converts data to volts
m = 0.475;          
n = 0.0045;
Trykk = (((Analog6*100)/4.7)+15)*1000;   %Converts to Pa

figure;             
plot(time,Trykk);
ylabel('Pressure[Pa]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Pressure');   %Plot title

%% Accelerometer X-axis 
Analog1 = A2*5/255 ; %Converts data to volts
AccXGain = 0.020;
AccXOffset = 2.68;        %Calibrated at zero-g (nominal 2.5)
Analog1 = (Analog1-AccXOffset)/AccXGain; %Converts to G

figure;             
plot(time,Analog1);
ylabel('[g]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Acceleration Y');   %Plot title


%% Accelerometer Y-axis 
Analog2 = A3*5/255; %Converts data to volts
AccYGain = 0.040;     % 
AccYOffset = 2.64;  % Calibrated at zero-g (nominal 2.5)
Analog2 = (Analog2-AccYOffset)/AccYGain; %Converts to G

figure;             
plot(time,Analog2);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Acceleration X');   %Plot title

%% Light Sensor (Phototransistor) 
Analog5 = A1*5/255 ; %Converts data to volts

figure;             
plot(time,Analog5);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Light');   %Plot title


%% Current output
Analog7 = A0*5/255*4.3; %Converts data to volts (4.3 is the multiplier for the voltage divider)

figure;             
plot(time,Analog7);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Battery');   %Plot title


%% Height plot (Derived from pressure sensor)

r=8.3145; %Specific gas constant
lb=-0.0056; %Temperature gradient [K/m]
Tb=273.15-0.4; %Starting temperature at ground (in kelvin)
P0=101841; %Ground pressure in Pa
M = 0.02895; %Molecular mass of air.
g = 9.80665;
%altitude= ; %Altitude in meters

height= (Tb/lb)*[(Trykk/P0).^((-r*lb)/(g*M))-1];

%beta = -T0/a;
%gamma = -(R*a)/(g*M);
%height = beta*((Analog6/P0).^gamma);

figure;             
plot(time,height);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Height [m]');   %Plot title
%%
%Spinn speed (Derived from sentripetal acceleration)
%spinn =  0; %sentripital force in 'g'

radius = 37E-3;                        %Accelerometer distanse to center of the rocket
spinn = sqrt(abs(Analog2)*g./radius)/(2*pi); %Angular speed

figure;             
plot(time,spinn);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Spinn');   %Plot title

%% Velocity
offset = 1.292;
velocity = cumtrapz(time,(g*(Analog1+offset)));

figure;             
plot(time,velocity);
ylabel('[ ]'); %Label for the Y-axis
xlabel('Time [s]');        %Label for the X-axis (normally time)
grid on
title('Velocity');   %Plot title


%% Cleaning Workspace (Removing unused variables)
clear('dataset','rawdata');
clear('Time','hours','minutes','seconds','offset_time','i','raw_text');
clear ('PcbTempGain','ExtTempGain','AccXGain','AccXOffset','AccYGain','AccYOffset')
clear ('m','n','radius');
clear ('A0','A1','A2','A3','A4','A5','A6','A7');


%% Notes.. 


