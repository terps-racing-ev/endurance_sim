function torque = torquevspower(power,rpm)
%voltage in kV because power is in kW and torque curve thing should be in
%amps
%power = power*2;
w = rpm*2*pi/60;
torque = power*1000/w;
%{
voltage = .3456;%max is 403.2 kV, nom is 345.6V

current = GR*power/(sqrt(2)*voltage);
if current > 220
    torque = .375*current;
else
    torque = 0.409*current;
end
%}
end