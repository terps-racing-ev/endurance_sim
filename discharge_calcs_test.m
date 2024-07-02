function func = discharge_calcs_test()

syms d c pow
I = 30; aV = 4.15; aC = 4.1; bV = 3.6; bC = 3.6; mV = 3;
s = 96; p = 4;
f = d*( (aV*s - d*(aV*s-bV*s)/(I*p)) - c*( (aV*s-mV*s)/(aC*p) - ...
    (d/(I*p))*( (aV*s-mV*s)/(aC*p) - (bV*s-mV*s)/(bC*p) ) ) ) - pow;
a = solve(f == 0, d);
func = simplify(a(2));
end