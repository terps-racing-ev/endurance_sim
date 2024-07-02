function losses = free_losses(rpm)
%in kW
losses = (3E-05*rpm^2 + 0.0987*rpm - 3.7363)/1000;
end