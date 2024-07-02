function [m_eff,inv_eff] = efficiency_calcs(torque, rpm)
inv_eff = 0.90;

if rpm <1000
    m_eff = 0.88;

elseif rpm > 1000 && rpm <= 1500
    if torque > 40
        m_eff = 0.935;
    elseif torque > 30
        m_eff = 0.91;
    else 
    m_eff = 0.88;
    end
elseif rpm > 1500 && rpm <= 2000
    if torque > 40
        m_eff = 0.94;
    elseif torque > 30
        m_eff = 0.92;
    else 
    m_eff = 0.9;
    end
    
elseif rpm > 2000 && rpm <= 3000
    if torque > 55
    m_eff = 0.96;
    elseif torque > 40
        m_eff = 0.95;
    elseif torque > 34
        m_eff = 0.94;
    elseif torque > 30
        m_eff = 0.92;
    else 
    m_eff = 0.88;
    end

elseif rpm > 3000 && rpm <= 3500
    if torque > 50
        m_eff = 0.96;
    elseif torque > 43
        m_eff = 0.95;
    elseif torque > 38
        m_eff = 0.94;
    elseif torque > 30
        m_eff = 0.92;
    else 
    m_eff = 0.88;
    end

elseif rpm > 3500 && rpm <= 4000
    if torque > 55
        m_eff = 0.96;
    elseif torque > 50
        m_eff = 0.95;
    elseif torque > 38
        m_eff = 0.94;
    elseif torque > 30
        m_eff = 0.92;
    else 
    m_eff = 0.88;
    end
elseif rpm > 4000 && rpm <= 4250
    if torque > 65
        m_eff = 0.96;
    elseif torque > 55
        m_eff = 0.95;
    elseif torque > 40
        m_eff = 0.94;
    elseif torque > 32
        m_eff = 0.92;
    else 
    m_eff = 0.88;
    end

elseif rpm > 4250 && rpm <= 5000
    if torque > 60
        m_eff = 0.95;
    elseif torque > 55
        m_eff = 0.95;
    elseif torque > 43
        m_eff = 0.94;
    elseif torque > 34
        m_eff = 0.92;
    else 
    m_eff = 0.88;
    end

else
    m_eff = 0.90;
end