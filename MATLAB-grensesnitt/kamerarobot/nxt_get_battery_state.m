function [recharge chargetime] = nxt_get_battery_state()
%checks whether the battery needs charging

    %Set recharge limits
    chargeVoltage = 8000; % lower battery limit in mV
    chargetime = 3600;      % secs

    mvolts = NXT_GetBatteryLevel; %millivolts
    disp('Battery: ');
    disp(mvolts);

    if (mvolts < chargeVoltage)
        recharge = 1;
    else
        recharge = 0;
    end

end