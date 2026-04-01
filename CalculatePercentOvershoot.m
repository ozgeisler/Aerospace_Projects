function PO = CalculatePercentOvershoot(t,y)

y_final = y(end);
y_max = max(y);

if y_max > y_final
    PO = ((y_max - y_final) / abs(y_final)) * 100;
    else
        PO = 0; % No overshoot
    end
end
   
