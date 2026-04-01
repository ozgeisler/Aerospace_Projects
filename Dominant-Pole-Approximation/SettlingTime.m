function Ts = SettlingTime(t,y,delta)
y_final = y(end);
upper_bound = y_final + delta * abs(y_final);
lower_bound = y_final - delta * abs(y_final);

outside_indices = find(y > upper_bound | y < lower_bound);

if isempty(outside_indices)
        Ts = t(1); % Already settled
    else
        Ts = t(outside_indices(end));
    end
end
