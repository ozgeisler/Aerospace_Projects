function Tr = RiseTime(t,y,deltastart,deltastop)
y_final = y(end);
y_start = deltastart*y_final;
y_stop = deltastop*y_final;

t_start = t(find(y>=y_start,1,'first'));
t_stop = t(find(y>=y_stop,1,'first'));

Tr = t_stop - t_start;

end