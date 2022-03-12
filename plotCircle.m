function plotCircle(x,y,color)

th = 0:pi/20:2*pi;

xunit = 0.2 * cos(th) + x;
yunit = 0.2 * sin(th) + y;
h = fill(xunit, yunit, color);

hold on;

end