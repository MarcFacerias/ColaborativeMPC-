syms x u

dx = x^2 + u*x;

Jb = jacobian(dx, [x, u]);

x = 1;
u = 1;
A = double(subs(Jb(1)));
B = double(subs(Jb(2)));

t = linspace(-5,5,100);

for i =1:100
    x = t(i);
    u = t(i);

    dx2(i) = 2 + A*(x-1) + B*(u-1);
    dx1(i) = subs(dx);

end

hold on 

plot(t,dx1)
plot(t,dx2)

hold off 