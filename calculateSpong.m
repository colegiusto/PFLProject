function spong = calculateSpong(x,p)

q1 = x(1);
q2 = x(2);
dq1 = x(3);
dq2 = x(4);

g = 9.8;
m1 = p.m1;
m2 = p.m2;
I1 = p.I1;
I2 = p.I2;

l1 = p.l1;
l2 = p.l2;

lc1 = p.lc1;
lc2 = p.lc2;

spong.d11 = m1*lc1^2 + m2*(l1^2+lc2^2+2*lc1*lc2.*cos(q2)) + I1+ I2;
spong.d22 = m2*lc2^2 + I2;

spong.d12 = m2*(lc2^2+l1*lc2.*cos(q2)) + I2;
spong.d21 = spong.d12;

spong.h1 = -m2*l1*lc2.*sin(q2).*dq2.^2 - 2*m2*l1*lc2.*sin(q2).*dq2.*dq1;
spong.h2 = m2*l1*lc2.*sin(q2).*dq1.^2;

spong.phi1 = (m1*lc1+m2*l1)*g.*cos(q1)+m2*lc2*g.*cos(q1+q2);
spong.phi2 = m2*lc2*g.*cos(q1+q2);

end

