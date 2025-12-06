%% PFL swingup for the Acrobot
% Define parameters of dynamics (From spong95)

p.g = 9.8;
p.m1 = 1;
p.m2 = 1;

p.l1 = 1;
p.l2 = 2;

p.lc1 = 0.5;
p.lc2 = 1;

p.I1 = 0.083;
p.I2 = 0.33;
J = @(x,h,F)(F(repmat(x,size(x'))+diag(h))-F(repmat(x,size(x'))))./h';


%% Calculate zero dynamics phase portrait

q1_ref = pi/2;

% figure(1)
% clf; hold on;
% animate(0, [q1_ref 0 0 0], p)

N = 10;
[q2s, dq2s] = meshgrid(linspace(-2*pi, 2*pi, N), linspace(-20,20, N));

dirs = zeros([size(q2s), 2]);
for i = 1:length(dirs)
    for j = 1:length(dirs)
        terms = calculateSpong([q1_ref; q2s(i,j); 0; dq2s(i,j)], p);
        
        hbar = terms.h2 - terms.d22 * terms.h1/  terms.d12;
        phibar = terms.phi2 - terms.d22 * terms.phi1 / terms.d12; 
    
        u = hbar + phibar;

        vec = dynamics([q1_ref; q2s(i,j); 0; dq2s(i,j)], u, p);
        
        dirs(i,j,1) = vec(2);
        dirs(i,j,2) = vec(4);
    end
end

streamline(q2s, dq2s, dirs(:,:,1), dirs(:,:,2), q2s, dq2s)
xlim([-pi*2,pi*2])
ylim([-20,20])
title("Phase Portrait of the Zero Dynamics for q1=pi/2")
xlabel("q2")
ylabel("dq2")


%% Simulate convergence to zero dynamics with PD control


kp = 29.8;
kd = 8;

x_ref = [pi/2; 0; 0; 0];

controller = @(x) linearizedPD(x,x_ref,p,kp,kd);

x0 = [-pi/2; 0; 0; 0];

sol = ode45(@(t,x)dynamics(x, controller(x), p), linspace(0, 10,1000), x0);

%% animate
for i = 1:length(sol.x)
    figure(1)
    clf; hold on;
    animate(sol.x(i), sol.y(:,i), p)
    drawnow
    pause(0.01)
end

%% Plot trajectories

t = sol.x;
x = sol.y;

figure(2)
subplot(2,1,1)
plot(t, x(1,:), t, x(2,:))

subplot(2,1,2)
plot(t, x(3,:), t, x(4,:))


%% Calculate linear LQR controller

A = J([pi/2; 0; 0; 0], 1e-5*ones(1,4), @(x)dynamics(x,0,p));
B = J(0, 1e-5, @(u)dynamics([pi/2; 0; 0; 0], u, p));

Q = eye(4);
R = 1;

K = lqr(A, B, Q, R);

%% Simulate

controller = @(x) switchingController(x,p,0.4, K, @(x,p)linearizedPD(x, [pi/2; 0; 0; 0], p, kp, kd));
sol = ode45(@(t,x)dynamics(x, controller(x), p), linspace(0, 10,1000), x0);

%% animate
for i = 1:length(sol.x)
    figure(1)
    clf; hold on;
    animate(sol.x(i), sol.y(:,i), p)
    drawnow
    pause(0.01)
end

%% plot
t = sol.x;
x = sol.y;

figure(2)
subplot(2,1,1)
plot(t, x(1,:), t, x(2,:))

subplot(2,1,2)
plot(t, x(3,:), t, x(4,:))


%% Energy Based Swingup
alpha = 0.2;
controller = @(x)energyController(x,p,kp,kd, alpha);

sol = ode45(@(t,x)dynamics(x, controller(x), p), linspace(0, 100,1000), x0+[0.2; 0; 0; 0]);

%% animate
for i = 1:length(sol.x)
    figure(1)
    clf; hold on;
    animate(sol.x(i), sol.y(:,i), p)
    drawnow
    pause(0.01)
end

%% plot
t = sol.x;
x = sol.y;

figure(2)
subplot(2,1,1)
plot(t, x(1,:), t, x(2,:))

subplot(2,1,2)
plot(t, x(3,:), t, x(4,:))

%% Now energy swingup with stabilization
controller = @(x) switchingController(x,p,0.1, K, @(x,p)energyController(x,p,kp,kd, alpha));

sol = ode45(@(t,x)dynamics(x, controller(x), p), linspace(0, 100,1000), x0+[0.2; 0; 0; 0]);

%% animate
for i = 1:length(sol.x)
    figure(1)
    clf; hold on;
    animate(sol.x(i), sol.y(:,i), p)
    drawnow
    pause(0.01)
end

%% plot
t = sol.x;
x = sol.y;

figure(2)
subplot(2,1,1)
plot(t, x(1,:), t, x(2,:))

subplot(2,1,2)
plot(t, x(3,:), t, x(4,:))