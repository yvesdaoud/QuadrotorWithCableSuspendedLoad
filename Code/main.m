%% Parameter Initializations

clear
close all

%quadrotor
Ixx = 0.005;
Iyy = 0.007;
Izz = 0.006;
JQ = [Ixx,0,0 ; 0,Iyy,0 ; 0,0,Izz]; % inertia matrix of quadrotor

mQ = 0.665; % mass of quadrotor
g = 9.80665; % gravity

%load
mL = 0.050; % mass of load
l = 0.3; % length of cable


%% Initial Conditions

%load
xL_0 = [-0.01; 0; -0.3]; % initial load position at origin
if (norm(xL_0) ~= 0)
    xL_0 = l*xL_0 / norm(xL_0);
end
vL_0 = [0; 0; 0]; % intial load velocity starts from rest
w_0 = [0; 0; 0]; % initial load angular velocity

%quadrotor
xQ_0 = [0; 0; 0]; % initial q uadrotor position "l" above load
vQ_0 = [0; 0; 0]; % initial quadrotor velocity starts from rest
R_0 = eye(3); % initial attitude of the quadrotor at identity
Omega_0 = [0; 0; 0]; % initial quadrotor angular velocity

q_0 = (xL_0 - xQ_0) / l; % unit vector from quadrotor to load
d_q_0 = [0; 0; 0];

%useful for controller
q_d_m1 = [0; 0; 0];
q_d_m2 = [0; 0; 0];
R_d_m1 = eye(3);
Omega_d_m1 = [0; 0; 0];


%% Simulation Options

time_current = 0;
t_max = 5;


%% Controller

%PD
kx = diag( [ 8, 8, 6 ] );
kv = diag( [ 4, 4, 3 ] );
kI = diag( [ 0, 0, 0 ] );

%PID
% kx = diag( [ 8, 8, 50 ] );
% kv = diag( [ 4, 4, 10 ] );
% kI = diag( [ 4, 4, 0 ] );

kR = 10;
kOmega = 0.5;

kq = 2;
kw = 0.5;

%frequencies of the 2 controllers
f_attitude = 500; % attitude controller frequency 1kHz
time_step_attitude = 1/f_attitude; % time step between each iteration for attitude controller
f_position = 250; % position controller frequency 500Hz
time_step_pos = 1/f_position; % time step between each iteration for position controller


%% Trajectory Generation

%desired step trajectory
b1 = [0; 1; 0]; % desired yaw = 0 degrees ZYX Euler Convention

xd = [1; 1; 1]; % desired load position
vd = [0; 0; 0]; % desired load velocity
d2_xd = [0; 0; 0]; % desired load acceleration


%desired trajectory to track
enable_trajectory_generation = true;

%sinusoid z and linear x
all_times = linspace(time_current, t_max, t_max/time_step_pos);
freq = 3;
ampl = 0.2;
x_max = 5;
xd_generated_x = all_times * x_max / t_max;
xd_generated_y = zeros(1, t_max/time_step_pos);
xd_generated_z = ampl * sin( freq * all_times );
xd_generated = [xd_generated_x', xd_generated_y', xd_generated_z'];
vd_generated = zeros(t_max/time_step_pos,3);


%% Plotting Initialization

dt = time_step_attitude;

times = zeros(t_max/dt, 1);
xLs = zeros(t_max/dt, 3);
vLs = zeros(t_max/dt, 3);
xQs = zeros(t_max/dt, 3);
vQs = zeros(t_max/dt, 3);
Rs = zeros(t_max/dt, 9);
phis = zeros(t_max/dt, 1);
thetas = zeros(t_max/dt, 1);
psis = zeros(t_max/dt, 1);
norm_deltas = zeros(t_max/dt, 1);
taut_or_nontauts = zeros(t_max/dt, 1);
fs = zeros(t_max/dt, 1);
Ms = zeros(t_max/dt, 3);


%% Plant Simulation

index = 1;
internal_index = 1;
time_prev = time_current;

while (time_current < t_max)
    
    if (t_max - time_current < dt*1e-2)
        break;
    end
    
    %% Position Controller
    
    if (index==1 || ( (time_current-time_prev) >= time_step_pos-0.00000001 ) ) % runs at f_position
        
        %Trajectory generation
        if (enable_trajectory_generation == true)
            xd = xd_generated(internal_index, :)';
            vd = vd_generated(internal_index, :)';
        end
        
        
        %Integral controller
        if (index > 1)
            
            temp1 = xLs(1:2:index-1,:);
            
            if (enable_trajectory_generation == true)
                temp2 = xd_generated(1:internal_index-1,:);
            else
                temp2 = xd'.*ones(size(temp1,1),3);
            end
            
            e_x = temp1 - temp2;
            integral_xL = trapz( all_times(1:internal_index-1), e_x, 1 )';
        
        else  
            integral_xL = [0;0;0];
        end
        
        [q_d, Fn] = load_position_controller(xL_0, xd, vL_0, vd, d2_xd, integral_xL,...
                                                                mQ, mL, g, l, q_0, d_q_0, kx, kv, kI);
    
        [F, f] = load_attitude_controller(mQ, l, R_0, Fn, q_0, d_q_0, q_d, q_d_m1, q_d_m2, dt, kq, kw);
        
        if (time_current >= 2*dt)   
            q_d_m2 = q_d_m1;
            q_d_m1 = q_d;
        elseif (time_current >= dt)
            q_d_m1 = q_d;
        end
        
        time_prev = time_current;
        internal_index = internal_index + 1;
        
    end
    
    
    %% Attitude Controller
    
    [M,R_d,Omega_d] = quadrotor_attitude_controller(JQ, F, b1, R_0, R_d_m1, dt, Omega_0, Omega_d_m1, kR, kOmega);
    
    if (time_current >= dt)
       R_d_m1 = R_d;
       Omega_d_m1 = Omega_d;
    end
    

    %% Dynamics

    delta = xQ_0 - xL_0;
    
    if ( norm(delta) < l-0.01 ) %nontaut dynamics
        
        disp("nontaut dynamics")
        disp(norm(delta))
        taut_or_nontaut = 1;
        
        [xL, xQ, R, vL, vQ, Omega] = nontaut_dynamics(mL, mQ, g, JQ, vL_0, xL_0, vQ_0,...
                                                            xQ_0, f, Omega_0, R_0, M, time_current, dt);
        [q, d_q, w] = nontaut_2_taut(xL,vL,xQ,vQ,l,q_0,dt);
        
    elseif ( norm(delta) >= l-0.01 && norm(delta) < l+0.01 ) %taut dynamics
        
        taut_or_nontaut = 0;
        
        [xL, q, d_q, R, vL, w, Omega] = taut_dynamics(g, mL, mQ, l, JQ, Omega_0, R_0, M, q_0, d_q_0,...
                                                                    f, w_0, xL_0, vL_0, time_current, dt);
        [xQ, vQ] = taut_2_nontaut(xL,q,vL,w,l);
        
    else
        
        taut_or_nontaut = -1;
        warning("at "+index+" delta value bigger "+norm(delta))
        
        q = q / norm(q);
        d_q = [0; 0; 0];
        xQ = xL - l * q;
        
    end
    
    %rotation matrix to euler angles
    [phi_0, theta_0, psi_0] = rot_2_eul(R_0);
    
    
    %% Store and update variables
    
    times(index,1) = time_current;
    xLs(index,:) = xL_0;
    vLs(index,:) = vL_0;
    xQs(index,:) = xQ_0;
    vQs(index,:) = vQ_0;
    Rs(index,:) = reshape(R_0,9,1);
    phis(index,1) = phi_0;
    thetas(index,1) = theta_0;
    psis(index,1) = psi_0;
    norm_deltas(index,1) = norm(delta);
    taut_or_nontauts(index,1) = taut_or_nontaut;
    fs(index,1) = f;
    Ms(index,:) = M;
    
    %update variables
    xL_0 = xL;
    vL_0 = vL;
    xQ_0 = xQ;
    vQ_0 = vQ;
    q_0 = q;
    d_q_0 = d_q;
    R_0 = R;
    w_0 = w;
    Omega_0 = Omega;
    
    %update time
    time_current = time_current + dt;
    index = index + 1;
    
    
end


%% Plotting

figure(1)

subplot(2,1,1)
plot(times,xQs)
grid on
xlabel("t")
ylabel("xQ")
legend("xQ(1)","xQ(2)","xQ(3)")

subplot(2,1,2)
plot(times,xLs)
hold on
if (enable_trajectory_generation == true)
    plot(all_times,xd_generated,'--k')
else
    plot([times(1),times(end)],[xd,xd],'k--')
end
grid on
xlabel("t")
ylabel("xL")
legend("xL(1)","xL(2)","xL(3)")


figure(2)

subplot(2,1,1)
plot(times,norm_deltas)
grid on
xlabel("t")
ylabel("norm(delta)")

subplot(2,1,2)
scatter(times,taut_or_nontauts)
grid on
xlabel("t")
ylabel("taut or nontaut")


figure(3)

subplot(3,1,1)
plot(times,phis)
grid on
xlabel("t")
ylabel("phi")

subplot(3,1,2)
plot(times,thetas)
grid on
xlabel("t")
ylabel("theta")

subplot(3,1,3)
plot(times,psis)
grid on
xlabel("t")
ylabel("psi")


figure(4)

subplot(4,1,1)
plot(times,fs)
grid on
xlabel("t")
ylabel("f")

subplot(4,1,2)
plot(times,Ms(:,1))
grid on
xlabel("t")
ylabel("M(1)")

subplot(4,1,3)
plot(times,Ms(:,2))
grid on
xlabel("t")
ylabel("M(2)")

subplot(4,1,4)
plot(times,Ms(:,3))
grid on
xlabel("t")
ylabel("M(3)")


figure(5)

plot3(xQs(:,1),xQs(:,2),xQs(:,3),'r')
grid on
hold on
plot3(xLs(:,1),xLs(:,2),xLs(:,3),'b')
if (enable_trajectory_generation == true)
    plot3(xd_generated(:,1),xd_generated(:,2),xd_generated(:,3),'--k')
else
    plot3(xd(1),xd(2),xd(3),'xk','MarkerSize',10)
end
xlabel("x")
ylabel("y")
zlabel("z")
legend("quadrotor trajectory","load trajectory","desired load trajectory")


figure(6)

temp1 = xLs(1:2:end,:);        
if (enable_trajectory_generation == true)
    temp2 = xd_generated(1:end,:);
else
    temp2 = xd'.*ones(size(temp1,1),3);
end
e_x = temp1 - temp2;
plot(all_times,e_x)
grid on
xlabel("t")
ylabel("error")
legend("err xL(1)","err xL(2)","err xL(3)")


%3D Animation Frames -----------
figure(7)

time_step_static = 367;

x_static = xQs(1:time_step_static:end,1);
y_static = xQs(1:time_step_static:end,2);
z_static = xQs(1:time_step_static:end,3);
x1_static = xLs(1:time_step_static:end,1);
y1_static = xLs(1:time_step_static:end,2);
z1_static = xLs(1:time_step_static:end,3);
R_static = Rs(1:time_step_static:end,:);

min_x = min( min(xQs(:,1)), min(xLs(:,1)) );
min_y = min( min(xQs(:,2)), min(xLs(:,2)) );
min_z = min( min(xQs(:,3)), min(xLs(:,3)) );
max_x = max( max(xQs(:,1)), max(xLs(:,1)) );
max_y = max( max(xQs(:,2)), max(xLs(:,2)) );
max_z = max( max(xQs(:,3)), max(xLs(:,3)) );

if (enable_trajectory_generation == true)
    plot3(xd_generated(:,1),xd_generated(:,2),xd_generated(:,3),'--k')
else
    plot3(xd(1),xd(2),xd(3),'xk','MarkerSize',10)
end
hold on
plot3(xQs(:,1),xQs(:,2),xQs(:,3),'r')
plot3(xLs(:,1),xLs(:,2),xLs(:,3),'b')

grid on
xlim([min_x-0.2, max_x+0.2]);
ylim([min_y-0.2, max_y+0.2]);
zlim([min_z-0.7, max_z+0.7]);
xlabel("x")
ylabel("y")
zlabel("z")

scatter3(x1_static,y1_static,z1_static,100,'r.')

h = get(gca,'DataAspectRatio');
wingspan_x = 0.8 / h(2);
wingspan_y = 0.8 / h(1);
props = 7;

for idx_static = 1:length(x_static)
    
    x_dr = x_static(idx_static);
    y_dr = y_static(idx_static);
    z_dr = z_static(idx_static);
    
    %plot quadrotor without rotation
%     plot3([x_dr-wingspan_x/2,x_dr+wingspan_x/2],[y_dr,y_dr],[z_dr,z_dr],'b','LineWidth',2)
%     plot3([x_dr,x_dr],[y_dr-wingspan_y/2,y_dr+wingspan_y/2],[z_dr,z_dr],'b','LineWidth',2)
%     plot3(x_dr-wingspan_x/2,y_dr,z_dr,'k.','MarkerSize',props)
%     plot3(x_dr+wingspan_x/2,y_dr,z_dr,'k.','MarkerSize',props)
%     plot3(x_dr,y_dr-wingspan_y/2,z_dr,'k.','MarkerSize',props)
%     plot3(x_dr,y_dr+wingspan_y/2,z_dr,'k.','MarkerSize',props)
    
    %plot quadrotor with rotation
    R_dr = R_static(idx_static,:);
    R_dr = reshape(R_dr,3,3);
    p = [x_dr; y_dr; z_dr];
    g_st = [R, p ; 0,0,0,1];
    
    X_dr_l1 = g_st * [-wingspan_x/2 ; 0 ; 0 ; 1];
    X_dr_r1 = g_st * [+wingspan_x/2 ; 0 ; 0 ; 1];
    X_dr_l2 = g_st * [0 ; -wingspan_y/2 ; 0 ; 1];
    X_dr_r2 = g_st * [0 ; +wingspan_y/2 ; 0 ; 1];
    
    plot3([X_dr_l1(1),X_dr_r1(1)],[X_dr_l1(2),X_dr_r1(2)],[X_dr_l1(3),X_dr_r1(3)],'b','LineWidth',2)
    plot3([X_dr_l2(1),X_dr_r2(1)],[X_dr_l2(2),X_dr_r2(2)],[X_dr_l2(3),X_dr_r2(3)],'b','LineWidth',2)
    
    
    %plot cable
    plot3([x_static(idx_static), x1_static(idx_static)],[y_static(idx_static), y1_static(idx_static)],...
                                                            [z_static(idx_static), z1_static(idx_static)], '-k')
    
end


%SAVE figures
savefig = false;
if (savefig == true)
    saveas(figure(1),'Zresults/xL xQ.png')
    saveas(figure(2),'Zresults/taut vs nontaut.png')
    saveas(figure(3),'Zresults/angles.png')
    saveas(figure(4),'Zresults/forces.png')
    saveas(figure(5),'Zresults/trajectory.png')
    saveas(figure(6),'Zresults/error.png')
    saveas(figure(7),'Zresults/3D animation.png')
end


%% 3D Animation Movie

figure(8)

time_step_dynamic = 5;

x_dynamic = xQs(1:time_step_dynamic:end,1);
y_dynamic = xQs(1:time_step_dynamic:end,2);
z_dynamic = xQs(1:time_step_dynamic:end,3);
x1_dynamic = xLs(1:time_step_dynamic:end,1);
y1_dynamic = xLs(1:time_step_dynamic:end,2);
z1_dynamic = xLs(1:time_step_dynamic:end,3);
R_dynamic = Rs(1:time_step_dynamic:end,:);

%plot desired and actual static lines
if (enable_trajectory_generation == true)
    plot3(xd_generated(:,1),xd_generated(:,2),xd_generated(:,3),'--k')
else
    plot3(xd(1),xd(2),xd(3),'xk','MarkerSize',10)
end
hold on
plot3(xQs(:,1),xQs(:,2),xQs(:,3),'r')
plot3(xLs(:,1),xLs(:,2),xLs(:,3),'b')

grid on
xlim([min_x-0.4, max_x+0.4]);
ylim([min_y-0.2, max_y+0.2]);
zlim([min_z-0.7, max_z+0.7]);
set(gca,'Color','none');
set(gca,'CLim',[0, 1E-4]);
xlabel("x")
ylabel("y")
zlabel("z")

%initialize live plots
x_dr = x_dynamic(1);
y_dr = y_dynamic(1);
z_dr = z_dynamic(1);

plot_num1_l = plot3([x_dr-wingspan_x/2,x_dr+wingspan_x/2], [y_dr,y_dr], [z_dr,z_dr], 'b','LineWidth',2);
plot_num1_r = plot3([x_dr,x_dr], [y_dr-wingspan_y/2,y_dr+wingspan_y/2], [z_dr,z_dr], 'b','LineWidth',2);
plot_num2 = scatter3(x1_dynamic(1),y1_dynamic(1),z1_dynamic(1),200,'r.');
plot_num3 = plot3([x_dynamic(1), x1_dynamic(1)], [y_dynamic(1), y1_dynamic(1)],...
                                                        [z_dynamic(1), z1_dynamic(1)], '-k','LineWidth',1.5);


for idx_dynamic = 1:length(x_dynamic) 
    
    x_dr = x_dynamic(idx_dynamic);
    y_dr = y_dynamic(idx_dynamic);
    z_dr = z_dynamic(idx_dynamic);

    %plot quadrotor without rotation
%     plot_num1_l.XData = [x_dr-wingspan_x/2,x_dr+wingspan_x/2]; 
%     plot_num1_l.YData = [y_dr,y_dr]; 
%     plot_num1_l.ZData = [z_dr,z_dr];
%     
%     plot_num1_r.XData = [x_dr,x_dr]; 
%     plot_num1_r.YData = [y_dr-wingspan_y/2,y_dr+wingspan_y/2]; 
%     plot_num1_r.ZData = [z_dr,z_dr];
    
    %plot quadrotor with rotation
    R_dr = R_dynamic(idx_dynamic,:);
    R_dr = reshape(R_dr,3,3);
    p = [x_dr; y_dr; z_dr];
    g_st = [R, p ; 0,0,0,1];
    
    X_dr_l1 = g_st * [-wingspan_x/2 ; 0 ; 0 ; 1];
    X_dr_r1 = g_st * [+wingspan_x/2 ; 0 ; 0 ; 1];
    X_dr_l2 = g_st * [0 ; -wingspan_y/2 ; 0 ; 1];
    X_dr_r2 = g_st * [0 ; +wingspan_y/2 ; 0 ; 1];

    plot_num1_l.XData = [X_dr_l1(1),X_dr_r1(1)]; 
    plot_num1_l.YData = [X_dr_l1(2),X_dr_r1(2)]; 
    plot_num1_l.ZData = [X_dr_l1(3),X_dr_r1(3)];
    
    plot_num1_r.XData = [X_dr_l2(1),X_dr_r2(1)]; 
    plot_num1_r.YData = [X_dr_l2(2),X_dr_r2(2)]; 
    plot_num1_r.ZData = [X_dr_l2(3),X_dr_r2(3)];
    
    
    %plot load
    plot_num2.XData = x1_dynamic(idx_dynamic); 
    plot_num2.YData = y1_dynamic(idx_dynamic); 
    plot_num2.ZData = z1_dynamic(idx_dynamic); 
    
    
    %plot cable
    plot_num3.XData = [x_dynamic(idx_dynamic), x1_dynamic(idx_dynamic)]; 
    plot_num3.YData = [y_dynamic(idx_dynamic), y1_dynamic(idx_dynamic)]; 
    plot_num3.ZData = [z_dynamic(idx_dynamic), z1_dynamic(idx_dynamic)];

    pause(0.002)
     
end
  
