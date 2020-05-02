function dydt = quadcopter_ode(t,y)

[H,Ts,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta,id_f1, id_f2]  = drone_info;
[mass,inertia_moment,arm_moment,gravitational_acceleration] = parameters;
%parameters
tau = 5*10^(-3);
fmax=mass*gravitational_acceleration;
alpha = 1/fmax;
%% Unpack the state and input vectors

position_x= y(1);
position_z= y(2);
pitch= y(3);
velocity_x= y(4);
velocity_z= y(5);
velocity_pitch= y(6);
f1 = y(7);
f2 = y(8);
    
%u1=f1-f1;
%u2=f1+f2;
diff_mode  = f1-f2;
common_mode = f1+f2;



%%Equations of motion
x_acceleration = (1/mass)*sin(pitch)* common_mode;
z_acceleration = gravitational_acceleration -(1/mass)*cos(pitch)* common_mode;
pitch_acceleration = (arm_moment/inertia_moment)*diff_mode;

%%Equations of thrust forces
throttle_ref = alpha*[diff_mode common_mode]';
f_velocity = (-[f1 f2]'/tau) + throttle_ref/tau;

%% 

dydt=[velocity_x;velocity_z;velocity_pitch;x_acceleration;z_acceleration;pitch_acceleration;f_velocity];

end