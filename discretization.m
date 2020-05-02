function [c, ceq] = discretization(y,x_init,z_init,theta_init,xvelocity_init,zvelocity_init,angvelocity_init,f1_init,f2_init)

[H,Ts,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta,id_f1, id_f2] = drone_info;

% Unpacking
x = y(id_x);
z = y(id_z);
theta = y(id_theta);
x_velocity = y(id_dotx);
z_velocity = y(id_dotz);
angular_velocity = y(id_dottheta);
f1 = y(id_f1);
f2 = y(id_f2);


current_state=[x_init; z_init;theta_init;xvelocity_init; zvelocity_init;angvelocity_init;f1_init;f2_init];
state =[current_state, [x,z,theta,x_velocity,z_velocity,angular_velocity, f1, f2]'];

size_state= size(state)

% Run discrete prediction
ceq = [];
for i = 1:H
    ceq = [ceq; (state(:,i+1) - timestep_euler(Ts,state(:,i)))];

end


c=[];
% c=vecnorm(abs(theta)-pi/9);
end