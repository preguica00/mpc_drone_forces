function [c, ceq] = discretization(y,x_init,z_init,theta_init,xvelocity_init,zvelocity_init,angvelocity_init,f1_init,f2_init)

[H,Ts,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta,id_f1, id_f2] = drone_info;
[mass,inertia_moment,arm_moment,gravitational_acceleration] = parameters;
% [xobs,yobs,obj_coord, radius] = obstacle;

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

%     ceq(end+1) = x(i+1) - (x(i) + Ts*x_velocity(i));
%     ceq(end+1) = z(i+1) - (z(i) + Ts*z_velocity(i));
%     ceq(end+1) = theta(i+1) - (theta(i) + Ts*angular_velocity(i));
%     
%     ceq(end+1) = x_velocity(i+1) - (x_velocity(i) + (Ts/mass)*sin(theta(i))*mode_common(i));
%     ceq(end+1) = z_velocity(i+1) -(z_velocity(i)+ Ts*(gravitational_acceleration-((1/mass)*cos(theta(i))*mode_common(i))));
%     ceq(end+1) = angular_velocity(i+1) - (angular_velocity(i) + Ts*(arm_moment/inertia_moment)*mode_diff(i));
%     ceq(end+1) = y(i+1) - (simulate_timestep(current_state, [u1(k);u2(k)]);

% x = y(id_x);
% z = y(id_z);
% c=[];
% c=vecnorm(abs(theta)-pi/9);
c=[];
% c = [sum(radius) - vecnorm([x';z']-obj_coord);(abs(theta)-20)'];
end