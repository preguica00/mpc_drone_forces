function current_state = simulate_timestep(current_state)
[H,Ts,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta,id_f1, id_f2] = drone_info;
    tspan = [0 Ts];

    my_ode = @(t,y) quadcopter_ode(t,y);
    [~, y] = ode45(my_ode,tspan, current_state);
    current_state = [y(end,1), y(end,2),y(end,3),y(end,4),y(end,5),y(end,6),y(end,7),y(end,8)];
end