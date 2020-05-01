function cost = costfunction(y, H)

    [H,Ts,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta,id_f1, id_f2]  = drone_info;
   [mass,inertia_moment,arm_moment,gravitational_acceleration] = parameters;
    %final positions
    x_final=60;
    z_final=60;
    theta_final=0;
    theta_dotfinal=0;
    common_final = mass*gravitational_acceleration;

    % Unpacking
   
     
    x = y(id_x);
    z = y(id_z);
    theta = y(id_theta);
    x_velocity = y(id_dotx);
    z_velocity = y(id_dotz);
    angular_velocity = y(id_dottheta);
    f1 = y(id_f1);
    f2 = y(id_f2);
    
    mode_diff = f1(:)-f2(:);
    mode_common = f1(:)+f2(:);
   
%         cost = sum(2*(x(:)-x_final).^2+(z(:)-z_final).^2 + (theta(:)).^2 + (x_velocity(:)-dotx_final).^2+ (z_velocity(:)-dotz_final).^2+(angular_velocity(:)-dottheta_final).^2 + (mode_diff(:)).^2+(mode_common(:)).^2);   

    cost = sum(2*(x(:)-x_final).^2+(z(:)-z_final).^2 + (theta(:)- theta_final).^2 + (x_velocity(:)).^2+ (z_velocity(:)).^2+(angular_velocity(:)-theta_dotfinal).^2 + (mode_diff(:)).^2+(mode_common(:)-common_final).^2);   

end