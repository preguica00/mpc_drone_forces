  function  [command, optimum, predicted_trajectory] = ...
    optimizetrajectory(current_state, optimum)


    % Initial conditions
    x_init = current_state(1);
    z_init = current_state(2);
    theta_init = current_state(3);
    xvelocity_init = current_state(4);
    zvelocity_init = current_state(5);
    angvelocity_init = current_state(6);
    f1_init = current_state(7);
    f2_init = current_state(8);
    
  [H,Ts,id_u1,id_u2,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta,id_f1, id_f2] = drone_info;
    
    %initial conditions
    if isempty(optimum)
        optimum = zeros(10*H,1);
    end

opts = optimset('Display','iter','Algorithm','interior-point', 'MaxIter', 30000, 'MaxFunEvals', 30000);



   [optimum, ~] = fmincon(@(y)costfunction(y, H), optimum,[],[],[],[],[],[],@(y)discretization(y,x_init,z_init,theta_init,xvelocity_init,zvelocity_init,angvelocity_init,f1_init,f2_init),opts);
   
   u1_optimum = optimum(id_u1);
   u2_optimum = optimum(id_u2);
   command = [u1_optimum(1), u2_optimum(1)];
   


    x_optimum = optimum(id_x);
    z_optimum = optimum(id_z);
    theta_optimum = optimum(id_theta);
    xvelocity_optimum = optimum(id_dotx);
    zvelocity_optimum = optimum(id_dotz);
    angvelocity_optimum = optimum(id_dottheta);
    f1_optimum=optimum(id_f1);
    f2_optimum=optimum(id_f2);
    
    predicted_trajectory = [x_optimum,z_optimum,theta_optimum,xvelocity_optimum,zvelocity_optimum,angvelocity_optimum,f1_optimum ,f2_optimum];
  end


