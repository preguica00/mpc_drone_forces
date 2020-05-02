function [H,Ts,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta,id_f1, id_f2] = drone_info
    
    H =12;
    Ts = 0.2;
    
    id_x = 1:H;
    id_z = (1:H) + H;
    id_theta= (1:H) + 2*H;
    id_dotx = (1:H) + 3*H;
    id_dotz= (1:H) + 4*H;
    id_dottheta = (1:H) + 5*H;
    id_f1= (1:H) + 6*H;
    id_f2 = (1:H) + 7*H;

end

