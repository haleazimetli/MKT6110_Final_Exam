function plotUasStateVariables(uu)
%
% modified 12/11/2009 - RB

    % process inputs to function
    pn          = uu(1);             % North position (meters)
    pe          = uu(2);             % East position (meters)
    h           = -uu(3);            % altitude (meters)
    u           = uu(4);             % body velocity along x-axis (meters/s)
    v           = uu(5);             % body velocity along y-axis (meters/s)
    w           = uu(6);             % body velocity along z-axis (meters/s)
    phi         = 180/pi*uu(7);      % roll angle (degrees)   
    theta       = 180/pi*uu(8);      % pitch angle (degrees)
    psi         = 180/pi*uu(9);      % yaw angle (degrees)
    p           = 180/pi*uu(10);     % body angular rate along x-axis (degrees/s)
    q           = 180/pi*uu(11);     % body angular rate along y-axis (degrees/s)
    r           = 180/pi*uu(12);     % body angular rate along z-axis (degrees/s)
    pn_c        = uu(13);            % commanded North position (meters)
    pe_c        = uu(14);            % commanded East position (meters)
    h_c         = -uu(15);           % commanded Down position (meters)
    u_c         = uu(16);            % commanded body velocity along x-axis (meters/s)
    v_c         = uu(17);            % commanded body velocity along y-axis (meters/s)
    w_c         = uu(18);            % commanded body velocity along z-axis (meters/s)
    phi_c       = 180/pi*uu(19);     % commanded roll angle (degrees)   
    theta_c     = 180/pi*uu(20);     % commanded pitch angle (degrees)
    psi_c       = 180/pi*uu(21);     % commanded yaw angle (degrees)
    p_c         = 180/pi*uu(22);     % commanded body angular rate along x-axis (degrees/s)
    q_c         = 180/pi*uu(23);     % commanded body angular rate along y-axis (degrees/s)
    r_c         = 180/pi*uu(24);     % commanded body angular rate along z-axis (degrees/s)
    pn_hat      = uu(25);            % estimated North position (meters)
    pe_hat      = uu(26);            % estimated East position (meters)
    h_hat       = -uu(27);           % estimated Down position (meters)
    u_hat       = uu(28);            % estimated body velocity along x-axis (meters/s)
    v_hat       = uu(29);            % estimated body velocity along y-axis (meters/s)
    w_hat       = uu(30);            % estimated body velocity along z-axis (meters/s)
    phi_hat     = 180/pi*uu(31);     % estimated roll angle (degrees)   
    theta_hat   = 180/pi*uu(32);     % estimated pitch angle (degrees)
    psi_hat     = 180/pi*uu(33);     % estimated yaw angle (degrees)
    p_hat       = 180/pi*uu(34);     % estimated body angular rate along x-axis (degrees/s)
    q_hat       = 180/pi*uu(35);     % estimated body angular rate along y-axis (degrees/s)
    r_hat       = 180/pi*uu(36);     % estimated body angular rate along z-axis (degrees/s)
    delta_e     = 180/pi*uu(37);     % elevator angle (degrees)
    delta_a     = 180/pi*uu(38);     % aileron angle (degrees)
    delta_r     = 180/pi*uu(39);     % rudder angle (degrees)
    delta_t     = uu(40);            % throttle setting (unitless)
    t           = uu(41);            % simulation time
    phi_dne = uu(7);
%fprintf("phi raw: %f (rad), deg: %f\n", phi_dne, phi_dne * 180/pi)

    % define persistent variables 
    persistent pn_handle
    persistent pe_handle
    persistent h_handle
    persistent u_handle
    persistent v_handle
    persistent w_handle
    persistent phi_handle
    persistent theta_handle
    persistent psi_handle
    persistent p_handle
    persistent q_handle
    persistent r_handle
    persistent delta_e_handle
    persistent delta_a_handle
    persistent delta_r_handle
    persistent delta_t_handle
    

  % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(2), clf

        subplot(8,2,1)
        hold on
        pn_handle = graph_y_yhat_yd(t, pn, pn_hat, pn_c, 'p_n', []);
        
        subplot(8,2,2)
        hold on
        u_handle = graph_y_yhat_yd(t, u, u_hat, u_c, 'u', []);

        subplot(8,2,3)
        hold on
        pe_handle = graph_y_yhat_yd(t, pe, pe_hat, pe_c, 'p_e', []);

        subplot(8,2,4)
        hold on
        v_handle = graph_y_yhat_yd(t, v, v_hat, v_c, 'v', []);

        subplot(8,2,5)
        hold on
        h_handle = graph_y_yhat_yd(t, h, h_hat, h_c, 'h', []);

        subplot(8,2,6)
        hold on
        w_handle = graph_y_yhat_yd(t, w, w_hat, w_c, 'w', []);

        subplot(8,2,7)
        hold on
        phi_handle = graph_y_yhat_yd(t, phi, phi_hat, phi_c, '\phi', []);
        
        subplot(8,2,8)
        hold on
        p_handle = graph_y_yhat_yd(t, p, p_hat, p_c, 'p', []);
        
        subplot(8,2,9)
        hold on
        theta_handle = graph_y_yhat_yd(t, theta, theta_hat, theta_c, '\theta', []);
        
        subplot(8,2,10)
        hold on
        q_handle = graph_y_yhat_yd(t, q, q_hat, q_c, 'q', []);
        
        subplot(8,2,11)
        hold on
        psi_handle = graph_y_yhat_yd(t, psi, psi_hat, psi_c, '\psi', []);
        
        subplot(8,2,12)
        hold on
        r_handle = graph_y_yhat_yd(t, r, r_hat, r_c, 'r', []);
        
        subplot(8,2,13)
        hold on
        delta_e_handle = graph_y(t, delta_e, '\delta_e', []);
        ylabel('\delta_e')
        
        subplot(8,2,14)
        hold on
        delta_a_handle = graph_y(t, delta_a, '\delta_a', []);
        ylabel('\delta_a')

        subplot(8,2,15)
        hold on
        delta_r_handle = graph_y(t, delta_r, '\delta_r', []);
        ylabel('\delta_r')
        
        subplot(8,2,16)
        hold on
        delta_t_handle = graph_y(t, delta_t, '\delta_t', []);
        ylabel('\delta_t')
        
    % at every other time step, redraw state variables
    else 
       graph_y_yhat_yd(t, pn, pn_hat, pn_c, 'p_n', pn_handle);
       graph_y_yhat_yd(t, pe, pe_hat, pe_c, 'p_e', pe_handle);
       graph_y_yhat_yd(t, h, h_hat, h_c, 'h', h_handle);
       graph_y_yhat_yd(t, u, u_hat, u_c, 'u', u_handle);
       graph_y_yhat_yd(t, v, v_hat, v_c, 'u', v_handle);
       graph_y_yhat_yd(t, w, w_hat, w_c, 'u', w_handle);
       graph_y_yhat_yd(t, phi, phi_hat, phi_c, '\phi', phi_handle);
       graph_y_yhat_yd(t, theta, theta_hat, theta_c, '\theta', theta_handle);
       graph_y_yhat_yd(t, psi, psi_hat, psi_c, '\psi', psi_handle);
       graph_y_yhat_yd(t, p, p_hat, p_c, 'p', p_handle);
       graph_y_yhat_yd(t, q, q_hat, q_c, 'q', q_handle);
       graph_y_yhat_yd(t, r, r_hat, r_c, 'r', r_handle);
       graph_y(t, delta_e, '\delta_e', delta_e_handle);
       graph_y(t, delta_a, '\delta_a', delta_a_handle);
       graph_y(t, delta_r, '\delta_r', delta_r_handle);
       graph_y(t, delta_t, '\delta_t', delta_t_handle);
    end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y with lable mylabel
function handle = graph_y(t, y, lab, handle)
  
  if isempty(handle),
    handle    = plot(t,y,'b');
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);    
  else
    set(handle,'Xdata',[get(handle,'Xdata'),t]);
    set(handle,'Ydata',[get(handle,'Ydata'),y]);
    %drawnow
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y and yd with lable mylabel
function handle = graph_y_yd(t, y, yd, lab, handle)
  
  if isempty(handle),
    handle(1)    = plot(t,y,'b');
    handle(2)    = plot(t,yd,'g--');
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
    %drawnow
  end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the variable y in blue, its estimated value yhat in green, and its 
% desired value yd in red, lab is the label on the graph
function handle = graph_y_yhat_yd(t, y, yhat, yd, lab, handle)
  
  if isempty(handle),
    handle(1)   = plot(t,y,'b');
    handle(2)   = plot(t,yhat,'g--');
    handle(3)   = plot(t,yd,'r-.');
    ylabel(lab)
    set(get(gca,'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);
    set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
    set(handle(3),'Ydata',[get(handle(3),'Ydata'),yd]);     
    %drawnow
  end

%
%=============================================================================
% sat
% saturates the input between high and low
%=============================================================================
%
function out=sat(in, low, high)

  if in < low,
      out = low;
  elseif in > high,
      out = high;
  else
      out = in;
  end

% end sat  


