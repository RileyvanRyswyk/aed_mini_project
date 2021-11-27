%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SS21 Advanced Electrical Drives - Mini Project
% Plot script
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;

salient_machine = struct(...
    'p', 4, ...
    'Psi_f', 90e-3, ...
    'L_sd', 200e-6, ...
    'L_sq', 500e-6, ...
    'i_smax', 500, ...
    'u_smax', 350/sqrt(3) ...
);

salient_machine_reduced = struct(...
    'p', 4, ...
    'Psi_f', 90e-3, ...
    'L_sd', 200e-6, ...
    'L_sq', 500e-6, ...
    'i_smax', 400, ...
    'u_smax', 350/sqrt(3) ...
);

non_salient_machine = struct(...
    'p', 4, ...
    'Psi_f', 60e-3, ...
    'L_sd', 200e-6, ...
    'L_sq', 200e-6, ...
    'i_smax', 350, ...
    'u_smax', 350/sqrt(3) ...
);

plot_current_trajectory(10, 0, 50000, non_salient_machine);

plot_current_trajectory(70, 0, 50000, salient_machine);

plot_current_trajectory(70, 0, 50000, salient_machine_reduced);

plot_torque_speed(70, 0, 50000, [salient_machine salient_machine_reduced]);


%% Function Name: plot_current_trajectory
%
% Description: plots dq-current trajectory for the machine specified over 
%              the requested speed range, for the requested torque.
%
% Assumptions: none
%
% Inputs:
%     T_e       : electrical torque [Nm]
%     n_min     : minimum speed [rpm]
%     n_max     : maximum speed [rpm]
%     options.p         : number of machine pole pairs
%     options.Psi_f     : Field flux linkage
%     options.L_sd      : Stator inductance (d-axis)
%     options.L_sq      : Stator inductance (q-axis)
%     options.i_smax    : Maximum stator current (amplitude)
%     options.u_smax    : Maximum stator voltage (amplitude)
%
% $Revision: R2020b$ 
%---------------------------------------------------------
function [] = plot_current_trajectory(T_e, n_min, n_max, options)
    
    f = figure;
    f.Position = [200 200 750 750];
    ax = gca; % axis handle
    hold on;
    
    %% Compute constants
    omega_s_min = n_min / 60 * 2 * pi * options.p;
    omega_s_max = n_max / 60 * 2 * pi * options.p;
    
    num_data_pts = 500;
    omega_s = linspace(omega_s_min, omega_s_max, num_data_pts);
    
    % Build machine
    m = machine_control_model(options);
    
    %% Plot contours
    plot_const_torque_lines(m);
    plot_mf_ellipses(m, options.p);
    plot_mtpf_line(m, omega_s);
    plot_mtpa_line(m, num_data_pts);
    plot_ma_circle(m);
    plot_controller_contour(m, omega_s, T_e);
    
    %% Tidy up    
    a = max(m.i_smax, m.i_sc)*1.2;
    axis([-a a/8 -a/12 a]);
    title("Current Trajectory from " + n_min + " to " + n_max + " rpm" ...
           + " for a reference torque of T_e^* = " + T_e ...
           + " and i_s^{max} = " + m.i_smax);
    xlabel('i_{sd} [A]');
    ylabel('i_{sq} [A]');
    ax.YAxisLocation = 'right';
    ax.XGrid = 'on';
    ax.YGrid = 'on';
    l = legend;
    l.Location = 'northwest';
    hold off
    
end

%% Function Name: plot_current_trajectory
%
% Description: plots dq-current trajectory for the machine specified over 
%              the requested speed range, for the requested torque.
%
% Assumptions: none
%
% Inputs:
%     T_e       : electrical torque [Nm]
%     n_min     : minimum speed [rpm]
%     n_max     : maximum speed [rpm]
%     machines  : array of machines
%
% $Revision: R2020b$ 
%---------------------------------------------------------
function [] = plot_torque_speed(T_e, n_min, n_max, machines)
    f = figure;
    f.Position = [200 200 750 750];
    ax = gca; % axis handle
    hold on;
    
   
    %% Plot
    for i = 1:length(machines)
        machine = machines(i);
        num_data_pts = 250;
        omega_s_min = n_min / 60 * 2 * pi * machine.p;
        omega_s_max = n_max / 60 * 2 * pi * machine.p;
        omega_s = linspace(omega_s_min, omega_s_max, num_data_pts);
        speed_scale_factor = 1000;
        
        m = machine_control_model(machine);
        plot_controller_torque_speed(m, omega_s, T_e, ...
            speed_scale_factor, i);
    end
    
    %% Tidy up    
    title("Torque-speed characteristic from " + n_min + " to " + n_max + " rpm" ...
           + " for a reference torque of T_e^* = " + T_e );
    xlabel("n [x" + speed_scale_factor + " rpm]");
    ylabel('T_e [Nm]');
    ax.YAxisLocation = 'right';
    ax.XGrid = 'on';
    ax.YGrid = 'on';
    %ax.XScale = 'log';
    l = legend;
    l.Location = 'northeast';
    hold off
    
end

%% Function Name: plot_const_torque_lines
%
% Description: plots lines of constant torque
%
% Assumptions: none
%
% Inputs:
%     m         : machine
%
% $Revision: R2020b$ 
%---------------------------------------------------------
function [] = plot_const_torque_lines(m)
    
    % determine appropriate torque values for the machine, spread over
    % roughly one decade
    T_e_max = m.T_en_max * m.T_norm;
    d = round(log(T_e_max) / log(10), 1);
    inc = floor(10^(d-1));
    T_const = inc:inc:inc*10;
    
    for i = 1:length(T_const)
        
        % define constant torque line function
        i_sq = @(x) T_const(i) / (1.5 * m.Psi_f) ...
            ./ (1 - 2*m.chi/m.kappa.*x/m.i_smax);
        
        % function plot
        h = fplot(i_sq, m.i_smax*[-1.2 0.2], ...
            'LineStyle', ':', 'Color', 0.7*[1 1 1], ...
            'DisplayName', 'Constant torque lines');
        
        if i ~= 1
            hide_legend_entry(h);
        end   
        
        % contour label
        h = text(20, i_sq(40), T_const(i) + " N/m", 'Color', ...
            0.7*[1 1 1]);
        set (h, 'Clipping', 'on'); % clip text
    end
end

%% Function Name: plot_mf_ellipses
%
% Description: plots lines of constant torque
%
% Assumptions: none
%
% Inputs:
%     m         : machine
%     p         : # pole pairs
%
% $Revision: R2020b$ 
%---------------------------------------------------------
function [] = plot_mf_ellipses(m, p)
    
    % determine appropriate values of omega to plot contours within
    % the area roughly contained by the MA circle
    omega_s_mf = m.u_smax / (m.L_sd * m.i_smax);
    d = floor(log(omega_s_mf) / log(10));
    omega_s_mf = logspace(d, d+1.5, 6);
    
    for i = 1:length(omega_s_mf)
        
        % define mf ellipse for this omega_s
        i_o = m.u_smax / (omega_s_mf(i) * m.L_sd * m.i_smax);
        i_sq = @(x) m.i_smax * real( ...
            sqrt(i_o^2 - (x/m.i_smax+m.kappa).^2))/(2*m.chi+1);
        
        % compute range of valid i_sd for contour
        min_i_sd = m.i_smax*(-i_o - m.kappa);
        max_i_sd = m.i_smax*(i_o - m.kappa);
        
        % function plot
        h = fplot(i_sq, [min_i_sd max_i_sd], ...
            'LineStyle', ':', 'Color', [0.5 1 0.5], ...
            'DisplayName', 'MF Ellipses');
        
        if i ~= 1
            hide_legend_entry(h);
        end        
        
        % contour label
        h = text(max_i_sd-15, i_sq(max_i_sd)-10 - 15*mod(i,2), ...
            round(omega_s_mf(i)/2/pi*60/p) + "rpm", ...
            'Color', [0.5 1 0.5]);
        set (h, 'Clipping', 'on'); % clip text
    end
end

%% Function Name: plot_mtpf_line
%
% Description: plots maximum torque per flux linkage line
%
% Assumptions: none
%
% Inputs:
%     m         : machine 
%     omega_s   : vector of omega_s values over desired range
%
% $Revision: R2020b$ 
%---------------------------------------------------------
function [] = plot_mtpf_line(m, omega_s)
    
    n = length(omega_s); 
    i_sdn_mtpf = zeros(n - 1, 1);
    i_sqn_mtpf = zeros(n - 1, 1);
    for i = 1:n
        m = setReqOperatingPoint(m, 0, omega_s(i));
        [~, i_sdn_mtpf(i), i_sqn_mtpf(i)] = compute_mtpf_op(m); 
    end
    
    % scale
    i_sd_mtpf = i_sdn_mtpf * m.i_smax;
    i_sq_mtpf = i_sqn_mtpf * m.i_smax;
    
    plot(i_sd_mtpf(2:end,1), ...
        i_sq_mtpf(2:end,1), ...
        ':g', ...
        'DisplayName','MTPF' ...
    );

    plot_arrowheads(i_sd_mtpf(2:end,1), i_sq_mtpf(2:end,1), 'g');
    
    %% Plot short-circuit current
    h = plot(-m.i_sc, 0, 'ok');
    text(-m.i_sc, 0,{'i_{sc}'},'VerticalAlignment','top', ...
        'HorizontalAlignment','right');
    hide_legend_entry(h); 
end

%% Function Name: plot_mtpa_line
%
% Description: plots maximum torque per ampere line
%
% Assumptions: none
%
% Inputs:
%     m             : machine 
%     num_data_pts  :
%
% $Revision: R2020b$ 
%---------------------------------------------------------
function [] = plot_mtpa_line(m, num_data_pts)
    
    % plot from zero torque to max torque
    T_en_vec = linspace(0, m.T_en_max * m.T_norm, num_data_pts);
    i_sd_mtpa = zeros(num_data_pts - 1, 1);
    i_sq_mtpa = zeros(num_data_pts - 1, 1);
    for i = 1:length(T_en_vec)
        m = setReqOperatingPoint(m, T_en_vec(i), 0);
        m = compute_mtpa_op(m); 
        [i_sd_mtpa(i), i_sq_mtpa(i)] = get_scaled_currents(m);
    end
    
    plot(i_sd_mtpa(2:end,1), ...
        i_sq_mtpa(2:end,1), ...
        ':r', ...
        'DisplayName','MTPA' ...
    );
end

%% Function Name: plot_ma_circle
%
% Description: plots maximum ampere circle
%
% Assumptions: none
%
% Inputs:
%     m         : machine
%
% $Revision: R2020b$ 
%---------------------------------------------------------
function [] = plot_ma_circle(m)
    
    % plot in third 
    theta = linspace(0, 2*pi);
    i_sd_ma = m.i_smax * cos(theta);
    i_sq_ma = m.i_smax * sin(theta);
        
    plot(i_sd_ma, i_sq_ma, 'r', ...
        'DisplayName','MA' ...
    );
end

%% Function Name: plot_controller_contour
%
% Description: plots the controller current contour
%
% Assumptions: none
%
% Inputs:
%     m         : machine 
%     omega_s   : vector of omega_s values to plot output for
%     T_e       : 
%
% $Revision: R2020b$ 
%---------------------------------------------------------
function [] = plot_controller_contour(m, omega_s, T_e)
    
    n = length(omega_s);
    i_sd = zeros(n, 1);
    i_sq = zeros(n, 1);
    for i = 1:n
        m = setReqOperatingPoint(m, T_e, omega_s(i));
        m = gen_pm_sync_currents(m); 
        [i_sd(i), i_sq(i)] = get_scaled_currents(m);
    end
    
    plot(i_sd, i_sq, ...
        'DisplayName','Controller', ...
        'color', 'b' ...
    );
    
    plot_arrowheads(i_sd, i_sq, 'b');  
end

%% Function Name: plot_controller_torque_speed
%
% Description: plots the controller current contour
%
% Assumptions: none
%
% Inputs:
%     m             : machine 
%     omega_s       : vector of omega_s values to plot output for
%     T_e           : 
%
% $Revision: R2020b$ 
%---------------------------------------------------------
function [] = plot_controller_torque_speed(m, omega_s, T_e, ...
                speed_scale_factor, machine_num)
    
    colours = ['b', 'g', 'r'];
    rad_to_scaled_rpm = 60 / 2 / pi / m.p / speed_scale_factor;
    T_e_max = m.T_en_max * m.T_norm;
    
    n = length(omega_s);
    T_e_vec = zeros(n, 1);
    for i = 1:n
        m = setReqOperatingPoint(m, T_e, omega_s(i));
        m = gen_pm_sync_currents(m); 
        [i_sd, i_sq] = get_scaled_currents(m);
        T_e_vec(i) = 1.5 * m.Psi_f * i_sq * ...
            (1 - 2*m.chi/m.kappa * i_sd/m.i_smax); % eq 7.27
    end
    
    plot(omega_s * rad_to_scaled_rpm, T_e_vec, ...
        'DisplayName', "T_e^* = " + T_e + " N/m : i_{smax} " + m.i_smax + " A", ...
        'color', colours(machine_num) ...
    );
    
    %% Plot omega_s_a
    h = plot(m.omega_s_a * rad_to_scaled_rpm * [1 1], [0 T_e], ...
        'color', 0.8*[1 1 1]);
    text(m.omega_s_a * rad_to_scaled_rpm, T_e,{'\omega_{s}^{A}'}, ...
        'VerticalAlignment','top', 'HorizontalAlignment','right');
    hide_legend_entry(h);
    
%     %% Plot omega_s_b
%     h = plot(omega_s_b * rad_to_scaled_rpm * [1 1], [0 T_e], ...
%         'color', 0.8*[1 1 1]);
%     text(omega_s_b * rad_to_scaled_rpm, T_e,{'\omega_{s}^{B}'}, ...
%         'VerticalAlignment','top', 'HorizontalAlignment','right');
%     hide_legend_entry(h);

    %% Plot max torque line
    T_e_vec = zeros(n, 1);
    for i = 1:length(omega_s)
        m = setReqOperatingPoint(m, T_e_max, omega_s(i));
        m = gen_pm_sync_currents(m);
        [i_sd, i_sq] = get_scaled_currents(m);
        T_e_vec(i) = 1.5 * m.Psi_f * i_sq * ...
            (1 - 2*m.chi/m.kappa * i_sd/m.i_smax); % eq 7.27
    end
    
    % scale speed by 1000
    plot(omega_s * rad_to_scaled_rpm, T_e_vec, ...
        'DisplayName', "T_e^* = T_e^{max} : i_{smax} = " + m.i_smax + " A", ...
        'color', colours(machine_num) , 'LineStyle', ':' ...
    );
   
    %plot_arrowheads(i_sd, i_sq, 'b');  
end

%% Function Name: plot_arrowheads
%
% Description: plots arrowheads for the specified points.
%
% Assumptions: none
%
% Inputs:
%     x         : x data points
%     y         : y data points
%     colour    : string 
%
% $Revision: R2020b$ 
%---------------------------------------------------------
function [] = plot_arrowheads(X, Y, colour)
    
    num_data_pts = length(X);
    spacing_factor = 5;
    size = 10;
    
    distance = 0;
    for i = 2:num_data_pts
        
        dx = X(i) - X(i-1);
        dy = Y(i) - Y(i-1);
        
        % no direction information, skip point
        if dx == 0 && dy == 0
            continue
        end

        % compute distance to last arrow head
        % if large enough, plot new arrow head
        distance = distance + sqrt(dx^2 + dy^2);
        if distance < size * spacing_factor
            continue
        end
        distance = 0;

        % arrowhead has default angle of pi/2
        theta = atan(dy/dx) - pi/2;
        
        % adjust for quadrant
        if dx < 0 && dy <= 0
            theta = theta + pi; % third quadrant
        elseif dx < 0 && dy > 0
            theta = theta - pi;
        end
        
        % rotation matrix
        R = [cos(theta) -sin(theta);
             sin(theta)  cos(theta)];

        % arrowhead definition
        arrowhead = [-size/4    0   size/4  ;
                     -size      0   -size   ];

        rotated_arrows = R*arrowhead;

        h = plot(rotated_arrows(1,:) + X(i), ...
            rotated_arrows(2,:) + Y(i), ...
            'color', colour ...
            ); 

        hide_legend_entry(h);
        
    end
end


%% Function Name: hide_legend_entry
%
% Description: plots arrowheads for the specified points.
%
% Assumptions: none
%
% Inputs:
%     handle    : plot handle
%
% $Revision: R2020b$ 
%---------------------------------------------------------
function [] = hide_legend_entry(handle)
    % hide from legend
    set( get( get( handle, 'Annotation'), 'LegendInformation' ), ...
        'IconDisplayStyle', 'off' );  
end