%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SS21 Advanced Electrical Drives - Mini Project
% 
% machine_control_model class file 
% Primary goal is to generate the required direct and quadrature axis 
% currents to produced the desired torque at the specified operating speed
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage / Hinweis
%
% Paste the code in this comment block in the command window, or into 
% another script (in the same folder) to run it. 
%
% % Machine Parameters
% salient_machine = struct(...
%     'p', 4, ...
%     'Psi_f', 90e-3, ...
%     'L_sd', 200e-6, ...
%     'L_sq', 500e-6, ...
%     'i_smax', 500, ...
%     'u_smax', 350/sqrt(3) ...
% );
%
% T_e = 70;
% omega_s = 1000;
% 
% % Create machine model
% m = machine_control_model(salient_machine);
% 
% % Set desired torque and current speed
% m = setReqOperatingPoint(m, T_e, omega_s);
% 
% % compute required machine currents
% m = gen_pm_sync_currents(m); 
% 
% % output scaled currents
% [i_sd, i_sq] = get_scaled_currents(m);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Class Name: machine_control_model
%
% Description: generates reference currents i_sd and i_sq for a generic 
%              PM synchronous machine at the specified operating point
%              using the theory of field-oriented control as derived in
%              Advanced Electrical Drives - Analysis Modeling Control (2nd)
%              De Doncker, Pulle, Veltman. 
%
% Assumptions: positive saliency, quasi-steady-state operation
%
% $Revision: R2020b$ 
%---------------------------------------------------------
classdef machine_control_model
    
    properties
        % Input Parameters
        Psi_f   {mustBeNonnegative}     % Field flux linkage [Vs]
        L_sd    {mustBeNonnegative}     % Stator inductance (d-axis) [H]
        L_sq    {mustBeNonnegative}     % Stator inductance (q-axis) [H]
        i_smax  {mustBeNonnegative}     % Max stator current [A]
        u_smax  {mustBeNonnegative}     % Max stator voltage [V]
        p       {mustBeInteger}         % pole pair number
        
        % Computed Parameters
        chi     {mustBeReal}            % Machine saliency 
        i_sc    {mustBeNonnegative}     % Short circuit current [A]
        kappa   {mustBeNonnegative}     % Normalized short circuit current  
        T_norm  {mustBeNonnegative}     % normalization constant for torque [Nm]
        T_en_max {mustBeNonnegative}    % maximum normalize electrical torque
        
        % Operating Point Params
        T_e     {mustBeNonnegative}     % requested electrical torque [Nm]
        T_en    {mustBeNonnegative}     % normalize requested electrical torque
        omega_s {mustBeNonnegative}     % operating speed [rad/s]
        i_o     {mustBeNonnegative}     % Maximum flux linkage current radius [A]
        
        % Transition speed between MTPA and basic field weaking operation
        omega_s_a {mustBeNonnegative}     % operating speed [rad/s]
        
        % Transition speed between basic field weaking operation and field
        % weakening operation
        omega_s_b {mustBeNonnegative}     % operating speed [rad/s]
        
        i_sdn   {mustBeReal}            % normalized d-axis current
        i_sqn   {mustBeReal}            % normalized q-axis current
    end
    
    methods
        
        %% Function Name: machine_control_model
        %
        % Description: class constructor. 
        %
        % Inputs:
        %     options.Psi_f     : Field flux linkage
        %     options.L_sd      : Stator inductance (d-axis)
        %     options.L_sq      : Stator inductance (q-axis)
        %     options.i_smax    : Maximum stator current (amplitude)
        %     options.u_smax    : Maximum stator voltage (amplitude)
        %---------------------------------------------------------
        function m = machine_control_model(options)
            
            % default parameters
            if nargin==0
                fprintf("Input params not provided, using default values");
                options = struct(...
                    'p', 4, ...
                    'Psi_f', 90e-3, ...
                    'L_sd', 200e-6, ...
                    'L_sq', 500e-6, ...
                    'i_smax', 500, ...
                    'u_smax', 350/sqrt(3) ...
                );
            end
            
            m.Psi_f = options.Psi_f;
            m.L_sd = options.L_sd;
            m.L_sq = options.L_sq;
            m.i_smax = options.i_smax;
            m.u_smax = options.u_smax;
            m.p = options.p;
            m.chi = (m.L_sq - m.L_sd) / (2 * m.L_sd);   % (equation 6.18)
            m.i_sc = m.Psi_f / m.L_sd;
            m.kappa = m.i_sc / m.i_smax;                % (equation 7.8a)
            m.T_norm = 1.5 * m.Psi_f * m.i_smax;        % (equation 7.27)
            m = update_max_norm_torque(m);
            
            if nargin==0
                m = setReqOperatingPoint(m, 70, 1000);
                m = gen_pm_sync_currents(m);
            end
        end
        
        %% Function Name: update_max_norm_torque
        %
        % Description: computes maximum possible torque for the machine 
        %              during base speed operation.
        %---------------------------------------------------------
        function m = update_max_norm_torque(m)
            % combine equations 7.27, 7.29a and 7.29b to solve for 
            % max torque under the constraint i_sn = 1

            if m.chi == 0 
                i_sdn_max_T = 0; % non-salient machine
            else
                a = m.kappa/8/m.chi;
                i_sdn_max_T = a - sqrt(a^2 + 1/2); % eq 7.29a
            end
            i_sqn_max_T = sqrt(1 - i_sdn_max_T^2); % eq 7.29b
            m.T_en_max = i_sqn_max_T*(1 - 2*m.chi/m.kappa * i_sdn_max_T); % eq 7.27
        end
        
        %% Function Name: setReqOperatingPoint
        %
        % Description: set the requested machine operating point. 
        %
        % Inputs:
        %     m         : machine
        %     T_e       : requested electrical torque
        %     omega_s   : operating speed
        %---------------------------------------------------------
        function m = setReqOperatingPoint(m, T_e, omega_s)
            m.T_e = T_e;
            m.omega_s = omega_s;
            
            % Compute operating point dependent properties
            m.T_en = T_e / m.T_norm;
            m.i_o = m.u_smax / (omega_s * m.L_sd * m.i_smax); % (equation 7.32)
        end
        
        %% Function Name: gen_pm_sync_currents
        %
        % Description: generates reference currents i_sd and i_sq
        %              for the requested operating point set
        %
        %---------------------------------------------------------
        function m = gen_pm_sync_currents(m)

            %% Determine operating point
            % Compute MTPA Point (base speed operation)
            m = compute_mtpa_op(m);

            % Compute transition speed between MTPA and basic field weaking operation
            % Solves equation 7.32 for omega_s using operating point A (MTPA)
            m.omega_s_a = m.u_smax / m.L_sd / m.i_smax ...
                        / sqrt((m.i_sdn + m.kappa)^2 + m.i_sqn^2 * (2*m.chi + 1)^2);

            % Compute transition speed between basic field weaking operation  
            % and field weaking operation
            if m.T_en > m.T_en_max
                m.omega_s_b = m.omega_s_a;
            else 
                m = compute_omega_s_b(m); 
            end

            % Compute basic field weakening operating point (MF), if beyond 
            % base speed operation, but only up to the intersection of the 
            % maximum ampere circle (omega_s_b)
            if m.omega_s > m.omega_s_a && m.omega_s <= m.omega_s_b
                m = compute_mf_op(m);
            end

            % Check Maximum Ampere (MA) limit, if it is exceeded calculate 
            % operating point along MA limit, while respecting MF limits.
            % Start of torque reduction [field weakening region]
            if m.i_sdn^2 + m.i_sqn^2 > 1 || m.omega_s > m.omega_s_b
                m = compute_ma_op(m);
            end

            % Check for operation beyond MTPF line, if the MTPF line has 
            % been exceeded, then compute operating point on MTPF line
            % [field weakening region]
            m = compute_mtpf_op(m);

            % Sanity check
            if m.i_sdn^2 + m.i_sqn^2 > 1 + eps
                error('current limit exceeded :(');
            end
        end
        
        %% Function Name: compute_mtpa_op
        %
        % Description: computes maximum torque per ampere operating point  
        %              for a PM synchronous machine at the specified speed 
        %              and desired torque.  
        %
        %---------------------------------------------------------
        function m = compute_mtpa_op(m)

            % check for non-salient case to avoid numerical issues with chi == 0
            if m.chi == 0
                m.i_sdn = 0;
                m.i_sqn = min(m.T_en, m.T_en_max); % eq 7.2 * 
                return
            end
            
            % constants used in following equations
            a = m.kappa / 8 / m.chi;
            b = 2 * m.chi / m.kappa;
            
            % combine equations 7.29a and 7.30 to solve for i_sdn
            % eq 7.29a i_sdn = (kappa/8/chi) - sqrt((kappa/8/chi)^2 + (i_sn^2)/2)  
            % eq 7.30 i_sn^2 = i_sdn^2 + (T_en / (1 - 2*chi/kappa*i_sdn))^2
            % must be arranged to be of form f(x) = 0
            function F = equations(x)
                % x(1) = i_sdn, x(2) = i_sn

                F(1) = a - sqrt(a^2 + (x(2)^2)/2) - x(1);
                F(2) = x(1)^2 + (m.T_en / (1 - b*x(1)))^2 - x(2)^2;
            end

            if m.T_en >= m.T_en_max
                % i_sn = 1
                m.i_sdn = a - sqrt(a^2 + 1/2); % eq 7.29
                m.i_sqn = sqrt(1 - (m.i_sdn)^2);

            else
                % faster but less robust near maxima
                % isolate i_sn^2 from 7.29a and plug into eq 7.30
                % must be arranged to be of form f(x) = 0
                eq = @(x) (m.T_en / (1 - b*x))^2 + x^2 ...
                          - 2*(x - a)^2 + 2*(a)^2;
                x0 = 0; % starting point
                %options = optimset('Display','iter'); % show iterations
                roots = fzero(eq, x0);

                % extract and compute operating point
                if ~all(size(roots))
                    % fzero failed, use more robust, but slower fsolve
                    options = optimoptions('fsolve','Display','off');
                    x0 = [0,0];
                    [x,~,exitflag,~] = fsolve(@equations, x0, options);
                    if exitflag ~= 1
                        error('Invalid operating point during MTPA calculation');
                    end
                    m.i_sdn = x(1);
                    m.i_sqn = m.T_en / (1 - b*m.i_sdn); % eq 7.27
                end
                m.i_sdn = roots(1);
                m.i_sqn = m.T_en / (1 - b*m.i_sdn); % eq 7.27   
                
            end
        end
        
        %% Function Name: compute_omega_s_b
        %
        % Description: computes transition speed from MF operation to MA 
        %              operation for salient machines. For non-salient 
        %              machines with the MTPF line within the MA circle 
        %              (k < 1), return the intersection point with the  
        %              MTPF line
        %
        %---------------------------------------------------------
        function m = compute_omega_s_b(m)

            % non-salient, with MTPF inside MA circle
            % dervied from equations 7.11, 7.13, 7.14a, b 
            if m.chi == 0 && m.kappa < 1
              %  omega_s_a = u_smax / L_sd / i_smax / sqrt(kappa^2 + T_en^2);
                m.omega_s_b = m.omega_s_a * sqrt(1 + (m.kappa/m.T_en)^2);
                return 
            end

            % Solve for op at MA circle and T_en intersection, nearest to i_sc
            % eq 7.27 T_en = i_sqn * (1 - 2 * chi/kappa * i_sdn)   
            % eq 7.28 i_sqn^2 + i_sdn^2 = i_sn^2  
            % under constraint i_sn^2 = 1
            function F = equations(x)
                % x(1) = i_sdn, x(2) = i_sqn
                F(1) = x(2) * (1 - 2 * m.chi/m.kappa * x(1)) - m.T_en;
                F(2) = x(2)^2 + x(1)^2 - 1;
            end

            % faster but less robust near maxima
            % must be arranged to be of form f(x) = 0
            eq = @(x) x^2 + (m.T_en / (1 - 2*m.chi/m.kappa*x))^2 - 1;
            x0 = -m.kappa; % starting point
            options = optimset('Display','none'); % show iterations
            roots = fzero(eq, x0, options);

            % if required, use slower, but robust solver
            if ~all(size(roots)) || isnan(roots)
                options = optimoptions('fsolve','Display','off');
                x0 = [-m.kappa, 0];
                [x,~,exitflag,~] = fsolve(@equations, x0, options);
                if exitflag ~= 1
                    error('Invalid operating point during omega_s_b calculation');
                end
                i_sdn_b = x(1);
                i_sqn_b = x(2);        
            else
                % compute operating point
                i_sdn_b = roots(1);
                i_sqn_b = m.T_en / (1 - 2*m.chi/m.kappa*i_sdn_b); % eq 7.27
            end
            
            % compute omega_s_b
            m.omega_s_b = m.u_smax / m.L_sd / m.i_smax ...
                        / sqrt((i_sdn_b + m.kappa)^2 + i_sqn_b^2 * (2*m.chi + 1)^2);

        end
        
        %% Function Name: compute_mf_op
        %
        % Description: computes maximum flux linkage operating point for a 
        %              PM synchronous machine at the specified speed and 
        %              desired torque. 
        %
        %---------------------------------------------------------
        function m = compute_mf_op(m)
            % combine equations 7.27 and 7.32 to solve for i_sdn
            % eq 7.27 T_en = i_sqn * (1 - 2 * chi/kappa * i_sdn)  
            % eq 7.32 ((i_sdn + kappa)^2 + (i_sqn)^2 * (2*chi + 1)^2) = i_o^2
            % isolate i_sqn from 7.27 and plug into eq 7.32
            
            T_en_mf = m.T_en;
            if m.T_en > m.T_en_max
                T_en_mf = m.T_en_max;
            end

            % to use more robust fsolve, if required
            function F = equations(x)
                % x(1) = i_sdn, x(2) = i_sn
                F(1) = x(2) * (1 - 2 * m.chi/m.kappa * x(1)) - T_en_mf;
                F(2) = (x(1) + m.kappa)^2 + x(2)^2 * (2*m.chi + 1)^2 - m.i_o^2;
            end

            % must be arranged to be of form f(x) = 0
            eq = @(x) (T_en_mf / (1 - 2*m.chi/m.kappa*x))^2 * (2*m.chi + 1)^2 ...
                      + (x + m.kappa)^2 - m.i_o^2;
            x0 = -m.kappa; % starting point
            options = optimset('Display','none'); % show iterations
            roots = fzero(eq, x0, options);

            % if fzero failed to converge, use slower more robust solver
            if ~all(size(roots)) || isnan(roots)
                options = optimoptions('fsolve','Display','off');
                x0 = [-m.kappa, 0];
                [x,~,exitflag,~] = fsolve(@equations, x0, options);
                if exitflag ~= 1
                    error('Invalid operating point during MF calculation');
                end
                m.i_sdn = x(1);
                m.i_sqn = x(2);
            else
                % extract and compute operating point
                m.i_sdn = roots(1);
                m.i_sqn = T_en_mf / (1 - 2*m.chi/m.kappa*m.i_sdn); % eq 7.27
            end    
        end
        
        %% Function Name: compute_ma_op
        %
        % Description: computes maximum ampere operating point for a 
        %              PM synchronous machine so as to maximize torque for  
        %              the current operating speed.  
        %
        %---------------------------------------------------------
        function m = compute_ma_op(m)
            % combine equations 7.28 and 7.32 to solve for i_sdn
            % eq 7.28 i_sqn^2 + i_sdn^2 = i_sn^2  
            % eq 7.32 ((i_sdn + kappa)^2 + (i_sqn)^2 * (2*chi + 1)^2) = i_o^2
            % under the constraint i_sn = 1

            % adapt formula to avoid division by zero for non-salient
            if m.chi == 0
                m.i_sdn = (m.i_o^2 - m.kappa^2 - 1) / 2 / m.kappa;
            else 
                a = (2*m.chi + 1)^2;
                m.i_sdn = (-m.kappa + sqrt(m.kappa^2 - ...
                    (1 - a)*(m.kappa^2 + a - m.i_o^2))) / (1 - a);
            end

            % Bound i_sdn to operable limits in case MF ellipse leads to solutions
            % beyond MA circle. For these cases the MTPF equations will determine
            % the operating point.
            m.i_sdn = max(-1, min(1, m.i_sdn));
            m.i_sqn = sqrt(1 - m.i_sdn^2);    
        end

        %% Function Name: compute_mtpf_op
        %
        % Description: computes maximum torque per flux linkage op for a 
        %              PM synchronous machine so as to maximize torque for 
        %              the current operating speed.  
        %
        %---------------------------------------------------------
        function [m, i_sdn_mtpf, i_sqn_mtpf] = compute_mtpf_op(m)

            % handle non-salient case to avoid division by zero
            if m.chi == 0
                i_sdn_mtpf = -m.kappa; % eq 7.15a
                i_sqn_mtpf = m.i_o; % eq 7.11, 7.15b, i_o
            else               
                a = (1 + 2*m.chi) * m.kappa/8/m.chi;
                i_sdn_mtpf = -m.kappa + a - sqrt(0.5*m.i_o^2 + a^2); % eq 7.33a
                i_sqn_mtpf = sqrt(m.i_o^2 - (m.kappa + i_sdn_mtpf)^2) ...
                    / (2*m.chi + 1); % eq 7.33b   
            end
            
            % check if the current operating point is set for beyond the 
            % mtpf line 
            if (isempty(m.i_sdn) || i_sdn_mtpf > m.i_sdn) ...
                    && i_sdn_mtpf^2 + i_sqn_mtpf^2 <= 1
                m.i_sdn = i_sdn_mtpf;
                m.i_sqn = i_sqn_mtpf;
            end
        end
        
        %% Function Name: get_scaled_currents
        %
        % Description: returns the scaled operating currents
        %
        %---------------------------------------------------------
        function [i_sd, i_sq] = get_scaled_currents(m)
            %% Scale outputs to non-pu values
            i_sd = m.i_sdn * m.i_smax;
            i_sq = m.i_sqn * m.i_smax;
        end        
    end
end