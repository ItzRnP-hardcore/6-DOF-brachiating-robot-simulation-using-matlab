function interactive_6dof_robot()
    % --- INITIAL CONFIGURATION & RADII ---
    cyl_radius = [2.5, 3, 3, 3, 3, 2.5]; 
    self_collision_margin = 1.0; 
    
    % --- MODIFIED DH PARAMETERS (Craig's Convention) ---
    dh_table = [ 0,         0,      10;  % Joint 1 
                 0,       -90,      10;  % Joint 2 
                10,         0,       0;  % Joint 3 
                 0,       -90,      10;  % Joint 4 
                 0,        90,      10;  % Joint 5 
                 0,       -90,      10]; % Joint 6 
                 
    link_colors = lines(6);
    
    % --- ENVIRONMENT & STATE ---
    Global_Base_T = eye(4);   
    current_base_joint = 1;   
    
    current_thetas = [0, -30, 0, 0, 0, 0]; 
    planned_path_q = []; 
    global_route_pts = []; 
    ee_trace = [];  
    
    % Generate Hexagonal Grid of Docking Points
    step_size = 30; 
    docking_points = [];
    row_height = step_size * (sqrt(3)/2);
    for row = -3:3
        for col = -3:3
            x = col * step_size; y = row * row_height;
            if mod(row, 2) ~= 0, x = x + (step_size / 2); end
            if abs(x) <= 65 && abs(y) <= 65
                docking_points = [docking_points; x, y, 0];
            end
        end
    end
    
    % --- COMPLEX ENVIRONMENT ---
    env_spheres = [
        35,  15, 20, 7; 
       -40, -40, 20, 8;
        45, -30, 25, 6
    ];
    env_capsules = [
       -25,  20,  0,  -25,  20, 45,  4;  
        15, -25,  0,   15, -25, 30,  5;  
       -10, -10, 30,   20, -10, 30,  3;  
         0,  20, 15,   20,  40, 25,  3   
    ];
    
    obstacles = struct('type',{},'pos',{},'size',{});
    
    % --- CREATE FIGURE AND UI ---
    fig = figure('Color', 'w', 'Name', 'Brachiating Robot Dynamic Planner', 'Units', 'normalized', 'Position', [0.05 0.05 0.9 0.9]);
    ax = axes('Parent', fig, 'Position', [0.35 0.05 0.6, 0.9]);
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal'); view(ax, [45, 30]);
    xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
    
    xlim(ax, [-70 70]); ylim(ax, [-70 70]); zlim(ax, [0 60]);
    patch(ax, [-75 75 75 -75], [-75 -75 75 75], [0 0 0 0], [0.4 0.6 0.4], 'FaceAlpha', 0.8, 'EdgeColor', [0.2 0.4 0.2], 'LineWidth', 2);
    
    % --- UI CONTROLS ---
    uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.01, 0.96, 0.3, 0.03], 'String', 'ROBOT CONTROL PANEL', 'FontSize', 12, 'FontWeight', 'bold', 'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
          
    uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.01, 0.93, 0.15, 0.02], 'String', 'Target (X, Y, Z):', 'BackgroundColor', 'w', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
    edit_targetX = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.16, 0.93, 0.04, 0.025], 'String', '50');
    edit_targetY = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.21, 0.93, 0.04, 0.025], 'String', '40');
    edit_targetZ = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.26, 0.93, 0.04, 0.025], 'String', '20');
    chk_ori = uicontrol('Style', 'checkbox', 'Units', 'normalized', 'Position', [0.01, 0.90, 0.3, 0.025], 'String', ' Force Final Orientation (Rx, Ry, Rz)', 'BackgroundColor', 'w', 'FontWeight', 'bold', 'Value', 1);
    edit_rx = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.16, 0.87, 0.04, 0.025], 'String', '0');
    edit_ry = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.21, 0.87, 0.04, 0.025], 'String', '90');
    edit_rz = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.26, 0.87, 0.04, 0.025], 'String', '0');
    uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.01, 0.83, 0.3, 0.02], 'String', 'RRT* Parameters (Iters | Step° | Goal Bias):', 'BackgroundColor', 'w', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
    edit_rrtIters = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.16, 0.80, 0.04, 0.025], 'String', '1500');
    edit_rrtStep = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.21, 0.80, 0.04, 0.025], 'String', '15');
    edit_goalBias = uicontrol('Style', 'edit', 'Units', 'normalized', 'Position', [0.26, 0.80, 0.04, 0.025], 'String', '0.2');
    status_text = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.01, 0.73, 0.3, 0.05], 'String', 'Status: Waiting for input...', 'BackgroundColor', 'w', 'HorizontalAlignment', 'left', 'ForegroundColor', 'b', 'FontSize', 10);
    btn_plan = uicontrol('Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.01, 0.67, 0.15, 0.04], 'String', 'PLAN SINGLE HOP', 'BackgroundColor', [0.6 0.8 1], 'FontWeight', 'bold', 'Callback', @(~,~) planSingleHopAction());
    btn_exec = uicontrol('Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.17, 0.67, 0.15, 0.04], 'String', 'EXECUTE MOTION', 'BackgroundColor', [0.2 0.8 0.2], 'FontWeight', 'bold', 'Enable', 'off', 'Callback', @(~,~) executeMotionAction());
    uicontrol('Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.01, 0.62, 0.31, 0.04], 'String', 'Dynamic Plan to Target', 'BackgroundColor', [0.2 0.6 1], 'FontWeight', 'bold', 'Callback', @(~,~) planAndExecuteDynamicMultiHop());
    
    uicontrol('Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.01, 0.57, 0.15, 0.04], 'String', 'LATCH & SWAP', 'BackgroundColor', [1 0.6 0.2], 'FontWeight', 'bold', 'Callback', @(~,~) swapBase());
    uicontrol('Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.17, 0.57, 0.15, 0.04], 'String', 'SHOW FREE WS', 'BackgroundColor', [0.8 0.8 0.8], 'FontWeight', 'bold', 'Callback', @(~,~) plotFreeWorkspace());
    
    uicontrol('Style','text','Units','normalized',...
        'Position',[0.01 0.125 0.3 0.025],...
        'String','OBSTACLE CONTROL','FontWeight','bold','BackgroundColor','w', 'HorizontalAlignment', 'left');
    edit_obsX = uicontrol('Style','edit','Units','normalized',...
        'Position',[0.01 0.1 0.05 0.025],'String','0');
    edit_obsY = uicontrol('Style','edit','Units','normalized',...
        'Position',[0.07 0.1 0.05 0.025],'String','0');
    edit_obsZ = uicontrol('Style','edit','Units','normalized',...
        'Position',[0.13 0.1 0.05 0.025],'String','10');
    edit_obsSize = uicontrol('Style','edit','Units','normalized',...
        'Position',[0.19 0.1 0.05 0.025],'String','6');
    popup_type = uicontrol('Style','popupmenu','Units','normalized',...
        'Position',[0.25 0.1 0.07 0.025],...
        'String',{'Sphere','Cylinder','Box'});
    uicontrol('Style','pushbutton','Units','normalized',...
        'Position',[0.01 0.05 0.15 0.04],...
        'String','ADD OBSTACLE','Callback',@addObstacle);
    uicontrol('Style','pushbutton','Units','normalized',...
        'Position',[0.17 0.05 0.15 0.04],...
        'String','CLEAR OBSTACLES','Callback',@clearObstacles);
    
    uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.01, 0.52, 0.3, 0.02], 'String', 'MANUAL JOINT CONTROL', 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
    warn_text = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.15, 0.50, 0.17, 0.02], 'String', '⚠ COLLISION!', 'ForegroundColor', 'r', 'BackgroundColor', 'w', 'FontWeight', 'bold', 'Visible', 'off');
    
    slider_handles = gobjects(1, 6); text_handles = gobjects(1, 6);
    for i = 1:6
        y_pos = 0.46 - (i-1)*0.06;
        text_handles(i) = uicontrol('Style', 'text', 'Units', 'normalized', 'Position', [0.01, y_pos, 0.08, 0.03], 'String', sprintf('J%d (%d°)', i, current_thetas(i)), 'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
        slider_handles(i) = uicontrol('Style', 'slider', 'Units', 'normalized', 'Position', [0.1, y_pos, 0.22, 0.03], 'Min', -180, 'Max', 180, 'Value', current_thetas(i), 'Callback', @(~,~) updateFromSliders());
    end
    updatePlot(current_thetas);
    
    % =========================================================
    % OBSTACLE FUNCTIONS
    % =========================================================
    function addObstacle(~,~)
        x=str2double(get(edit_obsX,'String'));
        y=str2double(get(edit_obsY,'String'));
        z=str2double(get(edit_obsZ,'String'));
        s=str2double(get(edit_obsSize,'String'));
        t=get(popup_type,'Value');
        obstacles(end+1)=struct('type',t,'pos',[x y z],'size',s);
        updatePlot(current_thetas);
    end
    function clearObstacles(~,~)
        obstacles=struct('type',{},'pos',{},'size',{});
        updatePlot(current_thetas);
    end
    
    % =========================================================================
    % MAIN LOGIC FLOW
    % =========================================================================
    
    function planSingleHopAction()
        ee_trace = []; 
        set(status_text, 'String', 'Status: Calculating Inverse Kinematics...', 'ForegroundColor', 'b'); 
        set(btn_exec, 'Enable', 'off'); drawnow;
        
        target_pos = [str2double(get(edit_targetX, 'String')); str2double(get(edit_targetY, 'String')); str2double(get(edit_targetZ, 'String'))];
        
        if current_base_joint == 1, default_z = [0;0;-1]; else, default_z = [0;0;1]; end
                      
        if get(chk_ori, 'Value')
            rx = str2double(get(edit_rx, 'String')); ry = str2double(get(edit_ry, 'String')); rz = str2double(get(edit_rz, 'String'));
            target_z_dir = rotz(rz)*roty(ry)*rotx(rx) * [0;0;1]; ori_weight = 30; 
        else
            target_z_dir = default_z; ori_weight = 30; 
        end
        
        [target_q, ik_success] = solveIK(target_pos, current_thetas, target_z_dir, ori_weight);
        
        if ~ik_success
            set(status_text, 'String', 'Status: IK Failed. Target unreachable.', 'ForegroundColor', 'r'); return;
        end
        
        set(status_text, 'String', 'Status: Planning RRT* Path (Watch Plot)...', 'ForegroundColor', 'b'); drawnow;
        rrt_iters = str2double(get(edit_rrtIters, 'String')); rrt_step = str2double(get(edit_rrtStep, 'String')); goal_bias = str2double(get(edit_goalBias, 'String'));
        planned_path_q = planRRTStar(current_thetas, target_q, rrt_iters, rrt_step, goal_bias);
        
        if isempty(planned_path_q)
            set(status_text, 'String', 'Status: RRT* Failed to find a collision-free path.', 'ForegroundColor', 'r');
        else
            ee_path = zeros(3, size(planned_path_q, 1));
            for p = 1:size(planned_path_q, 1)
                [~, T_ee_temp] = getForwardKinematics(planned_path_q(p, :)); ee_path(:, p) = T_ee_temp(1:3, 4);
            end
            plot3(ax, ee_path(1,:), ee_path(2,:), ee_path(3,:), 'g-', 'LineWidth', 4);
            plot3(ax, target_pos(1), target_pos(2), target_pos(3), 'gx', 'MarkerSize', 15, 'LineWidth', 3);
            set(status_text, 'String', 'Status: Path Found! Press EXECUTE MOTION.', 'ForegroundColor', [0 0.5 0]);
            set(btn_exec, 'Enable', 'on');
        end
    end
    
    function executeMotionAction()
        set(status_text, 'String', 'Status: Executing Path...', 'ForegroundColor', [0 0.5 0]); set(btn_exec, 'Enable', 'off');
        for i = 1:size(planned_path_q, 1)
            current_thetas = planned_path_q(i, :);
            for j = 1:6
                set(slider_handles(j), 'Value', current_thetas(j)); set(text_handles(j), 'String', sprintf('J%d (%d°)', j, round(current_thetas(j))));
            end
            
            [~, T_EE] = getForwardKinematics(current_thetas);
            ee_trace = [ee_trace, T_EE(1:3, 4)];
            
            updatePlot(current_thetas); drawnow limitrate;
        end
        set(status_text, 'String', 'Status: Arrived at destination.', 'ForegroundColor', [0 0.5 0]);
        planned_path_q = []; 
    end
    
    % --- DYNAMIC MULTI HOP LOGIC ---
    function planAndExecuteDynamicMultiHop()
        ee_trace = []; 
        target_pos = [str2double(get(edit_targetX, 'String')); str2double(get(edit_targetY, 'String')); str2double(get(edit_targetZ, 'String'))];
        
        if get(chk_ori, 'Value')
            rx = str2double(get(edit_rx, 'String')); ry = str2double(get(edit_ry, 'String')); rz = str2double(get(edit_rz, 'String'));
            target_z_dir = rotz(rz)*roty(ry)*rotx(rx) * [0;0;1]; final_ori_weight = 30;
        else
            target_z_dir = [0;0;-1]; final_ori_weight = 20;
        end

        set(status_text, 'String', 'Status: Validating reachability from candidate docks...', 'ForegroundColor', 'b'); drawnow;
        dists_to_target = sum((docking_points - target_pos').^2, 2); 
        [~, sorted_dock_idx] = sort(dists_to_target);
        
        terminal_dock_idx = -1;
        for i = 1:length(sorted_dock_idx)
            candidate_idx = sorted_dock_idx(i);
            if norm(docking_points(candidate_idx,:)' - target_pos) > 55
                continue;
            end
            
            old_Base       = Global_Base_T;
            old_base_joint = current_base_joint;
            Global_Base_T       = eye(4);
            Global_Base_T(1:3,4) = docking_points(candidate_idx, :)';
            current_base_joint  = 1;   
            [~, ik_success] = solveIK(target_pos, [0,30,0,0,0,0], target_z_dir, final_ori_weight);
            Global_Base_T      = old_Base;
            current_base_joint = old_base_joint;
            
            if ik_success
                terminal_dock_idx = candidate_idx;
                break;
            end
        end
        
        if terminal_dock_idx == -1
            set(status_text, 'String', 'Status: Target is unreachable from ANY nearby dock.', 'ForegroundColor', 'r'); return;
        end
        current_base_pos = Global_Base_T(1:3, 4);
        dists_to_base = sum((docking_points - current_base_pos').^2, 2); 
        [~, start_dock_idx] = min(dists_to_base);
        
        blocked_edges = zeros(0, 2);
        route_indices = findGlobalRoute(start_dock_idx, terminal_dock_idx, blocked_edges);
        rrt_iters = str2double(get(edit_rrtIters, 'String')); rrt_step = str2double(get(edit_rrtStep, 'String')); goal_bias = str2double(get(edit_goalBias, 'String'));
        
        while length(route_indices) > 1
            curr_dock = route_indices(1);
            next_dock = route_indices(2);
            next_dock_pos = docking_points(next_dock, :)';
            
            set(status_text, 'String', sprintf('Status: Routing to Dock %d...', next_dock), 'ForegroundColor', 'b');
            
            global_route_pts = docking_points(route_indices, :); updatePlot(current_thetas); drawnow;
            
            if current_base_joint == 1, z_dir = [0;0;-1]; else, z_dir = [0;0;1]; end
            
            [target_q, ik_success] = solveIK(next_dock_pos, current_thetas, z_dir, 30);
            rrt_success = false;
            
            if ik_success && ~checkCollisions(target_q)
                path_q = planRRTStar(current_thetas, target_q, rrt_iters, rrt_step, goal_bias);
                if ~isempty(path_q), rrt_success = true; end
            end
            
            if rrt_success
                for p = 1:size(path_q, 1)
                    current_thetas = path_q(p, :);
                    for j = 1:6, set(slider_handles(j), 'Value', current_thetas(j)); set(text_handles(j), 'String', sprintf('J%d (%d°)', j, round(current_thetas(j)))); end
                    
                    [~, T_EE] = getForwardKinematics(current_thetas);
                    ee_trace = [ee_trace, T_EE(1:3, 4)];
                    
                    updatePlot(current_thetas); drawnow limitrate;
                end
                swapBase(); pause(0.5);
                route_indices(1) = []; 
            else
                set(status_text, 'String', sprintf('Status: Path blocked to Dock %d. Replanning alternate route...', next_dock), 'ForegroundColor', [1 0.5 0]);
                blocked_edges = [blocked_edges; curr_dock, next_dock];
                
                route_indices = findGlobalRoute(curr_dock, terminal_dock_idx, blocked_edges);
                if isempty(route_indices)
                    set(status_text, 'String', 'Status: All alternate routes blocked! Stuck in grid.', 'ForegroundColor', 'r'); 
                    global_route_pts = []; updatePlot(current_thetas); return;
                end
                pause(1.0);
            end
        end
        
        global_route_pts = []; updatePlot(current_thetas);
        set(status_text, 'String', 'Status: Terminal Dock Reached. Reaching for target...', 'ForegroundColor', 'b'); drawnow;
        
        [target_q, ik_success] = solveIK(target_pos, current_thetas, target_z_dir, final_ori_weight);
        if ik_success
            path_q = planRRTStar(current_thetas, target_q, rrt_iters, rrt_step, goal_bias);
            if ~isempty(path_q)
                for p = 1:size(path_q, 1)
                    current_thetas = path_q(p, :);
                    for j = 1:6, set(slider_handles(j), 'Value', current_thetas(j)); set(text_handles(j), 'String', sprintf('J%d (%d°)', j, round(current_thetas(j)))); end
                    
                    [~, T_EE] = getForwardKinematics(current_thetas);
                    ee_trace = [ee_trace, T_EE(1:3, 4)];
                    
                    updatePlot(current_thetas); drawnow limitrate;
                end
                set(status_text, 'String', 'Status: Success! Target Reached via Dynamic Replanning.', 'ForegroundColor', [0 0.5 0]);
                return;
            end
        end
        set(status_text, 'String', 'Status: Reached terminal dock, but RRT* failed to reach Target.', 'ForegroundColor', 'r');
    end
    
    % --- A* GLOBAL ROUTING ---
    function route = findGlobalRoute(start_idx, goal_idx, blocked_edges)
        max_reach = 35;
        open_set = start_idx; 
        came_from = containers.Map('KeyType', 'double', 'ValueType', 'double');
        
        g_score = inf(size(docking_points, 1), 1); g_score(start_idx) = 0;
        f_score = inf(size(docking_points, 1), 1); f_score(start_idx) = norm(docking_points(start_idx, :) - docking_points(goal_idx, :));
        
        while ~isempty(open_set)
            [~, min_idx] = min(f_score(open_set)); current = open_set(min_idx);
            
            if current == goal_idx
                route = goal_idx; curr = goal_idx;
                while came_from.isKey(curr), curr = came_from(curr); route = [curr, route]; end
                return;
            end
            
            open_set(min_idx) = []; curr_pos = docking_points(current, :);
            
            for neighbor = 1:size(docking_points, 1)
                if neighbor == current, continue; end
                
                if ~isempty(blocked_edges)
                    if any((blocked_edges(:,1) == current & blocked_edges(:,2) == neighbor) | ...
                           (blocked_edges(:,2) == current & blocked_edges(:,1) == neighbor))
                        continue; 
                    end
                end
                
                dist = norm(curr_pos - docking_points(neighbor, :));
                if dist <= max_reach
                    tentative_g_score = g_score(current) + dist;
                    if tentative_g_score < g_score(neighbor)
                        came_from(neighbor) = current; g_score(neighbor) = tentative_g_score;
                        f_score(neighbor) = g_score(neighbor) + norm(docking_points(neighbor, :) - docking_points(goal_idx, :));
                        if ~ismember(neighbor, open_set), open_set(end+1) = neighbor; end
                    end
                end
            end
        end
        route = []; 
    end
    
    % --- UTILS ---
    function updateFromSliders()
        for j = 1:6
            val = round(get(slider_handles(j), 'Value')); set(text_handles(j), 'String', sprintf('J%d (%d°)', j, val)); current_thetas(j) = val;
        end
        updatePlot(current_thetas);
        if checkCollisions(current_thetas), set(warn_text, 'Visible', 'on'); else, set(warn_text, 'Visible', 'off'); end
    end
    function plotFreeWorkspace()
        set(status_text, 'String', 'Status: Sampling Workspace (5000 points)...', 'ForegroundColor', 'b'); drawnow;
        num_samples = 5000; valid_count = 0; ws_points = zeros(3, num_samples);
        for i = 1:num_samples
            q_test = (rand(1,6) * 360) - 180;
            if ~checkCollisions(q_test)
                valid_count = valid_count + 1; [~, T_ee] = getForwardKinematics(q_test); ws_points(:, valid_count) = T_ee(1:3, 4);
            end
        end
        if valid_count > 0
            scatter3(ax, ws_points(1,1:valid_count), ws_points(2,1:valid_count), ws_points(3,1:valid_count), 5, [0 0.8 1], 'filled', 'MarkerFaceAlpha', 0.2);
            set(status_text, 'String', sprintf('Status: Found %d Free Configurations.', valid_count), 'ForegroundColor', [0 0.5 0]);
        end
    end
    
    % =========================================================================
    % KINEMATICS & COLLISION LOGIC
    % =========================================================================
    function T_i = compute_Ti(a, alpha, d, theta)
        Q = [1, 0, 0, a; 0, cosd(alpha), -sind(alpha), 0; 0, sind(alpha), cosd(alpha), 0; 0, 0, 0, 1];
        R = [cosd(theta), -sind(theta), 0, 0; sind(theta), cosd(theta), 0, 0; 0, 0, 1, d; 0, 0, 0, 1]; T_i = Q * R;
    end
    function R = rotx(deg), R = [1 0 0; 0 cosd(deg) -sind(deg); 0 sind(deg) cosd(deg)]; end
    function R = roty(deg), R = [cosd(deg) 0 sind(deg); 0 1 0; -sind(deg) 0 cosd(deg)]; end
    function R = rotz(deg), R = [cosd(deg) -sind(deg) 0; sind(deg) cosd(deg) 0; 0 0 1]; end
    function [positions, T_EE] = getForwardKinematics(thetas)
        T_local = eye(4); local_frames = zeros(4,4,7); local_frames(:,:,1) = eye(4);
        for k = 1:6
            T_local = T_local * compute_Ti(dh_table(k,1), dh_table(k,2), dh_table(k,3), thetas(k)); local_frames(:,:,k+1) = T_local;
        end
        T_global = zeros(4, 4, 7);
        if current_base_joint == 1, T_global_1 = Global_Base_T; else, T_global_1 = Global_Base_T / local_frames(:,:,7); end
        positions = zeros(3, 7);
        for k = 1:7
            T_global(:,:,k) = T_global_1 * local_frames(:,:,k); positions(:, k) = T_global(1:3, 4, k);
        end
        if current_base_joint == 1, T_EE = T_global(:,:,7); else, T_EE = T_global(:,:,1); end
    end
    
    function isCollision = checkCollisions(thetas)
        positions = getForwardKinematics(thetas);
        isCollision = false;
        
        % 1. Predefined Sphere Checks
        for obs = 1:size(env_spheres, 1)
            obs_center = env_spheres(obs, 1:3)'; obs_radius = env_spheres(obs, 4);
            for k = 1:6
                p1 = positions(:, k); p2 = positions(:, k+1); r_link = cyl_radius(k);
                dist = dist3D_Point_to_Segment(obs_center, p1, p2);
                if dist < (obs_radius + r_link + 0.5), isCollision = true; return; end
            end
        end
        % 2. Predefined Capsule Checks
        for obs = 1:size(env_capsules, 1)
            obs_p1 = env_capsules(obs, 1:3)'; obs_p2 = env_capsules(obs, 4:6)'; obs_radius = env_capsules(obs, 7);
            for k = 1:6
                link_p1 = positions(:, k); link_p2 = positions(:, k+1); r_link = cyl_radius(k);
                dist = dist3D_Segment_to_Segment(link_p1, link_p2, obs_p1, obs_p2);
                if dist < (obs_radius + r_link + 0.5), isCollision = true; return; end
            end
        end
        % 3. UI Added Dynamic Obstacles Check
        for obs = 1:length(obstacles)
            o = obstacles(obs);
            for k = 1:6
                p1 = positions(:, k); p2 = positions(:, k+1); 
                switch o.type
                    case 1 % sphere
                        dist = dist3D_Point_to_Segment(o.pos', p1, p2);
                        if dist < (o.size + cyl_radius(k))
                            isCollision = true; return;
                        end
                    case 2 % cylinder
                        d_xy = norm(o.pos(1:2)' - p1(1:2));
                        if d_xy < (o.size + cyl_radius(k))
                            isCollision = true; return;
                        end
                    case 3 % box — full capsule vs AABB check
                        box_min = (o.pos - o.size)';
                        box_max = (o.pos + o.size)';
                        seg_to_box_dist = dist3D_Segment_to_AABB(p1, p2, box_min, box_max);
                        if seg_to_box_dist < cyl_radius(k)
                            isCollision = true; return;
                        end
                end
            end
        end
        
        % 4. Self Collision Check
        pairs = [1 4; 1 5; 1 6; 2 5; 2 6; 3 6];
        for i = 1:size(pairs, 1)
            idx1 = pairs(i, 1); idx2 = pairs(i, 2);
            p1 = positions(:, idx1); p2 = positions(:, idx1+1); r1 = cyl_radius(idx1);
            q1 = positions(:, idx2); q2 = positions(:, idx2+1); r2 = cyl_radius(idx2);
            dist = dist3D_Segment_to_Segment(p1, p2, q1, q2);
            if dist < (r1 + r2 + self_collision_margin), isCollision = true; return; end
        end
        
        % -------------------------------------------------------------------
        % 5. FIXED GROUND COLLISION CHECK 
        % -------------------------------------------------------------------
        % Strictly prevents joint centers from dipping below the surface plane 
        % (with minor eps margin for floating-point dock intersections). 
        if any(positions(3, :) < -0.01)
            isCollision = true; 
            return; 
        end
    end
    
    function dist = dist3D_Point_to_Segment(pt, p1, p2)
        v = p2 - p1; w = pt - p1; c1 = dot(w, v); c2 = dot(v, v);
        if c1 <= 0, dist = norm(pt - p1); elseif c2 <= c1, dist = norm(pt - p2); else, dist = norm(pt - (p1 + (c1 / c2) * v)); end
    end

    function dist = dist3D_Segment_to_AABB(p1, p2, box_min, box_max)
        % Analytically finds minimum distance from segment P(t)=p1+t*(p2-p1)
        % to an axis-aligned bounding box. Uses piecewise-quadratic critical
        % points: evaluate at endpoints + all slab face intersections.
        d = p2 - p1;
    
        % Start with segment endpoints, add t-values where segment crosses each face
        t_candidates = [0; 1];
        for i = 1:3
            if abs(d(i)) > 1e-10
                t_candidates(end+1) = (box_min(i) - p1(i)) / d(i);
                t_candidates(end+1) = (box_max(i) - p1(i)) / d(i);
            end
        end
    
        % Only keep t in [0,1] (on the segment)
        t_candidates = t_candidates(t_candidates >= 0 & t_candidates <= 1);
    
        dist = inf;
        for ii = 1:numel(t_candidates)
            pt      = p1 + t_candidates(ii) * d;          % point on segment
            closest = max(box_min, min(box_max, pt));      % nearest point on/in AABB
            dist    = min(dist, norm(pt - closest));
            if dist == 0, return; end                      % early exit: intersection
        end
    end
    
    function dist = dist3D_Segment_to_Segment(p1, p2, q1, q2)
        u = p2 - p1; v = q2 - q1; w = p1 - q1;
        a = dot(u,u); b = dot(u,v); c = dot(v,v); d = dot(u,w); e = dot(v,w);
        D = a*c - b*b; sc = 0; sN = 0; sD = D; tc = 0; tN = 0; tD = D;
        if D < 1e-8, sN = 0; sD = 1; tN = e; tD = c;
        else
            sN = (b*e - c*d); tN = (a*e - b*d);
            if sN < 0, sN = 0; tN = e; tD = c; elseif sN > sD, sN = sD; tN = e + b; tD = c; end
        end
        if tN < 0
            tN = 0; if -d < 0, sN = 0; elseif -d > a, sN = sD; else, sN = -d; sD = a; end
        elseif tN > tD
            tN = tD; if (-d + b) < 0, sN = 0; elseif (-d + b) > a, sN = sD; else, sN = (-d + b); sD = a; end
        end
        if abs(sN) < 1e-8, sc = 0; else, sc = sN / sD; end
        if abs(tN) < 1e-8, tc = 0; else, tc = tN / tD; end
        dP = w + (sc * u) - (tc * v); dist = norm(dP);
    end
    
    function [best_q, success] = solveIK(target_pos, q_seed, target_z_dir, ori_weight)
        costFunc = @(q) evaluateIK(q, target_pos, target_z_dir, ori_weight);
        options = optimset('Display', 'off', 'MaxIter', 1500, 'MaxFunEvals', 3000);
        seeds = [q_seed];
        for s = 1:20, seeds = [seeds; (rand(1,6)*360) - 180]; end
        
        best_fval = inf; best_q = q_seed;
        
        best_fval_any = inf; best_q_any = q_seed;
        for s = 1:size(seeds,1)
            [q_sol, fval] = fminsearch(costFunc, seeds(s,:), options);
            q_sol = mod(q_sol + 180, 360) - 180;
            
            if fval < best_fval_any
                best_fval_any = fval; best_q_any = q_sol;
            end
            
            if fval < best_fval && ~checkCollisions(q_sol)
                best_fval = fval; best_q = q_sol; 
            end
            if best_fval < 2.0, break; end
        end
        
        if best_fval == inf && best_fval_any < 10.0
            best_q = best_q_any;
            best_fval = best_fval_any;
        end
        success = best_fval < 10.0; 
    end
    
    function cost = evaluateIK(q, target_pos, target_z_dir, ori_weight)
        [~, T_ee] = getForwardKinematics(q);
        cost = norm(T_ee(1:3, 4) - target_pos)*10 + (norm(T_ee(1:3, 3) - target_z_dir) * ori_weight); 
    end
    
    % --- RRT* PLANNER WITH L6 NORM ---
    function path = planRRTStar(q_start, q_goal, max_nodes, step_size, goal_bias)
        tree.q = q_start; 
        tree.parent = 0;
        tree.cost = 0;
        
        search_radius = step_size * 2.5; 
        
        for iter = 1:max_nodes
            if rand < goal_bias, q_rand = q_goal; else, q_rand = (rand(1,6) * 360) - 180; end
            
            % Find Nearest Node Using L6 Norm
            diffs_nn = mod(tree.q - q_rand + 180, 360) - 180;
            dists = sum(abs(diffs_nn).^6, 2).^(1/6);
            [~, nearest_idx] = min(dists);
            
            q_near = tree.q(nearest_idx, :);
            dir = mod(q_rand - q_near + 180, 360) - 180;
            dist = norm(dir, 6); % L6 Norm for vector direction limiting
            
            if dist > step_size
                q_new = q_near + (dir/dist) * step_size; 
            else
                q_new = q_rand; 
            end
            q_new = mod(q_new + 180, 360) - 180;
            
            if ~checkCollisions(q_new)
                % RRT* Radius Search (L6 Norm)
                diffs_all = mod(tree.q - q_new + 180, 360) - 180;
                dists_all = sum(abs(diffs_all).^6, 2).^(1/6);
                near_indices = find(dists_all <= search_radius);
                
                % Optimize Parent
                best_parent = nearest_idx;
                min_cost = tree.cost(nearest_idx) + norm(mod(q_new - tree.q(nearest_idx,:) + 180, 360) - 180, 6);
                
                for i = 1:length(near_indices)
                    near_idx = near_indices(i);
                    c_near = tree.cost(near_idx) + norm(mod(q_new - tree.q(near_idx,:) + 180, 360) - 180, 6);
                    if c_near < min_cost
                        best_parent = near_idx;
                        min_cost = c_near;
                    end
                end
                
                % Add node to tree
                tree.q = [tree.q; q_new]; 
                tree.parent = [tree.parent; best_parent];
                tree.cost = [tree.cost; min_cost];
                new_idx = size(tree.q, 1);
                
                % RRT* Rewiring
                for i = 1:length(near_indices)
                    near_idx = near_indices(i);
                    if near_idx == best_parent, continue; end
                    
                    c_rewire = min_cost + norm(mod(tree.q(near_idx,:) - q_new + 180, 360) - 180, 6);
                    if c_rewire < tree.cost(near_idx)
                        tree.parent(near_idx) = new_idx;
                        tree.cost(near_idx) = c_rewire;
                    end
                end
                
                % Goal detection logic using L6
                goal_diff = mod(q_new - q_goal + 180, 360) - 180;
                if norm(goal_diff, 6) < step_size
                    tree.q = [tree.q; q_goal]; 
                    tree.parent = [tree.parent; new_idx];
                    tree.cost = [tree.cost; min_cost + norm(goal_diff, 6)];
                    
                    path_idx = size(tree.q, 1); path_reversed = [];
                    while path_idx ~= 0
                        path_reversed = [tree.q(path_idx, :); path_reversed]; 
                        path_idx = tree.parent(path_idx); 
                    end
                    path = path_reversed; 
                    return;
                end
            end
        end
        path = [];
    end
    
    function swapBase()
        T_local_6 = eye(4);
        for k = 1:6
            T_local_6 = T_local_6 * compute_Ti(dh_table(k,1), dh_table(k,2), dh_table(k,3), current_thetas(k));
        end
        
        if current_base_joint == 1
            Global_Base_T = Global_Base_T * T_local_6;
            current_base_joint = 6;
        else
            Global_Base_T = Global_Base_T / T_local_6;
            current_base_joint = 1;
        end
        
        Global_Base_T(3, 4) = 0;   
        updatePlot(current_thetas);
    end
    
    % =========================================================================
    % GRAPHICS PIPELINE
    % =========================================================================
    function updatePlot(thetas)
        cla(ax);
        
        patch(ax, [-75 75 75 -75], [-75 -75 75 75], [0 0 0 0], [0.4 0.6 0.4], 'FaceAlpha', 0.8, 'EdgeColor', [0.2 0.4 0.2], 'LineWidth', 2);
        
        for d = 1:size(docking_points, 1)
            plot3(ax, docking_points(d,1), docking_points(d,2), docking_points(d,3), 'ko', 'MarkerSize', 10, 'LineWidth', 1.5);
            plot3(ax, docking_points(d,1), docking_points(d,2), docking_points(d,3), 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);
        end
        
        if ~isempty(global_route_pts)
            plot3(ax, global_route_pts(:,1), global_route_pts(:,2), global_route_pts(:,3), 'Color', [1 0.5 0], 'LineStyle', '--', 'LineWidth', 3);
        end
        
        if size(ee_trace, 2) > 1
            plot3(ax, ee_trace(1,:), ee_trace(2,:), ee_trace(3,:), 'c-', 'LineWidth', 2.5);
        end
        
        for obs = 1:size(env_spheres, 1)
            [sx, sy, sz] = sphere(15); r = env_spheres(obs, 4);
            surf(ax, sx*r + env_spheres(obs,1), sy*r + env_spheres(obs,2), sz*r + env_spheres(obs,3), 'FaceColor', 'r', 'EdgeColor', 'none', 'FaceAlpha', 0.5);
        end
        
        for obs = 1:size(env_capsules, 1)
            p1 = env_capsules(obs, 1:3)'; p2 = env_capsules(obs, 4:6)'; r = env_capsules(obs, 7);
            drawCylinder_between_points(ax, p1, p2, r, [0.8 0.3 0.1]); 
            [sx, sy, sz] = sphere(10);
            surf(ax, sx*r + p1(1), sy*r + p1(2), sz*r + p1(3), 'FaceColor', [0.8 0.3 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
            surf(ax, sx*r + p2(1), sy*r + p2(2), sz*r + p2(3), 'FaceColor', [0.8 0.3 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
        end
        
        for obs = 1:length(obstacles)
            o = obstacles(obs);
            switch o.type
                case 1 % Sphere
                    [sx, sy, sz] = sphere(15);
                    surf(ax, sx*o.size + o.pos(1), ...
                             sy*o.size + o.pos(2), ...
                             sz*o.size + o.pos(3), ...
                             'FaceColor','r','EdgeColor','none','FaceAlpha',0.3);
                case 2 % Cylinder
                    [cx, cy, cz] = cylinder(o.size,20);
                    cz = cz * o.size * 2;
                    surf(ax, cx + o.pos(1), ...
                             cy + o.pos(2), ...
                             cz + o.pos(3) - o.size, ...
                             'FaceColor','g','FaceAlpha',0.3);
                case 3 % Box
                    drawBox(o.pos, o.size);
            end
        end
        
        [positions, T_EE] = getForwardKinematics(thetas);
        drawLatch(ax, Global_Base_T, 4, [0.2 0.2 0.2]); 
        for k = 1:6
            drawCylinder_between_points(ax, positions(:, k), positions(:, k+1), cyl_radius(k), link_colors(k,:));
            drawHub(ax, positions(:, k+1), cyl_radius(k) * 1.3);
        end
        drawLatch(ax, T_EE, 4, [0.8 0.8 0.2]); 
        plot3(ax, T_EE(1,4), T_EE(2,4), T_EE(3,4), 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm'); 
    end
    
    function drawLatch(ax, T, radius, color)
        [cX, cY, cZ] = cylinder(radius, 20); cZ = cZ * 2 - 1; 
        for m = 1:numel(cX), v = T * [cX(m); cY(m); cZ(m); 1]; cX(m) = v(1); cY(m) = v(2); cZ(m) = v(3); end
        surf(ax, cX, cY, cZ, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    end
    
    function drawHub(ax, pos, r)
        [sX, sY, sZ] = sphere(10); surf(ax, sX*r + pos(1), sY*r + pos(2), sZ*r + pos(3), 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none');
    end
    
    function drawCylinder_between_points(ax, p1, p2, r, color)
        v = p2 - p1; L = norm(v); if L < 1e-4, return; end 
        [cX, cY, cZ] = cylinder(r, 20); cZ = cZ * L; v_dir = v / L; z_dir = [0; 0; 1];
        cross_p = cross(z_dir, v_dir); sin_angle = norm(cross_p); cos_angle = dot(z_dir, v_dir);
        if sin_angle < 1e-6
            if cos_angle < 0, R = [1 0 0; 0 -1 0; 0 0 -1]; else, R = eye(3); end
        else
            cross_skew = [0, -cross_p(3), cross_p(2); cross_p(3), 0, -cross_p(1); -cross_p(2), cross_p(1), 0];
            R = eye(3) + cross_skew + (cross_skew^2) * ((1 - cos_angle) / (sin_angle^2));
        end
        for m = 1:numel(cX), point = R * [cX(m); cY(m); cZ(m)] + p1; cX(m) = point(1); cY(m) = point(2); cZ(m) = point(3); end
        surf(ax, cX, cY, cZ, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    end
    function drawBox(center,s)
        [X,Y,Z]=ndgrid([0 1]);
        X=(X*2-1)*s+center(1);
        Y=(Y*2-1)*s+center(2);
        Z=(Z*2-1)*s+center(3);
        K=convhull(X(:),Y(:),Z(:));
        trisurf(K,X(:),Y(:),Z(:),'FaceAlpha',0.2);
    end
end
