function animate_3d(speed, name, s_pts)

    % Load data
    lm = reshape(fread(fopen(strcat('/tmp/landmarks.log'), 'r'), 'double'), 3, []);
    air_state = reshape(fread(fopen(strcat(['/tmp/',name,'_true_state.log']), 'r'), 'double'), 1 + 19, []);
    gmbl_state = reshape(fread(fopen(strcat(['/tmp/','gimbal','_true_state.log']), 'r'), 'double'), 1 + 19, []);
    air_command = reshape(fread(fopen(strcat(['/tmp/',name,'_commanded_state.log']), 'r'), 'double'), 1 + 19, []);
    bike_state = reshape(fread(fopen('/tmp/bike1_true_state.log', 'r'), 'double'), 7, []);
    gmbl_params = ReadYaml('../params/gimbal.yaml');
    fov_x = 2*atan(gmbl_params.image_size{1}/2/gmbl_params.camera_matrix{1});
    fov_y = 2*atan(gmbl_params.image_size{2}/2/gmbl_params.camera_matrix{5});
    R_g_cb = R_from_q(cell2mat(gmbl_params.q_bcb));

    persistent body_handle gmbl_handle true_path_handle cmd_handle bike_trail_handle bike_handle

    % body vertices
    points = s_pts * [0.7, 0, 0;
                      1.0, 0, 0.1;
                      -0.3, 0.3, 0;
                      -0.5, 0.5, 0.1;
                      -0.3, -0.3, 0;
                      -0.5, -0.5, 0.1];
    s_ax = 5; % axis buffer
    air_history = 10000;
    bike_history = 30000;
    for i =1:speed:length(air_state)
        if i < air_history
            qh = i - 1;
        else
            qh = air_history;
        end
        if i < bike_history
            bh = i - 1;
        else
            bh = bike_history;
        end
        if i == 1
            figure(), grid on, hold on, axis equal
            set(gcf, 'name', '3D Animation', 'NumberTitle', 'off')
            set(gcf, 'color', 'w')
            axis([min([air_state(2,:),bike_state(2,:)])-s_ax max([air_state(2,:),bike_state(2,:)])+s_ax,... % x
                  min([air_state(3,:),bike_state(3,:)])-s_ax max([air_state(3,:),bike_state(3,:)])+s_ax,... % y
                  min([air_state(4,:),bike_state(4,:)])-s_ax max([air_state(4,:),bike_state(4,:)])+s_ax])   % z
            xlabel('North axis');
            ylabel('East axis');
            zlabel('Down axis');
            view(-50, 20) % (azimuth, elevation)
            set(gca,'YDir','Reverse')
            set(gca,'ZDir','Reverse')
            
            plot3(lm(1,:), lm(2,:), lm(3,:), 'k.', 'MarkerSize', 4.0)
            plot3([air_state(2,1),air_command(2,1)], [air_state(3,1),air_command(3,1)], [air_state(4,1),air_command(4,1)], 'g--')
            body_handle = draw_body(i, points, air_state, []);
            gmbl_handle = draw_cam_fov(i, gmbl_state, fov_x, fov_y, R_g_cb, []);
            true_path_handle = plot3(air_state(2,1:i), air_state(3,1:i), air_state(4,1:i), 'b', 'linewidth', 1.5);
            cmd_handle = plot3(air_command(2,1:i), air_command(3,1:i), air_command(4,1:i), 'g--', 'linewidth', 1.3);
            bike_trail_handle = plot3(bike_state(2,1:i), bike_state(3,1:i), bike_state(4,1:i), 'm', 'linewidth', 1.3);
            bike_handle = plot3(bike_state(2,i), bike_state(3,i), bike_state(4,i), 'm', 'Marker', '*', 'markersize', 5.0);
        else
            draw_body(i, points, air_state, body_handle);
            draw_cam_fov(i, gmbl_state, fov_x, fov_y, R_g_cb, gmbl_handle);
            set(true_path_handle, 'XData', air_state(2,i-qh:i), 'YData', air_state(3,i-qh:i),'ZData', air_state(4,i-qh:i));
            set(cmd_handle, 'XData', air_command(2,1:i), 'YData', air_command(3,1:i),'ZData', air_command(4,1:i));
            set(bike_trail_handle, 'XData', bike_state(2,i-bh:i), 'YData', bike_state(3,i-bh:i),'ZData', bike_state(4,i-bh:i));
            set(bike_handle, 'XData', bike_state(2,i), 'YData', bike_state(3,i),'ZData', bike_state(4,i));
            drawnow
        end
    end

end

function handle = draw_body(iter, body_verts, state, handle)
    R_I_b = R_from_q(state(11:14, iter));
    verts_I = (R_I_b' * body_verts' + state(2:4, iter))';

    % define all vertices
    V_fr = verts_I([1,2,4,3], :); % front-right
    V_fl = verts_I([1,2,6,5], :); % front-left
    V_r = verts_I([3,4,6,5], :); % rear
    V_b = verts_I([1, 3, 5], :); % bottom
    V_t = verts_I([2, 4, 6], :); % top
    V = [V_fr;V_fl;V_r;V_b;V_t]; % combined
    
    % define faces
    F = [1,2,3,4; % front-right
         5,6,7,8; % front-left
         9,10,11,12; % rear
         13,14,15,13; % bottom
         16,17,18,16]; % top
     
    % define colors
    myred   = [1, 0, 0];
    mygreen = [0, 1, 0];
    myblue  = [0, 0, 1];
    myblack = [0, 0, 0];
    patchcolors = [myred;  % front-right
                   myblue;  % front-left
                   mygreen;  % rear
                   myblack;  % bottom
                   myblack]; % top
        
    if iter == 1
        handle = patch('Vertices', V, 'Faces', F,...
                       'FaceVertexCData', patchcolors, 'FaceColor', 'flat',...
                       'FaceAlpha', 0.5);
    else
        set(handle, 'Vertices', V, 'Faces', F);
    end
end

function handle = draw_cam_fov(iter, state, fov_x, fov_y, R_g_cb, handle)
    % extract data
    R_I_g = R_from_q(state(11:14, iter));
    pn = state(2, iter);
    pe = state(3, iter);
    pd = state(4, iter);
                           
    % define unit vectors along fov in the camera-body frame
    % this is derived from the geometry of a pin-hole camera's fov
    value = sqrt(1 + tan(fov_x/2)^2 + tan(fov_y/2)^2);
    ix = 1/value;
    iy = tan(fov_x/2)/value;
    iz = tan(fov_y/2)/value;
    pts = [ ix,  iy, -iz     % top-right
            ix, -iy, -iz     % top-left
            ix, -iy,  iz     % bot-left
            ix,  iy,  iz ]'; % bot-right
        
    % transform from gimbal coordinates to inertial coordinates
    pts = R_I_g'*R_g_cb'*pts;

    % first vertex is at center of MAV vehicle frame
    V = [pn, pe, pd]; 
    
    % project field of view lines onto ground plane and make correction
    % when the projection is above the horizon
    for i = 1:4
        
        % alpha is the angle that the field-of-view line makes with horizon
        alpha = atan2(pts(3,i),norm(pts(1:2,i)));
        
        if alpha > 0
            
            % fov line is below horizon and intersects ground plane
            V = [...
                V
                [pn-pd*pts(1,i)/pts(3,i), pe-pd*pts(2,i)/pts(3,i), 0]
                ];
            
        elseif alpha < 0
            
            % fov line is above horizon and intersects some high plane
            V = [...
                V
                [pn+pd*pts(1,i)/pts(3,i), pe+pd*pts(2,i)/pts(3,i), pd*2]
                ];
            
        else

            % fov line exactly on horizon and intersects no plane
            V = [...
                V
                [pn+999*cos(fov_x), pe+999*sin(fov_x), pd]
                ];
            
        end
    end
    
    F = [ 1  1  2  3    % top face
          1  1  2  5    % right face
          1  1  5  4    % bottom face
          1  1  4  3    % left face
          2  3  4  5 ]; % footprint face

    colors = [[1 1 1]; [1 1 1]; [1 1 1]; [1 1 1]; [0 1 0]];

    if iter == 1
        handle = patch('Vertices', V, 'Faces', F,...
                       'FaceVertexCData', colors, 'FaceColor', 'flat',...
                       'FaceAlpha', 0.05);
    else
        set(handle, 'Vertices', V, 'Faces', F);
    end
  
end 