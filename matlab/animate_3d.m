function animate_3d(speed, name)

    % Load data
    env = reshape(fread(fopen(strcat('/tmp/environment.log'), 'r'), 'double'), 3, []);
    air_state = reshape(fread(fopen(strcat(['/tmp/',name,'_true_state.log']), 'r'), 'double'), 1 + 19, []);
    air_command = reshape(fread(fopen(strcat(['/tmp/',name,'_commanded_state.log']), 'r'), 'double'), 1 + 19, []);
    bike_state = reshape(fread(fopen('/tmp/bike1_true_state.log', 'r'), 'double'), 7, []);

    persistent body_handle true_path_handle cmd_handle bike_trail_handle bike_handle

    % body vertices
    s_pts = 2.0; % model scale factor
    points = s_pts * [0.7, 0, 0;
                      1.0, 0, 0.1;
                      -0.3, 0.3, 0;
                      -0.5, 0.5, 0.1;
                      -0.3, -0.3, 0;
                      -0.5, -0.5, 0.1];
    s_ax = 1.2; % axis scale factor
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
            axis([s_ax*min([air_state(2,:),bike_state(2,:)]) s_ax*max([air_state(2,:),bike_state(2,:)]),... % x
                  s_ax*min([air_state(3,:),bike_state(3,:)]) s_ax*max([air_state(3,:),bike_state(3,:)]),... % y
                  s_ax*min([air_state(4,:),bike_state(4,:)]) s_ax*max([air_state(4,:),bike_state(4,:)])])   % z
            xlabel('North axis');
            ylabel('East axis');
            zlabel('Down axis');
            view(-50, 20) % (azimuth, elevation)
            set(gca,'YDir','Reverse')
            set(gca,'ZDir','Reverse')
            
            plot3(env(1,:), env(2,:), env(3,:), 'k.', 'MarkerSize', 4.0)
            plot3([air_state(2,1),air_command(2,1)], [air_state(3,1),air_command(3,1)], [air_state(4,1),air_command(4,1)], 'g--')
            body_handle = draw_body(i, points, air_state, []);
            true_path_handle = plot3(air_state(2,1:i), air_state(3,1:i), air_state(4,1:i), 'b', 'linewidth', 1.5);
            cmd_handle = plot3(air_command(2,1:i), air_command(3,1:i), air_command(4,1:i), 'g--', 'linewidth', 1.3);
            bike_trail_handle = plot3(bike_state(2,1:i), bike_state(3,1:i), bike_state(4,1:i), 'm', 'linewidth', 1.3);
            bike_handle = plot3(bike_state(2,i), bike_state(3,i), bike_state(4,i), 'm', 'Marker', '*', 'markersize', 5.0);
        else
            draw_body(i, points, air_state, body_handle);
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
            'FaceVertexCData', patchcolors, 'FaceColor', 'flat', 'FaceAlpha', 0.5);
    else
        set(handle, 'Vertices', V, 'Faces', F);
    end
end
