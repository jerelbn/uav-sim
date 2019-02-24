function animate_3d(speed, env, truth, cmd, bicycle_state)

    persistent body_handle true_path_handle cmd_handle bike_trail_handle bike_handle

    % body vertices
    points = [0.7, 0, 0;
              1.0, 0, 0.1;
              -0.3, 0.3, 0;
              -0.5, 0.5, 0.1;
              -0.3, -0.3, 0;
              -0.5, -0.5, 0.1];
    
    s = 1.5; % scale factor for axes
    quad_history = 10000;
    bike_history = 30000;
    for i =1:speed:length(truth)
        if i < quad_history
            qh = i - 1;
        else
            qh = quad_history;
        end
        if i < bike_history
            bh = i - 1;
        else
            bh = bike_history;
        end
        if i == 1
            figure(1e6), clf, grid on, hold on, axis equal
            set(gcf, 'name', '3D Animation', 'NumberTitle', 'off')
            set(gcf, 'color', 'w')
%             axis([s*min(env(1,:)) s*max(env(1,:)),... % x
%                   s*min(env(2,:)) s*max(env(2,:)),... % y
%                   s*min(env(3,:)) s*max(env(3,:))])   % z
            axis([-15, 15,... % x
                  -15, 15,... % y
                  -10,  3])
            xlabel('North axis');
            ylabel('East axis');
            zlabel('Down axis');
            view(-50, 20) % (azimuth, elevation)
            set(gca,'YDir','Reverse')
            set(gca,'ZDir','Reverse')
            
            plot3(env(1,:), env(2,:), env(3,:), 'k.', 'MarkerSize', 4.0)
            body_handle = draw_body(i, points, truth, []);
            true_path_handle = plot3(truth(2,1:i), truth(3,1:i), truth(4,1:i), 'b', 'linewidth', 1.5);
            cmd_handle = plot3(cmd(2,1:i), cmd(3,1:i), cmd(4,1:i), 'g--', 'linewidth', 1.3);
            bike_trail_handle = plot3(bicycle_state(2,1:i), bicycle_state(3,1:i), bicycle_state(4,1:i), 'm', 'linewidth', 1.3);
            bike_handle = plot3(bicycle_state(2,i), bicycle_state(3,i), bicycle_state(4,i), 'm', 'Marker', '*', 'markersize', 5.0);
        else
            draw_body(i, points, truth, body_handle);
            set(true_path_handle, 'XData', truth(2,i-qh:i), 'YData', truth(3,i-qh:i),'ZData', truth(4,i-qh:i));
            set(cmd_handle, 'XData', cmd(2,1:i), 'YData', cmd(3,1:i),'ZData', cmd(4,1:i));
            set(bike_trail_handle, 'XData', bicycle_state(2,i-bh:i), 'YData', bicycle_state(3,i-bh:i),'ZData', bicycle_state(4,i-bh:i));
            set(bike_handle, 'XData', bicycle_state(2,i), 'YData', bicycle_state(3,i),'ZData', bicycle_state(4,i));
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
