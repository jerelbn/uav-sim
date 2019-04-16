function plot_sensors(params)

if params.imu_enabled
    accel = reshape(fread(fopen(strcat(['/tmp/',params.name,'_accel.log']), 'r'), 'double'), 10, []); % [time;accel;bias;noise]
    figure()
    set(gcf, 'name', strcat([params.name,' accelerometer']), 'NumberTitle', 'off');
    titles = ["x","y","z"];
    for i=1:3
        subplot(3, 1, i), hold on, grid on
        title(titles(i))
        plot(accel(1,:), accel(i+1,:)-accel(i+4,:)-accel(i+7,:), 'b-', 'linewidth', 2.0)
        plot(accel(1,:), accel(i+1,:), 'r-', 'linewidth', 1.5)
        if i == 1
            legend('True', 'Measured')
        end
    end


    gyro = reshape(fread(fopen(strcat(['/tmp/',params.name,'_gyro.log']), 'r'), 'double'), 10, []); % [time;gyro;bias;noise]
    figure()
    set(gcf, 'name', strcat([params.name,' gyro']), 'NumberTitle', 'off');
    titles = ["x","y","z"];
    for i=1:3
        subplot(3, 1, i), hold on, grid on
        title(titles(i))
        plot(gyro(1,:), gyro(i+1,:)-gyro(i+4,:)-gyro(i+7,:), 'b-', 'linewidth', 2.0)
        plot(gyro(1,:), gyro(i+1,:), 'r-', 'linewidth', 1.5)
        if i == 1
            legend('True', 'Measured')
        end
    end
end


if params.baro_enabled
    baro = reshape(fread(fopen(strcat(['/tmp/',params.name,'_baro.log']), 'r'), 'double'), 4, []); % [time;baro;bias;noise]
    figure()
    set(gcf, 'name', strcat([params.name,' barometer']), 'NumberTitle', 'off');
    title("Barometer")
    grid on, hold on
    plot(baro(1,:), baro(2,:)-baro(3,:)-baro(4,:), 'b-', 'linewidth', 2.0)
    plot(baro(1,:), baro(2,:), 'r-', 'linewidth', 1.5)
    legend('True', 'Measured')
end


if params.pitot_enabled
    pitot = reshape(fread(fopen(strcat(['/tmp/',params.name,'_pitot.log']), 'r'), 'double'), 4, []); % [time;pitot;bias;noise]
    figure()
    set(gcf, 'name', strcat([params.name,' pitot tube']), 'NumberTitle', 'off');
    title("Pitot Tube")
    grid on, hold on
    plot(pitot(1,:), pitot(2,:)-pitot(3,:)-pitot(4,:), 'b-', 'linewidth', 2.0)
    plot(pitot(1,:), pitot(2,:), 'r-', 'linewidth', 1.5)
    legend('True', 'Measured')
end


if params.wvane_enabled
    wvane = reshape(fread(fopen(strcat(['/tmp/',params.name,'_wvane.log']), 'r'), 'double'), 3, []); % [time;wvane_meas;wvane_true]
    figure()
    set(gcf, 'name', strcat([params.name,' weather vane']), 'NumberTitle', 'off');
    title("Weather Vane")
    grid on, hold on
    plot(wvane(1,:), wvane(3,:), 'b-', 'linewidth', 2.0)
    plot(wvane(1,:), wvane(2,:), 'r-', 'linewidth', 1.5)
    legend('True', 'Measured')
end


if params.gps_enabled
    gps = reshape(fread(fopen(strcat(['/tmp/',params.name,'_gps.log']), 'r'), 'double'), 16, []); % [time;pos;vel;pos_bias;pos_noise;vel_noise]
    figure()
    set(gcf, 'name', strcat([params.name,' GPS position']), 'NumberTitle', 'off');
    titles = ["North","East","Altitude"];
    for i=1:3
        subplot(3, 1, i), hold on, grid on
        title(titles(i))
        if i < 3
            plot(gps(1,:), gps(i+1,:)-gps(i+7,:)-gps(i+10,:), 'b-', 'linewidth', 2.0)
            plot(gps(1,:), gps(i+1,:), 'r-', 'linewidth', 1.5)
        else
            plot(gps(1,:), -gps(i+1,:)+gps(i+7,:)+gps(i+10,:), 'b-', 'linewidth', 2.0)
            plot(gps(1,:), -gps(i+1,:), 'r-', 'linewidth', 1.5)
        end
        if i == 1
            legend('True', 'Measured')
        end
    end


    figure()
    set(gcf, 'name', strcat([params.name,' GPS velocity']), 'NumberTitle', 'off');
    titles = ["North","East","Altitude"];
    for i=1:3
        subplot(3, 1, i), hold on, grid on
        title(titles(i))
        if i < 3
            plot(gps(1,:), gps(i+4,:)-gps(i+13,:), 'b-', 'linewidth', 2.0)
            plot(gps(1,:), gps(i+4,:), 'r-', 'linewidth', 1.5)
        else
            plot(gps(1,:), -gps(i+4,:)+gps(i+13,:), 'b-', 'linewidth', 2.0)
            plot(gps(1,:), -gps(i+4,:), 'r-', 'linewidth', 1.5)
        end
        if i == 1
            legend('True', 'Measured')
        end
    end
end


if params.mocap_enabled
    mocap = reshape(fread(fopen(strcat(['/tmp/',params.name,'_mocap.log']), 'r'), 'double'), 22, []); % [time;pos_meas;att_meas;pos_truth;att_truth;pos_body2mocap;att_body2mocap]
    figure()
    set(gcf, 'name', strcat([params.name,' Mocap position']), 'NumberTitle', 'off');
    titles = ["North","East","Down"];
    for i=1:3
        subplot(3, 1, i), hold on, grid on
        title(titles(i))
        plot(mocap(1,:), mocap(i+8,:), 'b-', 'linewidth', 2.0)
        plot(mocap(1,:), mocap(i+1,:), 'r-', 'linewidth', 1.5)
        if i == 1
            legend('True', 'Measured')
        end
    end
    
    figure()
    set(gcf, 'name', strcat([params.name,' Mocap attitude']), 'NumberTitle', 'off');
    titles = ["w","x","y","z"];
    for i=1:4
        subplot(4, 1, i), hold on, grid on
        title(titles(i))
        plot(mocap(1,:), mocap(i + 11, :), 'linewidth', 2.0)
        plot(mocap(1,:), mocap(i + 4, :), 'r-', 'linewidth', 1.5)
        if i == 1
            legend('True', 'Measured')
        end
    end
end


if params.camera_enabled
    pix = reshape(fread(fopen(strcat(['/tmp/',params.name,'_camera.log']), 'r'), 'double'), 3*params.camera_max_features+1, []); % [pix_x;pix_y;pix_id]
end
