function animate_img(directory, speed)
    
    % Load pixel measurements
    file = fopen(strcat(directory,'camera.bin'), 'r');
    pix = fread(file, 'double');
    pix = reshape(pix, 15001, []);

    persistent image_handle

    for i =1:speed:size(pix,2)
        pix_img = pix(pix(:,i) >= 0,i);
        pix_img = reshape(pix_img(2:end),3,[]);
        if i == 1
            figure(1e6+1), clf
            image_handle = plot(pix_img(1,:), pix_img(2,:), 'b.');
            axis equal, axis ij
            set(gcf, 'name', 'Image Animation', 'NumberTitle', 'off')
            set(gca,'XAxisLocation','top','XLim',[0,641],'YLim',[0,481]);
            xlabel('x (pixels)')
            ylabel('y (pixels)')
            title('Camera View')
        else
            set(image_handle, 'XData', pix_img(1,:), 'YData', pix_img(2,:))
            drawnow
        end
    end

end
