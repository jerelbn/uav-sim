function animate_img(speed, params)

% Load data such that each column contains [t;pix1;...;pixN]
pix = reshape(fread(fopen(strcat(['/tmp/',params.name,'_camera.log']), 'r'), 'double'), 1+2*params.camera_max_features, []);

persistent image_handle

for i =1:speed:size(pix,2)
    pix_img = pix(~isnan(pix(:,i)),i);
    pix_img = reshape(pix_img(2:end),2,[]);
    if i == 1
        figure()
        image_handle = plot(pix_img(1,:), pix_img(2,:), 'b.');
        axis equal, axis ij
        set(gcf, 'name', 'Image Animation', 'NumberTitle', 'off')
        set(gca,'XAxisLocation','top','XLim',[0,640],'YLim',[0,480]);
        xlabel('x (pixels)')
        ylabel('y (pixels)')
        title('Camera View')
    else
        set(image_handle, 'XData', pix_img(1,:), 'YData', pix_img(2,:))
        drawnow
    end
end
