%% Plot the first camera image
figure(f); clf; f=f+1; hold on;
set(gcf, 'name', 'Camera', 'NumberTitle', 'off');
title('First Camera Image');
set(gca, 'YDir', 'reverse')
axis([0 640 0 480])
pix1 = pix(pix(:,3) >= 0);
pix1 = reshape(pix1(2:end),3,[]);
plot(pix1(1,:), pix1(2,:), 'b.');
