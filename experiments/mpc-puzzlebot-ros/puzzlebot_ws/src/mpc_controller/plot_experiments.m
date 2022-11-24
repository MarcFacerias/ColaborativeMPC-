close all 
sample = importSamples(); 
trajectorys = importRefS(); 

error = sqrt((trajectorys(:,1) - sample(:,1)).^2  + (trajectorys(:,2) - sample(:,2)).^2);
em = mean(error); 
% figure()
% plot(sample(:,1),"r")
% hold on 
% plot(trajectorys(:,1),"b")
% hold off 
% 
% figure()
% plot(sample(:,2),"r")
% hold on 
% plot(trajectorys(:,2),"b")
% hold off
% 
% figure()
% plot(sample(:,3),"r")
% hold on 
% plot(trajectorys(:,3),"b")
% hold off 

figure()
subplot(1,2,1)
grid on 
plot(sample(:,1),sample(:,2),"r")
hold on
title("Pose tracking", 'FontSize', 16)
grid on 
plot(trajectorys(:,1),trajectorys(:,2),"b")
hold off 
legend("Pose","Reference", 'FontSize', 14)
xlabel("x(m)", 'FontSize', 14)
ylabel("y(m)", 'FontSize', 14)

subplot(1,2,2)
plot(sample(:,3),"r")
hold on
title("Heading tracking", 'FontSize', 16)
grid on 
plot(trajectorys(:,3),"b")
hold off
legend("Heading","Reference", 'FontSize', 14)
xlabel("Sample number", 'FontSize', 14)
ylabel("Heading (rad)", 'FontSize', 14)

% figure()
% plot(error)