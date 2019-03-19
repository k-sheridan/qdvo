biases = [];
times = [];

for idx = (1:length(vio.graph.FrameContainer))
    biases = [biases, vio.graph.FrameContainer{idx}.imustate.biases];
    times = [times, vio.graph.FrameContainer{idx}.t];
end

times = times - times(1);

hold on
plot(times, biases(4, :))
plot(times, biases(5, :))
plot(times, biases(6, :))

legend('gyroBias_{x}', 'gyroBias_{y}', 'gyroBias_{z}')
title('Estimated Gyro Biases over Time');
xlabel('time (s)')
ylabel('rad/s')