clear;
clc;

load('vResults.mat')                                                       % load results

idxs = find((rob1_values > 0 & rob2_values > 0) > 0)';                     % find indexes that satisfy STL formulae

Kps = Kg_p(idxs);                                                          % getting Kp values
Kis = Kg_i(idxs);                                                          % getting Ki values
Kds = Kg_d(idxs);                                                          % getting Kd values                

hold on

plot(Kps', 'LineWidth', 2);                                                % plotting results
plot(Kis', 'LineWidth', 2);
plot(Kds', 'LineWidth', 2);

xlabel('index');
ylabel('value');

legend('Kp', 'Ki', 'Kd');