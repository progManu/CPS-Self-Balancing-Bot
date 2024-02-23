%{
clear;
clc;

load("fResults.mat");                                                      % loading falsification process results

idxs_np1 = rob1_values < 0;                                                % find indexes that do not satisfy STL first formula
idxs_np2 = rob2_values < 0;                                                % find indexes that do not satisfy STL second formula

np1s = np1.*idxs_np1;                                                      % get only noise values that satisfy STL formulae
np2s = np2.*idxs_np2;

len = size(np1s);

x = linspace(0, len(1)-1, len(1));                                         % generate x axis

figure(1);                                                                 % 2 figure plot
subplot(2, 1, 1);
scatter(x, np1s, "filled");
title('x additive noise');
grid("on");
subplot(2, 1, 2);
scatter(x, np2s, "filled");
title('\theta additive noise');
grid("on");
%}
clear;
clc;

load("fResults.mat");                                                      % loading falsification process results

idxs_np1 = rob1_values < 0;                                                % find indexes that do not satisfy STL first formula
idxs_np2 = rob2_values < 0;                                                % find indexes that do not satisfy STL second formula

np1s = np1.*idxs_np1;                                                      % get only noise values that satisfy STL formulae
np2s = np2.*idxs_np2;

len = size(np1s);

x = linspace(0, len(1)-1, len(1));                                         % generate x axis

figure(1);                                                                 % 2 figure plot
subplot(2, 1, 1);
scatter(x, np1s, 'filled', 'SizeData', 100);                                % Change marker size to 50
title('x additive noise');
grid('on');
subplot(2, 1, 2);
scatter(x, np2s, 'filled', 'SizeData', 100);                                % Change marker size to 50
title('\theta additive noise');
grid('on');
