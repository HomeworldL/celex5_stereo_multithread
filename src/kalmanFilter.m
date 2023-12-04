


function output = kalmanFilter(data, Q, R, N)
 
    if ~exist('Q', 'var')
        Q = 0.00009;
    end
    if ~exist('R', 'var')
        R = 1;
    end
    if ~exist('N', 'var')
        N = 1;
    end

    X = 0;
    P = 1;
    A = 1;
    H = 1;

    output = zeros(size(data));

    for ii = N + 1 : length(data)
       X_k = A * X;
       P_k = A * P * A' + Q;
       Kg = P_k * H' / (H * P_k * H' + R);
       z_k = data(ii - round(rand() * N));
       X = X_k + Kg * (z_k - H * X_k);
       P = (1 - Kg*H) * P_k;
       output(ii) = X;
    end
 
end




% Ö÷º¯Êý
% %%
% data = load('D:\Desktop\disp2.txt');
% out = kalmanFilter(data);
% 
% % out(1:3,1) = data(1:3,1);
% figure;
% plot(data);
% hold on;
% plot(out);
% 
% 
% %%
% out1 = load('D:\Desktop\result.txt');
% figure;
% plot(out - out1);
% 
% figure;
% plot(data);
% hold on;
% plot(out1);