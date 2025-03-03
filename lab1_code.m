% Simulation Time
T = 50;
kk = 0:1:T;

% Regressors
w1 = @(k) [ sin(0.25 * pi * k) ; cos(0.25 * pi * k) ];
w2 = @(k) [ sin(0.25 * pi * k) ; sin(0.25 * pi * k) ];

% Unknown Parameter (DO NOT USE IN YOUR DESIGN!)
psi = [ 4 ; 2 ];

% Constant Gain \bar{\gamma}
g = 0.5;

% 初始化估计参数 psik 和误差 e(k)
psih = NaN(2, length(kk));  % 估计值
psih(:, 1) = [0; 0];        % 初值设为零向量
e = NaN(1, length(kk));     % 误差

%% 自适应控制仿真
for idx = 1:(length(kk) - 1)
    % 计算测量的回归器 w(k) 和真实信号 r(k)
    w = w1(kk(idx));        
    r = psi.' * w;         % 参考信号 (未知但用于计算误差)
    lr = 0.5/(1+norm(w)^2);
    % 计算误差 e(k)
    e(idx) = (psih(:, idx).' * w) - r;
    
    % 自适应更新法则
    psih(:, idx + 1) = psih(:, idx) - lr * e(idx) * w;
end

% 计算最后一步误差
e(end) = (psih(:, end).' * w1(kk(end))) - (psi.' * w1(kk(end)));

%% 结果展示
% Display recovered value
psih(:, end)
% Plot
plot_staticEM(kk, psih, e)


%% 初始化估计参数 psik 和误差 e(k)
psih = NaN(2, length(kk));  % 估计值
psih(:, 1) = [0; 0];        % 初值设为零向量
e = NaN(1, length(kk));     % 误差

%% 自适应控制仿真
for idx = 1:(length(kk) - 1)
    % 计算测量的回归器 w(k) 和真实信号 r(k)
    w = w2(kk(idx));        
    r = psi.' * w;         % 参考信号 (未知但用于计算误差)
    lr = 0.5/(1+norm(w)^2);

    % 计算误差 e(k)
    e(idx) = (psih(:, idx).' * w) - r;
    
    % 自适应更新法则
    psih(:, idx + 1) = psih(:, idx) - lr * e(idx) * w;
    
end

% 计算最后一步误差
e(end) = (psih(:, end).' * w2(kk(end))) - (psi.' * w2(kk(end)));

%% 结果展示
% Display recovered value
psih(:, end)
% Plot
plot_staticEM(kk, psih, e)
%fprintf('Step %d: e = %.4f, w = [%.4f, %.4f]\n', idx, e(idx), w(1), w(2));
%N = 8;
%W = zeros(2,2);
%for i = 0:N-1
    %w = w2(i);
    %W = W + w * w';
%end
%W = W / N;
%eigenvalues = eig(W);
%disp('Eigenvalues of W(k,N):');
%disp(eigenvalues);
%% Adaptation in MRAC and Dynamic-EM
%Q2:
A_r = [0, 1; -0.04, 0.4];

% Define the identity matrix I for the equation
I_matrix = eye(size(A_r));

% Solve the discrete-time Lyapunov equation: A_r' * P * A_r - P = -I
P = dlyap(A_r', I_matrix);

% Display the computed matrix P
disp('The solution matrix P is:');
disp(P);

% Verify the solution: Compute A_r' * P * A_r - P
verification = A_r' * P * A_r - P;
disp('Verification (should be close to -I):');
disp(verification);

%% Plant
sys.A = [ 0 1 ; 0 0 ]; sys.B = [ 0 ; 1 ]; % Reference Model
sys.Ar = [ 0 1 ; -0.04 0.4 ]; sys.Br = [ 0 ; 1.5 ]; % Known
% Known
% Known
% Unknown
%% Q6:States + Initial Conditions---> w = w1
x = NaN(2, length(kk)); 
xr = NaN(2, length(kk)); 
x_e_hat = NaN(2,length(kk));
x_e = NaN(2,length(kk));
psih = NaN(2, length(kk)); 
g = 0.3;
b = sys.Br(2) / sys.B(2);
K = [-0.04 0.4];
% Augmented Regressor States
Z1 = NaN(2, length(kk)); 
Z2 = NaN(2, length(kk)); 
Z1(:, 1) = [0; 0];
Z2(:, 1) = [0; 0];
% TODO: Add more states as needed
x(:, 1) = [ 0 ; 0 ];
xr(:, 1) = [ 0 ; 0 ];
psih(:, 1) = [ 0 ; 0 ];
x_e_hat(:,1) = [0;0];
x_e(:,1) = [0;0];
% Simulate Dynamics

for idx = 1:(length(kk) - 1)
    % Measurements (w, x, xr)
    w = w1(kk(idx)); % Regressor
   
    % TODO: Controller
    x_current = x(:, idx);
    xr_current = xr(:, idx);
    psi_hat = psih(:, idx);
    
    % Compute Augmented Regressor w_a(k)
    Z1(:, idx + 1) = sys.A * Z1(:, idx) + sys.B * w(1);
    Z2(:, idx + 1) = sys.A * Z2(:, idx) + sys.B * w(2);
    W_a = [sys.B' * P * Z1(:, idx); sys.B' * P * Z2(:, idx)];
    
    lr = g/(1+norm(W_a)^2);
    u = K * x_current + w'*psi_hat;
    % Update Plant
    x(:, idx + 1) = sys.A * x(:, idx) + sys.B * u;
    % Update Reference Model
    xr(:, idx + 1) = sys.Ar * xr(:, idx) + sys.Br * psi.' * w;
    % Compute Error
    %x_e = x(:, idx) - xr(:, idx);
    x_e(:,idx+1) = sys.A * x_e(:,idx) + sys.B * (-psih(:,idx) + b * psi)' * w;


    e = sys.B' * P * x_e(:,idx);
    x_e_hat(:,idx + 1) = sys.A * x_e_hat(:,idx) + sys.B * psi_hat' * w;
    % Compute Augmented Error e_a(k) using Equation (72)
    %e_a = sys.B' * P * x_e + psi_hat' * W_a;
    %e_a = sys.B' * P * x_e + (psi_hat - b * psi).' * W_a;
    e_a = e - sys.B' * P * x_e_hat(:,idx)  + psi_hat' * W_a;
    %e_a = sys.B' * P * x_e + (psih(:, idx) - 1.5 * psi).' * W_a;

    % Adaptation Law for Parameter Update
    psih(:, idx + 1) = psih(:, idx) + lr*e_a*W_a;
end

% Display recovered value
psih(:, end)
% Plot Simulation
plot_dynamicEM(kk, psih, x - xr)


%% Q6:States + Initial Conditions---> w = w2
x = NaN(2, length(kk)); 
xr = NaN(2, length(kk)); 
x_e_hat = NaN(2,length(kk));
x_e = NaN(2,length(kk));
psih = NaN(2, length(kk)); 
g = 0.3;
b = sys.Br(2) / sys.B(2);
K = [-0.04 0.4];
% Augmented Regressor States
Z1 = NaN(2, length(kk)); 
Z2 = NaN(2, length(kk)); 
Z1(:, 1) = [0; 0];
Z2(:, 1) = [0; 0];
% TODO: Add more states as needed
x(:, 1) = [ 0 ; 0 ];
xr(:, 1) = [ 0 ; 0 ];
psih(:, 1) = [ 0 ; 0 ];
x_e_hat(:,1) = [0;0];
x_e(:,1) = [0;0];
% Simulate Dynamics

for idx = 1:(length(kk) - 1)
    % Measurements (w, x, xr)
    w = w2(kk(idx)); % Regressor
   
    % TODO: Controller
    x_current = x(:, idx);
    xr_current = xr(:, idx);
    psi_hat = psih(:, idx);
    
    % Compute Augmented Regressor w_a(k)
    Z1(:, idx + 1) = sys.A * Z1(:, idx) + sys.B * w(1);
    Z2(:, idx + 1) = sys.A * Z2(:, idx) + sys.B * w(2);
    W_a = [sys.B' * P * Z1(:, idx); sys.B' * P * Z2(:, idx)];
    
    lr = g/(1+norm(W_a)^2);
    u = K * x_current + w'*psi_hat;
    % Update Plant
    x(:, idx + 1) = sys.A * x(:, idx) + sys.B * u;
    % Update Reference Model
    xr(:, idx + 1) = sys.Ar * xr(:, idx) + sys.Br * psi.' * w;
    % Compute Error
    %x_e = x(:, idx) - xr(:, idx);
    x_e(:,idx+1) = sys.A * x_e(:,idx) + sys.B * (-psih(:,idx) + b * psi)' * w;


    e = sys.B' * P * x_e(:,idx);
    x_e_hat(:,idx + 1) = sys.A * x_e_hat(:,idx) + sys.B * psi_hat' * w;
    % Compute Augmented Error e_a(k) using Equation (72)
    %e_a = sys.B' * P * x_e + psi_hat' * W_a;
    %e_a = sys.B' * P * x_e + (psi_hat - b * psi).' * W_a;
    e_a = e - sys.B' * P * x_e_hat(:,idx)  + psi_hat' * W_a;
    %e_a = sys.B' * P * x_e + (psih(:, idx) - 1.5 * psi).' * W_a;

    % Adaptation Law for Parameter Update
    psih(:, idx + 1) = psih(:, idx) + lr*e_a*W_a;
end

% Display recovered value
psih(:, end)
% Plot Simulation
plot_dynamicEM(kk, psih, x - xr)


%% Q7:States + Initial Conditions---> w = w1

x = NaN(2, length(kk)); 
xr = NaN(2, length(kk)); 
x_e_hat = NaN(2,length(kk));
x_e = NaN(2,length(kk));
psih = NaN(2, length(kk)); 
g = 0.8;
b = sys.Br(2) / sys.B(2);
K = [-0.04 0.4];
% Augmented Regressor States
Z1 = NaN(2, length(kk)); 
Z2 = NaN(2, length(kk)); 
Z1(:, 1) = [0; 0];
Z2(:, 1) = [0; 0];
% TODO: Add more states as needed
x(:, 1) = [ 0 ; 0 ];
xr(:, 1) = [ 0 ; 0 ];
psih(:, 1) = [ 0 ; 0 ];
x_e_hat(:,1) = [0;0];
x_e(:,1) = [0;0];
% Simulate Dynamics

for idx = 1:(length(kk) - 1)
    % Measurements (w, x, xr)
    w = w2(kk(idx)); % Regressor
   
    % TODO: Controller
    x_current = x(:, idx);
    xr_current = xr(:, idx);
    psi_hat = psih(:, idx);
    
    % Compute Augmented Regressor w_a(k)
    Z1(:, idx + 1) = sys.A * Z1(:, idx) + sys.B * w(1);
    Z2(:, idx + 1) = sys.A * Z2(:, idx) + sys.B * w(2);
    W_a = [sys.B' * P * Z1(:, idx); sys.B' * P * Z2(:, idx)];
    
    lr = g/(1+norm(W_a)^2);
    u = K * x_current + w'*psi_hat;
    % Update Plant
    x(:, idx + 1) = sys.A * x(:, idx) + sys.B * u;
    % Update Reference Model
    xr(:, idx + 1) = sys.Ar * xr(:, idx) + sys.Br * psi.' * w;
    % Compute Error
    %x_e = x(:, idx) - xr(:, idx);
    x_e(:,idx+1) = sys.A * x_e(:,idx) + sys.B * (-psih(:,idx) + b * psi)' * w;


    e = sys.B' * P * x_e(:,idx);
    x_e_hat(:,idx + 1) = sys.A * x_e_hat(:,idx) + sys.B * psi_hat' * w;
    % Compute Augmented Error e_a(k) using Equation (72)
    %e_a = sys.B' * P * x_e + psi_hat' * W_a;
    %e_a = sys.B' * P * x_e + (psi_hat - b * psi).' * W_a;
    e_a = e - sys.B' * P * x_e_hat(:,idx)  + psi_hat' * W_a;
    %e_a = sys.B' * P * x_e + (psih(:, idx) - 1.5 * psi).' * W_a;

    % Adaptation Law for Parameter Update
    psih(:, idx + 1) = psih(:, idx) + lr*e_a*W_a;
end

% Display recovered value
psih(:, end)
% Plot Simulation
plot_dynamicEM(kk, psih, x - xr)