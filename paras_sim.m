% =========================================================================
% Simulation settings
% =========================================================================
% Related Journal Reference: 
% [1] Q.-V. Pham, M. Zeng, R. Ruby, T. Huynh-The, and W.-J. Hwang, 
%     "UAV communications for sustainable federated learning,” 
%      IEEE Transactions on Vehicular Technology, 
%      vol. 70, no. 4, pp. 3944–3948, Apr. 2021.
% [2] 
%
% COPYRIGHT NOTICE:
% All rights belong to Quoc-Viet Pham (email: vietpq90@gmail.com).
% This simulation code can be freely modified and distributed with the 
% original copyright notice. 
% Using this code with your own risk.
%
% Author: QUOC-VIET PHAM
% E-Mail: vietpq90@gmail.com
% Created: 2020 Nov 11
% Current: 2021 Jun 09
% =========================================================================
function sim_para = paras_sim
    % default parameters
    sim_para.K = 25; 
    sim_para.alpha = 2.01;
    sim_para.C = 50;
    sim_para.H = 10;
    sim_para.B = 20e6;
    % sim_para.n0 = db2lin(-174 - 30);
    sim_para.n0 = db2lin(-120 - 30);    
    % sim_para.beta0_h = 1.42e-4*2.2846;
    sim_para.beta0_h = db2lin(-3);
    % sim_para.beta0_h = 1.42e-4;
    sim_para.Pmax_UAV = db2lin(36-30);
    sim_para.Pmin_UAV = 0;
    sim_para.Pmax_User = db2lin(10-30);
    sim_para.Pmin_User = 0;
    sim_para.eta0 = 0.9;
    sim_para.zeta = 1e-28;
    sim_para.fmax = 1.0e9;
    sim_para.fmin = 0.1e9;
    sim_para.s = 100e3;
    load('CDk.mat');
    sim_para.Dk = Dk(1,1:sim_para.K);
    sim_para.Ck = Ck(1,1:sim_para.K);
    % sim_para.Dk = (5 + 5*rand(1,sim_para.K))*1e6;
    % sim_para.Ck = 10 + 10*rand(1,sim_para.K);
    % save('CDk.mat','Dk','Ck');
    sim_para.Nk = 4;
    sim_para.T = 8;
    sim_para.Tol = 1e-2;
    sim_para.rho = 0.5;
    sim_para.div = 20;
    sim_para.eta = 1e0;
end