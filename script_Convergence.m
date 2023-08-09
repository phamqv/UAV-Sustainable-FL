% =========================================================================
% Convergence of the proposed UAV-SFL algorithm
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

noReal = 1;
sim_para = paras_sim;
sim_para.Tol = 1e-3;

conv_cur_All = cell(1,noReal);
post_UE_All = cell(1,noReal);

for i = 1:noReal

    post_UE = positions(sim_para);
    post_UE_All{1,i} = post_UE;
    
    [t,f,P,p,b,q,obj_cur,conv_cur] = UAV_SFL_revised(sim_para,post_UE);
    conv_cur_All{1,i} = conv_cur;

end

% this figure applies for [noReal = 1]
figure(2)
hold on;
plot(1:length(conv_cur),conv_cur,'b-^','linewidth',3.0,'markers',12,'MarkerIndices',1:8:length(conv_cur));
hold off;
set(gca,'FontSize',25,'XLim',[1 length(conv_cur)]);
xticks = 1:21:length(conv_cur);
xticklabels({'1','22','43','64','85','106','127','148'});
xlabel('Iteration'); 
ylabel('UAVs Transmit Power (W)');
legend('UAV-SFL')
box on;

% save script_Convergence.mat