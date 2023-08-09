% =========================================================================
% Implementation of the UAV-SFL algorithm
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

function [t,f,P,p,b,q,obj_cur,conv_cur] = UAV_SFL_revised(sim_para,post_UE)
    % initialize feasible postions
    b = 0.5*ones(1,sim_para.K) * sim_para.B / sim_para.K;
    p = 0.1*ones(1,sim_para.K) * sim_para.Pmax_User;
    q = [mean(post_UE(:,1)), mean(post_UE(:,2))];
    u = zeros(1,sim_para.K);
    for k = 1:sim_para.K
        u(k) = (sim_para.H^2 ...
            + norm(q - post_UE(k,:),2)^2)^(sim_para.alpha/2);
    end
    Pmax = sim_para.Pmax_UAV;
    P = sim_para.Pmax_UAV;  

    % declare variables and parameters
    obj_pre = Inf;
    conv_cur = zeros(1,100);
    flag = 1;
    iter = 0;
    g0 = sim_para.beta0_h / sim_para.n0;
    beta0 = sim_para.eta0 * sim_para.T * sim_para.beta0_h;
    tmax = sim_para.T - sim_para.Nk*sim_para.Ck.*sim_para.Dk./sim_para.fmax;
    
    while flag
        iter = iter + 1;
        u_kap = u;
        p_kap = p;
        b_kap = b;
        
        %_________________________ the 1st block _________________________%
        tmin = sim_para.s ./ (b.*log2(1 + p*g0./(b .* u)));
        t = (tmax + tmin) / 2;
        tmax = t;
        fmin = sim_para.Nk*sim_para.Ck.*sim_para.Dk ./ (sim_para.T-t);
        f = min(max(fmin,sim_para.fmin),sim_para.fmax);
                        
        %_________________________ the 2nd block _________________________%
        cvx_begin 
        variable p(1,sim_para.K) 
        variable inv_p(1,sim_para.K) 
        variable b(1,sim_para.K) 
        variable q(1,2)
        variable E_var(1,sim_para.K)
        variable u(1,sim_para.K)
        variable u_au(1,sim_para.K)
        expressions phi_kap(1,sim_para.K) Rate(1,sim_para.K) 
        expression varphi_kap(1,sim_para.K)       
        
        for k = 1:sim_para.K
            % rate approximation
            tau = b_kap(k);
            x = u_kap(k)/(p_kap(k) * g0);
            y = b_kap(k);
            lambda = 2*tau*log(1+1/(x*y)); 
            mu = tau/(1+x*y); 
            nu = tau^2*log(1+1/(x*y));
            
            pi_kap = 0.25*(u(k)/u_kap(k) + p_kap(k)*inv_p(k)).^2;
            Rate(k) = log2(exp(1))*(lambda ...
                + mu * (2 - pi_kap - b(k)*b_kap(k)^-1) ...
                - nu * inv_pos(b(k)));
            
            % energy approximation
            % varphi_kap(k) = 0.25 * (t(k)*p(k) + inv_P).^2;
            phi_kap(k) = sim_para.eta*P*beta0/u_kap(k) ...
                - sim_para.eta*P*beta0/(u_kap(k)^2)*(u(k) - u_kap(k));            
        end
        
        % minimize (P)
        minimize 0
        subject to
            % constraint (3d)
            p >= sim_para.Pmin_User; 
            p <= sim_para.Pmax_User;
            
            % constraint (3e)
            sum(b) <= sim_para.B;
            b >= 0;
            
            % constraint (3h)
            sum(q.^2) <= sim_para.C^2;
            
            % constraint (15b)
            Rate >= sim_para.s ./ t;
                       
            % constraint (15c)
            % sim_para.zeta*sim_para.Ck.*sim_para.Dk.*f.^2 + t.*p ...
            %    <= phi_kap;
            sim_para.Nk* sim_para.zeta*sim_para.Ck.*sim_para.Dk.*f.^2 ...
                + t.*p <= phi_kap;
            
            % constraint (6d)
            u >= power(u_au, sim_para.alpha/2);
            for k = 1:sim_para.K
                u_au(k) >= sim_para.H^2 + sum((q - post_UE(k,:)).^2);
            end
                      
            % constraints for auxiliary variables
            for k = 1:sim_para.K
                [p(k) 1;1 inv_p(k)] == semidefinite(2);
            end
        
        cvx_end       
        
        %_________________________ the 3rd block _________________________%
        % E = sim_para.zeta*sim_para.Ck.*sim_para.Dk.*f.^2 + t.*p;
        E = sim_para.Nk*sim_para.zeta*sim_para.Ck.*sim_para.Dk.*f.^2 + t.*p;
        Pmin = max(E.*u / (sim_para.eta*beta0));
        % P = min((Pmax + Pmin)/2,2*Pmin);
        P = min((Pmax + Pmin)/2,sim_para.Pmax_UAV);
        Pmax = P;
        
        % check the stopping tolerance
        obj_cur = P;
        conv_cur(iter) = obj_cur;
        Tol = abs(obj_cur - obj_pre) / obj_pre;
        obj_pre = obj_cur;
        fprintf('Tolerance at iteration-%d is %f\n', iter, Tol);
        if (Tol <= sim_para.Tol)
            flag = 0;
        end
    end
    conv_cur = conv_cur(1:iter);
end


