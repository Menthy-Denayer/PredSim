function [mat,diff_mat_q,diff2_mat_q] = n_art_mat_3(q, order)
% This function returns the polynomials for approximating the muscle-tendon
% lengths, velocities and moment-arms.
%
% Authors: Original code from Wouter Aerts, adapted by Antoine Falisse
% Date: 12/19/2018
% 
% Adapted to 6 degrees of freedom by Dhruv Gupta on May 12, 2022
%
% Adapted to 2nd partial derivative by Menthy Denayer on May 29, 2026
n_dof = length(q(1,:));
nr_points = length(q(:,1));
q_all = zeros(nr_points, 6);
for dof_nr = 1:n_dof
    q_all(:,dof_nr) = q(:,dof_nr);
end
q = q_all;

nr_coefficients = 0;
for n_q1 = 0:order
    if n_dof<2
        n_q2s = 0;
    else
        n_q2s = 0:order-n_q1;
    end
    for n_q2 = n_q2s
        if n_dof<3
            n_q3s = 0;
        else
            n_q3s = 0:order-n_q1-n_q2;
        end
        for n_q3 = n_q3s
            if n_dof<4
                n_q4s = 0;
            else
                n_q4s = 0:order-n_q1-n_q2-n_q3;
            end
            for n_q4 = n_q4s
                if n_dof<5
                    n_q5s = 0;
                else
                    n_q5s = 0:order-n_q1-n_q2-n_q3-n_q4;
                end
                for n_q5 = n_q5s
                    if n_dof<6
                        n_q6s = 0;
                    else
                        n_q6s = 0:order-n_q1-n_q2-n_q3-n_q4-n_q5;
                    end
                    for n_q6 = n_q6s
                        nr_coefficients = nr_coefficients + 1;
                    end
                end
            end
        end
    end
end
mat = zeros(nr_points, nr_coefficients);
diff_mat_q = zeros(nr_points, nr_coefficients, n_dof);
diff2_mat_q = zeros(nr_points, nr_coefficients, n_dof^2);                   % second partial derivative, Menthy

coeff_nr = 1;
for n_q1 = 0:order
    if n_dof<2
        n_q2s = 0;
    else
        n_q2s = 0:order-n_q1;
    end
    for n_q2 = n_q2s
        if n_dof<3
            n_q3s = 0;
        else
            n_q3s = 0:order-n_q1-n_q2;
        end
        for n_q3 = n_q3s
            if n_dof<4
                n_q4s = 0;
            else
                n_q4s = 0:order-n_q1-n_q2-n_q3;
            end
            for n_q4 = n_q4s
                if n_dof<5
                    n_q5s = 0;
                else
                    n_q5s = 0:order-n_q1-n_q2-n_q3-n_q4;
                end
                for n_q5 = n_q5s
                    if n_dof<6
                        n_q6s = 0;
                    else
                        n_q6s = 0:order-n_q1-n_q2-n_q3-n_q4-n_q5;
                    end
                    for n_q6 = n_q6s
                        mat(:,coeff_nr) = q(:,1).^n_q1.*q(:,2).^n_q2.*q(:,3).^n_q3.*q(:,4).^n_q4.*q(:,5).^n_q5.*q(:,6).^n_q6;

                        diff_mat_q1 = n_q1*q(:,1).^(n_q1-1).*q(:,2).^n_q2.*q(:,3).^n_q3.*q(:,4).^n_q4.*q(:,5).^n_q5.*q(:,6).^n_q6;
                        diff_mat_q2 = q(:,1).^n_q1.*n_q2.*q(:,2).^(n_q2-1).*q(:,3).^n_q3.*q(:,4).^n_q4.*q(:,5).^n_q5.*q(:,6).^n_q6;
                        diff_mat_q3 = q(:,1).^n_q1.*q(:,2).^n_q2.*n_q3.*q(:,3).^(n_q3-1).*q(:,4).^n_q4.*q(:,5).^n_q5.*q(:,6).^n_q6;
                        diff_mat_q4 = q(:,1).^n_q1.*q(:,2).^n_q2.*q(:,3).^n_q3.*n_q4.*q(:,4).^(n_q4-1).*q(:,5).^n_q5.*q(:,6).^n_q6;
                        diff_mat_q5 = q(:,1).^n_q1.*q(:,2).^n_q2.*q(:,3).^n_q3.*q(:,4).^n_q4.*n_q5.*q(:,5).^(n_q5-1).*q(:,6).^n_q6;
                        diff_mat_q6 = q(:,1).^n_q1.*q(:,2).^n_q2.*q(:,3).^n_q3.*q(:,4).^n_q4.*q(:,5).^n_q5.*n_q6.*q(:,6).^(n_q6-1);

                        for dof_nr = 1:n_dof
                            eval(['diff_mat_q(:,coeff_nr,dof_nr) = diff_mat_q', num2str(dof_nr), ';']);
                        end

                        % second partial derivative, added by Menthy
                        n_list = [n_q1, n_q2, n_q3, n_q4, n_q5, n_q6];
                        dof_nr = 1;
                        for i = 1:n_dof
                            for j = 1:n_dof
                                if(i == j)
                                    is_col_i_j = zeros(1, 6); is_col_i_j(i) = 1;
                                    Nprod = 5;
                                    diff2_mat_q(:,coeff_nr,dof_nr) = n_list(i) * (n_list(i)-1) * q(:,i).^(n_list(i)-2);
                                else
                                    is_col_i_j = zeros(1, 6); is_col_i_j([i, j]) = 1;
                                    Nprod = 4;
                                    diff2_mat_q(:,coeff_nr,dof_nr) = n_list(i) * n_list(j) * q(:,i).^(n_list(i)-1) .* q(:,j).^(n_list(j)-1);
                                end
                                
                                % compute product
                                prod = 1;
                                prod_idxs = 1:6; prod_idxs = prod_idxs(~is_col_i_j);
                                for k = 1:Nprod
                                    prod = prod .* q(:, prod_idxs(k)).^n_list(prod_idxs(k));
                                end

                                diff2_mat_q(:,coeff_nr,dof_nr) = diff2_mat_q(:,coeff_nr,dof_nr) .* prod;

                                dof_nr = dof_nr + 1;
                            end
                        end

                        coeff_nr = coeff_nr + 1;
                    end
                end
            end
        end
    end
end
end