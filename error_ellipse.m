function [x, y] = error_ellipse(mean, covariance)
    % Calculate the eigenvectors and eigenvalues
    [eigenvec, eigenval] = eig(covariance);
    
    % Get the index of the largest eigenvector
    [largest_eigenvec_ind_c, r] = find(eigenval == max(max(eigenval)));
    largest_eigenvec = eigenvec(:, largest_eigenvec_ind_c);
    
    % Get the largest eigenvalue
    largest_eigenval = max(max(eigenval));
    
    % Get the smallest eigenvector and eigenvalue
    if largest_eigenvec_ind_c == 1
        smallest_eigenval = eigenval(2,2);
        smallest_eigenvec = eigenvec(:,2);
    else
        smallest_eigenval = eigenval(1,1);
        smallest_eigenvec = eigenvec(1,:);
    end
    
    % Calculate the angle between the x-axis and the largest eigenvector
    angle = atan2(largest_eigenvec(2), largest_eigenvec(1));
    
    % This angle is between -pi and pi.
    % Let's shift it such that the angle is between 0 and 2pi
    if(angle < 0)
        angle = angle + 2*pi;
    end
    
    % Get the 95% confidence interval error ellipse
    chisquare_val = 5.991;
    theta_grid = linspace(0, 2*pi);
    phi = angle;
    X0 = mean(1);
    Y0 = mean(2);
    a = sqrt(chisquare_val * largest_eigenval);
    b = sqrt(chisquare_val * smallest_eigenval);
    
    % The ellipse in x and y coordinates
    ellipse_x_r = a * cos(theta_grid);
    ellipse_y_r = b * sin(theta_grid);
    
    % Define a rotation matrix
    R = [cos(phi) sin(phi); -sin(phi) cos(phi)];
    
    % Rotate the ellipse to some angle phi
    r_ellipse = [ellipse_x_r; ellipse_y_r]' * R;
    
    % Draw the error ellipse
    x = r_ellipse(:,1) + X0;
    y = r_ellipse(:,2) + Y0;
end