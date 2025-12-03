

%% 
tic
xi = [.5,.707,.5]';
xf = [-1,0,0]';




N = 100;
step_size = norm(xf - xi) / N;
trajectory = xi;
straight_line = zeros(N+1, 3); % Initialize straight line trajectory array
for j =1 : 10000

   
    for i = 1:N+1
        t = (i-1) / N; 
        straight_line(i, :) = (1 - t) * xi' + t * xf';
    end
    
    
    disp(straight_line);
    second_point = straight_line(2, :)';
    
    % Define Lagrange multiplier problem
    fun = @(x) norm(x - second_point); % Objective function: Distance from current point to final point
    nonlcon = @(x) deal([], norm(x)^2 - 1); % Constraint: lies on the sphere

    x0 = xi;
    
    % Solve Lagrange multiplier problem using fmincon
    options = optimoptions('fmincon', 'Display', 'off');
    x_solution = fmincon(fun, x0, [], [], [], [], [], [], nonlcon, options);
    

    disp(['x_solution: ', num2str(x_solution(1))]);
    disp(['y_solution: ', num2str(x_solution(2))]);
    disp(['z_solution: ', num2str(x_solution(3))]);

    xi = x_solution;
    trajectory = [trajectory,xi];
    if norm(xi-xf) < 10e-3
        break
    end
end


% Plot the trajectory
figure;
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:), '-o', 'LineWidth', 2); % Plot the trajectory
hold on;
plot3(xi(1), xi(2), xi(3), 'ro', 'MarkerSize', 10); % Plot the final point
plot3(trajectory(1,1), trajectory(2,1), trajectory(3,1), 'go', 'MarkerSize', 10); % Plot the initial point
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
legend('Trajectory', 'Final Point', 'Initial Point');
title('Trajectory from Initial Point to Final Point');


axis equal;

% Add a sphere to the plot
[x_sphere, y_sphere, z_sphere] = sphere; 
sphere_radius = 1; 
x_sphere = x_sphere * sphere_radius; 
y_sphere = y_sphere * sphere_radius; 
z_sphere = z_sphere * sphere_radius; 
surf(x_sphere, y_sphere, z_sphere, 'FaceAlpha', 0.3, 'EdgeColor', 'none'); 



xlim([-1, 1]);
ylim([-1, 1]);
zlim([-1, 1]);



[phi,theta, rho] = cart2sph(trajectory(1,:), trajectory(2,:), trajectory(3,:));
geodesic = 0;
flat_geodesic = 0;
for i = 1:length(theta)-1
    geodesic = geodesic + rho(i) * sqrt((abs(theta(i+1)) - abs(theta(i)))^2 + ((phi(i+1) - phi(i))^2 * sin(theta(i)))^2);
    flat_geodesic = flat_geodesic + norm(trajectory(:,i+1)- trajectory(:,i));
end

toc
%% % Given Cartesian coordinates of initial and final points
xi = [-.67, -.43, sqrt(1-.43^2-.67^2)]';
xf = [.67, -.43,sqrt(1-.43^2-.67^2)]';
R = 1;
[phi_i, theta_i, ~] = cart2sph(xi(1), xi(2), xi(3));
[phi_f, theta_f, ~] = cart2sph(xf(1), xf(2), xf(3));
angle = acos(sin(theta_i)*sin(theta_f) + cos(theta_i)*cos(theta_f)*cos(phi_i - phi_f));

geodesic_distance = R * angle;

disp(['Geodesic Distance: ', num2str(geodesic_distance)]);



