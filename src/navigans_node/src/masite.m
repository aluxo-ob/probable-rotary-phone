% This file is based on townCentre.m
% Used to generate a coarse camera calibration for cameras at the intersection 
% of 5th and Beechwood

addpath('TOOLBOX_calib/');

fileName = 'Video3';

% select points from the image
if 0
    clf
    clear    
    a = imread( ['~/' fileName '.jpg'] );
    imshow( a ), hold on    
    count = 0; x =[]; y = [];
    
    % Collect pixel coordinates corresponding to points in plane
    while(1)
        [xw, yw, mouse_button]=ginput(1)
        if(mouse_button == 3)
            break
        end
        count = count + 1;
        x( count ) = xw;
        y( count ) = yw;
        plot(xw, yw, 'r+')
        %plot(xw+rand, yw+rand, 'bo')
        
        text(xw+3, yw+2, sprintf('%d', count), 'FontSize', 12, 'FontWeight', 'Bold' )
        title( sprintf('[%.2f, %.2f]', xw, yw ) )
    end
    title( fileName );
    [x; y]
    saveas(1, [fileName '_markers'], 'jpg' );
    fid = fopen([fileName '_data.txt'],'w');
    for k=1:count
        fprintf( fid, '%f %f\n', x(k), y(k));
    end
    fclose( fid );
    return
end

% Perform simple calibration
if 1

    % Obtain matrix A from the data points
    % x are the points in the image plane
    % X are the points in 3D space (world frame)
    %load data.mat
    % Video3
    x = [ 181.684669 272.277003
          179.454704 225.447735
          109.210801 275.621951
           34.506969 270.604530
          154.925087 187.538328
          566.911150 277.294425
          498.897213 193.113240
          554.088850 223.775261
          439.803136 141.824042
          442.590592 232.137631
          383.496516 185.308362
          630.465157 314.088850
          148.235192 294.019164
          443.148084 326.911150
           32.834495 228.792683 ];
    x = x';
    xe = x;
    
    Xutm = [ 8.981081008911 -6.767824649811 0.777583718300
             8.972593307495 -6.840630054474 1.910237550735
            10.428060       -8.081507       0.708104
            11.850910186768 -9.614816665649 0.651222467422
            10.554234504700 -6.439673900604 3.439467430115
             2.702903747559  2.470865726471 0.943239808083
             0.620263814926 -0.830464124680 2.668621540070
            -0.172788679600  0.978258848190 1.771192789078
            -0.276789307594 -3.342656612396 3.896268606186
            -0.455072462559 -3.196919441223 0.776353836060
            -5.858895778656 -8.726755142212 0.808652758598
             4.051758766174  4.836362838745 0.697832226753
            11.008689880371 -5.960782051086 0.686188578606
             7.831201553345  1.200059533119 0.573717236519
            11.900057792664 -9.778165817261 1.865399599075 ];
    Xutm = Xutm';
    X = Xutm;

    % To avoid numeric unstability in UTM, substract offset
    %     X = zeros( 3, 8 );
    %     X(1,:) = Xutm(1,:) - Xutm(1,6);
    %     X(2,:) = Xutm(2,:) - Xutm(2,6);
    %     X(3,:) = Xutm(3,:);
    %     X
    
    % Initial parameter estimate based on direct linear transformation
    n = size(x, 2);
    h = 1;
    for k = 1:n;
        A(h, :)     = [ X(1, k) X(2, k) X(3, k) 1 ...
                        0 0 0 0 ...
                       -x(1, k)*X(1, k) -x(1, k)*X(2, k) ...
                       -x(1, k)*X(3, k) -x(1, k) ];
        A(h + 1, :) = [ 0 0 0 0 ...
                        X(1, k) X(2, k) X(3, k) 1 ...
                       -x(2, k)*X(1, k) -x(2, k)*X(2, k) ...
                       -x(2, k)*X(3, k) -x(2, k) ];
        h = h + 2;
    end
    % Compute camera matrix
    [U S V] = svd(A);
    P = V(:, size(V,2));
    p = P;
    P = reshape(P, 4, 3)';
    
    % Decompose camera matrix
    C = P;
    if ~all(size(C) == [3 4]),
        error('argument is not a 3x4 matrix');
    end
    [u,~,v] = svd(C);
    
    t = v(:,4);
    t = t / t(4);
    t = t(1:3);
    
    M = C(1:3,1:3);
    
    [K, R] = vgg_rq(M);
    
    % normalize K so that lower left is 1
    K = K / K(3,3);
    
    f = K(1, 1);
    s = [1 K(2, 2) / K(1, 1)];
    if f < 0,
        f = -f;
        s = -s;
    end
    fprintf('Focal: %f\n',f);
    fprintf('Center: %f\n', K(1:2,3) );
    fprintf('Skew: %f\n', s);
    
    [R t; 0 0 0 1]
    
    t0 = t;
    R0 = R;
    
    % Attempt to refine parameters using Levenberg-Marquardt
    % ------------------------------------------------------
    % Note that: au = α, av=β, sk= γ in Zhang’s report
    syms u0 v0 au av sk real
    syms tx ty tz wx wy wz real
    syms X Y Z um vm real
    % New to incorporate distortion
    syms rd r2 k1 k2 k3 k4 ud vd xd1 xd2 real
    
    % the intrinsic parameter matrix
    K = [au sk u0 0;
          0 av v0 0;
          0  0  1 0];
    
    % Expression for the rotation matrix based on the Rodrigues formula
    theta = sqrt(wx^2 + wy^2 + wz^2);
    omega = [ 0 -wz  wy;
             wz   0 -wx;
            -wy  wx   0;];
    R = eye(3) + (sin(theta)/theta)*omega + ((1-cos(theta))/theta^2)*(omega*omega);
    
    % Expression for the translation vector
    t = [tx; ty; tz];
    
    % perspective projection of the model point (X,Y)
    uvs = K*[R t; 0 0 0 1]*[X; Y; Z; 1];
    ud  = uvs(1) / uvs(3);
    vd  = uvs(2) / uvs(3);
    
    % Account for radial distortion
    r2  = ud^2 + vd^2;
    rd  = 1 + k1*r2 + k2*r2^2;
    xd1 = rd*ud + 2*k3*ud*vd + k4*(r2 + 2*ud^2);
    xd2 = rd*vd + k3*(r2 + 2*vd^2) + 2*k4*ud*vd;

    u = au*(xd1 + sk/au*xd2) + u0;
    v = av*xd2 + v0;
    
    %             % Normalized image projection
    %             x   = Xc / Zc;
    %             y   = Yc / Zc;
    %             r_2 = x*x + y*y;
    %             % New normalized point including distortion
    %             dx_1     = 2*k(3)*x*y + k(4)*(r_2 + 2*x*x);
    %             dx_2     = k(3)*(r_2 + 2*y*y) + 2*k(4)*x*y;
    %             rad_dist = 1 + k(1)*r_2 +k(2)*r_2*r_2;
    %             xd_1     = rad_dist*x + dx_1;
    %             xd_2     = rad_dist*y + dx_2;
    %             % Final pixel coordinates
    %             xp = fc(1)*xd_1 + cc(1);
    %             yp = fc(2)*xd_2 + cc(2);
    %             plot(xp, yp, 'k.')    
    
    % calculate the geometric distance in x and y direction
    % um,vm =the x and y positions of an extracted corner
    % u,v = the x and y positions of the projection of the corresponding model point
    dx = um - u;
    dy = vm - v;
    
    % Evaluate the symbolic expression of the Jacobian w.r.t. the estimated parameters
    Jx = jacobian(dx,[au,av,u0,v0,sk,wx wy wz tx ty tz k1 k2 k3 k4]);
    Jy = jacobian(dy,[au,av,u0,v0,sk,wx wy wz tx ty tz k1 k2 k3 k4]);
    
    % NOTE use function sym2fun to generate frunctions for these symbolic
    % expressions, to make the code run faster. I used it to generate
    % functions jacobianX and jacobianY (see below) -- LENS
    
    % R0, t0 = initial extrinsic parameters obtained from the solution in Section 3
    tx = t0(1);
    ty = t0(2);
    tz = t0(3);
    
    % convert the 3x3 rotation matrix into 3-vector w=[wx wy wz] of the Rodigrues representation
    R  = R0;
    au = f;
    av = f;    
    u0 = 640/2;
    v0 = 480/2;
    sk = 1;
    k1 = 0; k2 = 0; k3 = 0; k4 = 0;
    
    theta = acos((trace(R)-1)/2);
    w = (theta/(2*sin(theta)))*[R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
    wx = w(1);
    wy = w(2);
    wz = w(3);
    iters = 250;
    lamda = 0.1;
    updateJ = 1;
    Ndata = 2*n; % n is the number of corner points
    Nparams = 15;
    for it = 1:iters
        fprintf('Iteration: %d\n',it);
        if updateJ == 1
            % set the number of iterations for the LM algorithm
            % initial the value of damping factor
            % create the intrinsic parameter matrix
            K=[au sk u0 0;
                0 av v0 0;
                0  0  1 0 ];
            
            % convert the 3-vector [wx wy wz] of the Rodigrues representation
            % into the 3x3 rotation matrix
            theta2 = wx^2 + wy^2 + wz^2;
            theta = sqrt(theta2);
            omega = [0 -wz  wy;
                    wz   0 -wx;
                   -wy  wx   0;];
            R = eye(3) + (sin(theta)/theta)*omega + ((1-cos(theta))/theta^2)*(omega*omega);
            t = [tx; ty; tz];
            
            % Evaluate the Jacobian at the current parameter values
            % and the values of geometric distance
            J = zeros(Ndata, Nparams);
            d = zeros(Ndata, 1);
            fprintf('Evaluating Jacobian for point\n');
            for i = 1:size(xe, 2)
                fprintf('%d,',i);
                X = Xutm(1, i);
                % (X,Y) are the coordinates of the ith model point
                Y = Xutm(2, i);
                Z = Xutm(3, i);
                % " the value of Jx evaluated at the ith model point X,Y and the current parameters"
                J(2*(i-1)+1,:) = jacobianX(X,Y,Z,au,av,k1,k2,k3,k4,sk,tx,ty,tz,u0,v0,wx,wy,wz)'; %eval(Jx);                
                % " the value of Jy evaluated at the ith model point X,Y and the current parameters"
                J(2*(i-1)+2,:) = jacobianY(X,Y,Z,au,av,k1,k2,k3,k4,sk,tx,ty,tz,u0,v0,wx,wy,wz)'; %eval(Jy);
                                % perspective projection of the model point
                uvs = K*[R t;  0 0 0 1]*[X; Y; Z; 1];                
                ud = uvs(1) / uvs(3);
                vd = uvs(2) / uvs(3);   

                % Account for radial distortion
                r2  = ud^2 + vd^2;
                rd  = 1 + k1*r2 + k2*r2^2;
                xd1 = rd*ud + 2*k3*ud*vd + k4*(r2 + 2*ud^2);
                xd2 = rd*vd + k3*(r2 + 2*vd^2) + 2*k4*ud*vd;

                up = au*(xd1 + sk/au*xd2) + u0;
                vp = av*xd2 + v0;
                
                fprintf('Comparing xe: (%f,%f) with xp: (%f,%f)\n', ...
                    xe(1,i), xe(2,i), up, vp);
                % compute the geometric distance in x & y directions
                % xe(1,i), xe(2,i) = the the x and y positions of the i th extracted corner.
                % up,vp = the x and y positions of the projection of the corresponding model point
                d(2*(i-1)+1,1) = xe(1,i) - up;
                d(2*(i-1)+2,1) = xe(2,i) - vp;
            end
            fprintf('\nComputing Hessian matrix\n');
            % compute the approximated Hessian matrix
            H = J'*J;
            if it == 1 % the first iteration : compute the initial total error
                e = dot(d, d);
                disp(e);
            end
        end
        % Apply the damping factor to the Hessian matrix
        H_lm = H + (lamda*eye(Nparams, Nparams));
        % Compute the updated parameters
        dp = -inv(H_lm) * (J' * d(:));
        au_lm = au + dp(1);
        av_lm = av + dp(2);
        u0_lm = u0 + dp(3);
        v0_lm = v0 + dp(4);
        sk_lm = sk + dp(5);
        wx_lm = wx + dp(6);
        wy_lm = wy + dp(7);
        wz_lm = wz + dp(8);
        tx_lm = tx + dp(9);
        ty_lm = ty + dp(10);
        tz_lm = tz + dp(11);
        k1_lm = k1 + dp(12);
        k2_lm = k2 + dp(13);
        k3_lm = k3 + dp(14);
        k4_lm = k4 + dp(15);
        
        % Evaluate the total geometric distance at the updated parameters
        K = [au_lm sk_lm u0_lm 0;
               0   av_lm v0_lm 0;
               0     0     1   0 ];
           
        omega = [ 0  -wz_lm wy_lm;
                wz_lm   0  -wx_lm;
               -wy_lm wx_lm   0; ];
           
        theta2 = wx_lm^2 + wy_lm^2 + wz_lm^2;
        theta = sqrt(theta2);
        R = eye(3) + (sin(theta)/theta)*omega + ((1-cos(theta))/theta^2)*(omega*omega);
        t = [tx_lm; ty_lm; tz_lm];
        d_lm = zeros(Ndata, 1);
        fprintf('Evaluating the total geometric distance at the updated parameters\n');
        for i = 1:size(xe, 2)
            X = Xutm(1, i);
            Y = Xutm(2, i);
            Z = Xutm(3, i);
            uvs = K*[R t; 0 0 0 1]*[X; Y; Z; 1];
            ud = uvs(1) / uvs(3);
            vd = uvs(2) / uvs(3);     
            
            % Account for radial distortion
            r2  = ud^2 + vd^2;
            rd  = 1 + k1_lm*r2 + k2_lm*r2^2;
            xd1 = rd*ud + 2*k3_lm*ud*vd + k4_lm*(r2 + 2*ud^2);
            xd2 = rd*vd + k3_lm*(r2 + 2*vd^2) + 2*k4_lm*ud*vd;

            up = au_lm*(xd1 + sk_lm/au_lm*xd2) + u0_lm;
            vp = av_lm*xd2 + v0_lm;
            
            d_lm(2*(i-1)+1,1) = xe(1,i) - up;
            d_lm(2*(i-1)+2,1) = xe(2,i) - vp;
        end
        % e_lm is the error between the image coordinates and projective coordinates using
        % the updated parameters
        e_lm = dot(d_lm, d_lm);
        %If the total geometric distance of the updated parameters is less than the previous one
        % then makes the updated parameters to be the current parameters
        % and decreases the value of the damping factor
        if e_lm < e
            lamda = lamda / 10;
            au = au_lm;
            av = av_lm;
            u0 = u0_lm;
            v0 = v0_lm;
            sk = sk_lm;
            wx = wx_lm;
            wy = wy_lm;
            wz = wz_lm;
            tx = tx_lm;
            ty = ty_lm;
            tz = tz_lm;
            k1 = k1_lm;
            k2 = k2_lm;
            k3 = k3_lm;
            k4 = k4_lm;
            e  = e_lm;
            disp(e);
            updateJ = 1;
        else
            % otherwise increase the value of the damping factor and try again
            updateJ = 0;
            lamda = lamda*10;
        end
    end
end
   
if 1
    % x are the points in the image plane
    % X are the points in 3D space (world frame)
    
    % Make sure that R, t, and K are already in the workspace!
    
    % Video3
    x = [ 181.684669 272.277003
          179.454704 225.447735
          109.210801 275.621951
           34.506969 270.604530
          154.925087 187.538328
          566.911150 277.294425
          498.897213 193.113240
          554.088850 223.775261
          439.803136 141.824042
          442.590592 232.137631
          383.496516 185.308362
          630.465157 314.088850
          148.235192 294.019164
          443.148084 326.911150
           32.834495 228.792683 ];
    x = x';    
    
    Xutm = [ 8.981081008911 -6.767824649811 0.777583718300
             8.972593307495 -6.840630054474 1.910237550735
            10.428060       -8.081507       0.708104
            11.850910186768 -9.614816665649 0.651222467422
            10.554234504700 -6.439673900604 3.439467430115
             2.702903747559  2.470865726471 0.943239808083
             0.620263814926 -0.830464124680 2.668621540070
            -0.172788679600  0.978258848190 1.771192789078
            -0.276789307594 -3.342656612396 3.896268606186
            -0.455072462559 -3.196919441223 0.776353836060
            -5.858895778656 -8.726755142212 0.808652758598
             4.051758766174  4.836362838745 0.697832226753
            11.008689880371 -5.960782051086 0.686188578606
             7.831201553345  1.200059533119 0.573717236519
            11.900057792664 -9.778165817261 1.865399599075 ];
    Xutm = Xutm';
   
    
    % Now, let's project a point of the world into the image
    a = imread( ['~/' fileName '.jpg'] );
    clf
    imshow( a ), hold on
    
    H = [ R t; 0 0 0 1];
    
    for q = 1:size(Xutm, 2)        
        % Point in world frame
        Pwf = [ Xutm(:,q); 1];
        % Point in camera frame
        Pcf = H*Pwf;
        Pcf = K*Pcf;
        Xc = Pcf(1); Yc = Pcf(2); Zc = Pcf(3);        
        
        % Normalized image projection
        ud = Xc/Zc;
        vd = Yc/Zc;        
        
        % Account for radial distortion
        r2  = ud^2 + vd^2;
        rd  = 1 + k1*r2 + k2*r2^2;
        xd1 = rd*ud + 2*k3*ud*vd + k4*(r2 + 2*ud^2);
        xd2 = rd*vd + k3*(r2 + 2*vd^2) + 2*k4*ud*vd;

        xp = au*(xd1 + sk/au*xd2) + u0;
        yp = av*xd2 + v0;        
        
        plot(xp, yp, 'r+', 'linewidth', 2)
        
        % Original markers
        plot(x(1,q), x(2,q), 'bo', 'linewidth', 2)
        
    end
    %             % Convert from world frame to camera frame
    %             P = R*[X; Y; 0] + T;
    %             Xc = P(1); Yc = P(2); Zc = P(3);
    %
    %             % Compute pixel coordinates of a point in camera frame (Xc, Yc, Zc)
    %             % Normalized image projection
    %             x   = Xc / Zc;
    %             y   = Yc / Zc;
    %             r_2 = x*x + y*y;
    %             % New normalized point including distortion
    %             dx_1     = 2*k(3)*x*y + k(4)*(r_2 + 2*x*x);
    %             dx_2     = k(3)*(r_2 + 2*y*y) + 2*k(4)*x*y;
    %             rad_dist = 1 + k(1)*r_2 +k(2)*r_2*r_2;
    %             xd_1     = rad_dist*x + dx_1;
    %             xd_2     = rad_dist*y + dx_2;
    %             % Final pixel coordinates
    %             xp = fc(1)*xd_1 + cc(1);
    %             yp = fc(2)*xd_2 + cc(2);
    %             plot(xp, yp, 'k.')
    
    K = [au_lm sk_lm u0_lm 0;
           0   av_lm v0_lm 0;
           0     0     1   0 ];
    K
    omega = [ 0  -wz_lm wy_lm;
            wz_lm   0  -wx_lm;
           -wy_lm wx_lm   0; ];
    theta2 = wx_lm^2 + wy_lm^2 + wz_lm^2;
    theta = sqrt(theta2);
    R = eye(3) + (sin(theta)/theta)*omega + ((1-cos(theta))/theta^2)*(omega*omega);
    t = [tx_lm; ty_lm; tz_lm];
    R
    t

end

% %P = importdata('tempforecast-03-004.txt');
% 
% x1 = P(:,1);
% y1 = P(:,2);
% x2 = P(:,3);
% y2 = P(:,4);
% x3 = P(:,5);
% y3 = P(:,6);
% x4 = P(:,7);
% y4 = P(:,8);
% cr = P(:,9);
% cg = P(:,10);
% cb = P(:,11);
% 
% xC = (x1+x2+x3+x4)./4;
% yC = (y1+y2+y3+y4)./4;
% fv = P(:,12);
% 
% % Get calibration parameters
% fc = [2696.35888671875; 2696.35888671875];
% cc = [959.5; 539.5];
% T  = [-0.05988363921642303467; 3.83331298828; 12.39112186431884765625];
% % pose quaternion expressed as w x y z
% q  = [0.49527896681027261394 0.69724917918208628720 -0.43029624469563848566 0.28876888503799524877];
% R  = quaternion2matrix( q );
% k  = [-0.60150605440139770508; 4.70203733444213867188; -0.00047452122089453042; -0.00782289821654558182; 0];
% 
% % Compute equation of ground plane, in camera frame
% p0 = [0 0 0]';
% p1 = [1 0 0]';
% p2 = [0 1 0]';
% n  = [0 0 1]'; 
% 
% p0c = R*p0 + T;
% p1c = R*p1 + T;
% p2c = R*p2 + T;
% A =  det( [1 p0c(2:3)'; 1 p1c(2:3)'; 1 p2c(2:3)'] );
% B =  det( [p0c(1) 1 p0c(3); p1c(1) 1 p1c(3); p2c(1) 1 p2c(3) ] );
% C =  det( [p0c(1:2)' 1; p1c(1:2)' 1; p2c(1:2)' 1] );
% D = -det( [p0c'; p1c'; p2c' ] );
% 
% % http://paulbourke.net/geometry/pointlineplane/
% % For a ray coming from the origin of camera frame through point P(x2, y2, z2) in the
% % image plane, the intersection with the plane occurs at
% %u = D / (-A*x2 -B*y2 -C*z2);
% 
% %The intersection point in camera frame is 
% %Pg = u*P;
% 
% if 1
%     for X = 3:1:28
%         for Y = -2:1:15
%             
%             % Convert from world frame to camera frame
%             P = R*[X; Y; 0] + T;
%             Xc = P(1); Yc = P(2); Zc = P(3);
%             
%             % Compute pixel coordinates of a point in camera frame (Xc, Yc, Zc)
%             % Normalized image projection
%             x   = Xc / Zc;
%             y   = Yc / Zc;
%             r_2 = x*x + y*y;
%             % New normalized point including distortion
%             dx_1     = 2*k(3)*x*y + k(4)*(r_2 + 2*x*x);
%             dx_2     = k(3)*(r_2 + 2*y*y) + 2*k(4)*x*y;
%             rad_dist = 1 + k(1)*r_2 +k(2)*r_2*r_2;
%             xd_1     = rad_dist*x + dx_1;
%             xd_2     = rad_dist*y + dx_2;
%             % Final pixel coordinates
%             xp = fc(1)*xd_1 + cc(1);
%             yp = fc(2)*xd_2 + cc(2);
%             plot(xp, yp, 'k.')            
%             text(xp, yp, sprintf('(%d,%d)',X,Y) );
%             [xn] = normalize_pixel( [xp; yp], fc, cc, k, 0);
% %             fprintf( '(%.3f, %.3f) => [%.2f, %.2f] ==> (%.3f, %.3f)\n', ...
% %                 Xc, Yc, xp, yp, xn(1)*Zc, xn(2)*Zc);
% %                         
%         end
%     end
% end
% 
% while 1
%     [mx, my, button] = ginput(1);
%     %fprintf('%.2f, %.2f, %d\n', mx, my, button);
%     if button ~= 1
%         break;
%     end
%     % Convert from pixel coordinates to world frame, on the ground plane
%     [xn] = normalize_pixel( [mx; my], fc, cc, k, 0);
%     A = [ R(1,1) - R(3,1)*xn(1)    R(1,2) - R(3,2)*xn(1)
%           R(2,1) - R(3,1)*xn(2)    R(2,2) - R(3,2)*xn(2) ];
%     B = [ xn(1)*T(3) - T(1)
%           xn(2)*T(3) - T(2) ];
%     p = A\B;    
%     fprintf('(%.2f, %.2f, 0)\n', p(1), p(2));
% end
% 
% return
% 
% % inverse problem: computing the normalized image projection vector xn from the pixel coordinate
% %[xn] = normalize_pixel(x_kk,fc,cc,kc,alpha_c)
% %
% %Computes the normalized coordinates xn given the pixel coordinates x_kk
% %and the intrinsic camera parameters fc, cc and kc.
% %
% %INPUT: x_kk: Feature locations on the images
% %       fc: Camera focal length
% %       cc: Principal point coordinates
% %       kc: Distortion coefficients
% %       alpha_c: Skew coefficient
% %
% %OUTPUT: xn: Normalized feature locations on the image plane (a 2XN matrix)
% 
% for q = 1:size(P,1)  
%         F = [1 2 4 3];
%         V = [];
%         % Convert from world frame to camera frame
%         p = R*[x1(q); y1(q); 0] + T;        
%         Xc = p(1); Yc = p(2); Zc = p(3);
%         
%         % Enforce these limits to keep the grid on the street
%         if( x1(q) < 5 ) || ( x1(q) > 30) || (y1(q) < -1) || (y1(q) > 15)
%             continue;
%         end
%         
%         if( fv(q) < -8 )
%             continue;
%         end
%         
%         % Compute pixel coordinates of a point in camera frame (Xc, Yc, Zc)
%         % Normalized image projection
%         x   = Xc / Zc;
%         y   = Yc / Zc;
%         r_2 = x*x + y*y;
%         % New normalized point including distortion
%         dx_1     = 2*k(3)*x*y + k(4)*(r_2 + 2*x*x);
%         dx_2     = k(3)*(r_2 + 2*y*y) + 2*k(4)*x*y;
%         rad_dist = 1 + k(1)*r_2 +k(2)*r_2*r_2;
%         xd_1     = rad_dist*x + dx_1;
%         xd_2     = rad_dist*y + dx_2;
%         % Final pixel coordinates
%         xp = fc(1)*xd_1 + cc(1);
%         yp = fc(2)*xd_2 + cc(2);       
%         V = [xp yp];
%         
%         % Convert from world frame to camera frame
%         p = R*[x2(q); y2(q); 0] + T;        
%         Xc = p(1); Yc = p(2); Zc = p(3);
%         
%         % Compute pixel coordinates of a point in camera frame (Xc, Yc, Zc)
%         % Normalized image projection
%         x   = Xc / Zc;
%         y   = Yc / Zc;
%         r_2 = x*x + y*y;
%         % New normalized point including distortion
%         dx_1     = 2*k(3)*x*y + k(4)*(r_2 + 2*x*x);
%         dx_2     = k(3)*(r_2 + 2*y*y) + 2*k(4)*x*y;
%         rad_dist = 1 + k(1)*r_2 +k(2)*r_2*r_2;
%         xd_1     = rad_dist*x + dx_1;
%         xd_2     = rad_dist*y + dx_2;
%         % Final pixel coordinates
%         xp = fc(1)*xd_1 + cc(1);
%         yp = fc(2)*xd_2 + cc(2);       
%         V = [V; xp yp];
% 
%                 % Convert from world frame to camera frame
%         p = R*[x3(q); y3(q); 0] + T;        
%         Xc = p(1); Yc = p(2); Zc = p(3);
%         
%         % Compute pixel coordinates of a point in camera frame (Xc, Yc, Zc)
%         % Normalized image projection
%         x   = Xc / Zc;
%         y   = Yc / Zc;
%         r_2 = x*x + y*y;
%         % New normalized point including distortion
%         dx_1     = 2*k(3)*x*y + k(4)*(r_2 + 2*x*x);
%         dx_2     = k(3)*(r_2 + 2*y*y) + 2*k(4)*x*y;
%         rad_dist = 1 + k(1)*r_2 +k(2)*r_2*r_2;
%         xd_1     = rad_dist*x + dx_1;
%         xd_2     = rad_dist*y + dx_2;
%         % Final pixel coordinates
%         xp = fc(1)*xd_1 + cc(1);
%         yp = fc(2)*xd_2 + cc(2);       
%         V = [V; xp yp];
% 
%         % Convert from world frame to camera frame
%         p = R*[x4(q); y4(q); 0] + T;        
%         Xc = p(1); Yc = p(2); Zc = p(3);
%         
%         % Compute pixel coordinates of a point in camera frame (Xc, Yc, Zc)
%         % Normalized image projection
%         x   = Xc / Zc;
%         y   = Yc / Zc;
%         r_2 = x*x + y*y;
%         % New normalized point including distortion
%         dx_1     = 2*k(3)*x*y + k(4)*(r_2 + 2*x*x);
%         dx_2     = k(3)*(r_2 + 2*y*y) + 2*k(4)*x*y;
%         rad_dist = 1 + k(1)*r_2 +k(2)*r_2*r_2;
%         xd_1     = rad_dist*x + dx_1;
%         xd_2     = rad_dist*y + dx_2;
%         % Final pixel coordinates
%         xp = fc(1)*xd_1 + cc(1);
%         yp = fc(2)*xd_2 + cc(2);       
%         V = [V; xp yp];
% 
%         patch('Faces', F, 'Vertices', V, 'FaceAlpha', 0.3, 'FaceColor', [cr(q) cg(q) cb(q)], ...
%             'EdgeColor', 'cyan', 'LineStyle', ':')                
%         
%         %{
%         [xn] = normalize_pixel( [xp; yp], fc, cc, k, 0);
%         fprintf( '(%.3f, %.3f) => [%.2f, %.2f] ==> (%.3f, %.3f)\n', ...
%             Xc, Yc, xp, yp, xn(1)*Zc, xn(2)*Zc);
%         %}
%         
%     
% end
% 
% 
