clear all
close all
start = clock;

% parameters
dt = 0.05;
tfinal = 60;
gravity = 32.174;
sigma_a = 0.3;   % acceleraomter noise (ft/s2)
sigma_w = 0.001; % gyro noise (rad/sec)
sigma_gps_assumed = 1;  % gps point measurement noise (per full screen)
sigma_gps = 1;       % gps point measurement noise (per full screen), about a pixel

allowError_imu = 1;

% allocate memory       matlab thing, array stuff
points = tfinal/dt + 1;
t  = zeros(1,points);
x  = zeros(10,points);   % north, east, down, vn, ve, vd, q0, q1, q2, q3   (truth, ownship)  ???
xh = zeros(9,points);    % north, east, down, vn, ve, vd, r0, r1, r2       (estimate, ownship)  ???
qh = zeros(4,points);    % quat estimate
xdot  = zeros(10,1);
xhdot = zeros(6,1);
qhdot = zeros(4,1);
P = zeros(9,9,points);
A = zeros(9,9);
Psave = zeros(9,points);
wb = zeros(3,1); % angular velocity in body (truth)   ???
ab = zeros(3,1); % accel in body            (truth)
pz_prc = zeros(2,3);
C_nav = zeros(2,9);
dq = zeros(4,1);
gps_delay_i=5;
pexp_buffer = zeros(3,gps_delay_i+1);
vexp_buffer = zeros(3,gps_delay_i+1);

% final setup and intial conditions
Q = diag([0 0 0 sigma_a^2 sigma_a^2 sigma_a^2 sigma_w^2 sigma_w^2 sigma_w^2]);  
R = diag([sigma_gps_assumed^2 sigma_gps_assumed^2 sigma_gps_assumed^2]);
x(:,1)   = [0;0;0;0;0;0;1;0;0;0];
P(:,:,1) = diag([1 1 1 1 1 1 0.01*0.01 0.01*0.01 0.01*0.01]);
xh(:,1)  = [x(1:6,1);0;0;0]; % perfect start
qh(:,1)  = x(7:10,1);         % perfect start

for k=1:points
    t(k)=(k-1)*dt;
    
    % truth dynamics    
    if t(k) < 3  %% basically gives the matlab a function to "turn"
        wb = [0;0;0];
        ab = [3;0;0];
    elseif t(k) > 30 && t(k) < 40
        wb = [0;0;9]*pi/180;
        ab = [0;1.41;0];
    else
        wb = [0;0;0];
        ab = [0;0;0];
    end
    
    Lib = quat2dcm(x(7:10,k));  %%what is this notation??
    Lbi = Lib';
    xdot(1:3) = x(4:6,k);
    xdot(4:6) = Lib*ab;
    xdot(7:10) = quat_kin(wb(1),wb(2),wb(3),x(7:10,k));
    
    % IMU model
    % [todo: account for imu not at c.g. ]
    accelerometers = ab - Lbi*[0;0;gravity] + sigma_a*randn(3,1)*allowError_imu;
    gyros          = wb                     + sigma_w*randn(3,1)*allowError_imu;
    
    % get measurements
    if k>gps_delay_i % gps?? update 
        % [todo: not necessarily a measurement every time...]
        
        % measurement model
        % add latency compensation
        delay_i = max([1 k-gps_delay_i]);
        pexp = x(1:3,delay_i); 
        vexp = x(4:6,delay_i);
        xg_L = pexp + sigma_gps*randn(3,1);

        newGps=1;
    else
        newGps=0;
    end
        
    % done with truth stuff - time for some filter
    
    % delay states (for latency compensation)
    % do this even when no new vision data
    for j=gps_delay_i:-1:1
        vexp_buffer(:,j+1) = vexp_buffer(:,j);
        pexp_buffer(:,j+1) = pexp_buffer(:,j);
    end
    vexp_buffer(:,1) = xh(4:6,k);
    pexp_buffer(:,1) = xh(1:3,k);
    vexp = vexp_buffer(:,1+gps_delay_i);
    pexp = pexp_buffer(:,1+gps_delay_i);

    if newGps
        % discrete update (GPS)

        residual = xg_L - xh(1:3,k);
        
        C_nav = zeros(3,9);  C_nav(1,1)=1; C_nav(2,2)=1; C_nav(3,3)=1;

        % update ownship state
        K = (P(:,:,k)*C_nav')/( C_nav*P(:,:,k)*C_nav' + R );
        dx = K*residual;
        xh(:,k) = xh(:,k) + dx;
        % reset_quat (like navigation.cpp)
        dq(1) = 0.5*( -xh(7,k)*qh(2,k) - xh(8,k)*qh(3,k) - xh(9,k)*qh(4,k) );
        dq(2) = 0.5*(  xh(7,k)*qh(1,k) - xh(8,k)*qh(4,k) + xh(9,k)*qh(3,k) );
        dq(3) = 0.5*(  xh(7,k)*qh(4,k) + xh(8,k)*qh(1,k) - xh(9,k)*qh(2,k) );
        dq(4) = 0.5*( -xh(7,k)*qh(3,k) + xh(8,k)*qh(2,k) + xh(9,k)*qh(1,k) );
        qh(:,k) = normalize_quat( qh(:,k) + dq );
        xh(7:9,k) = zeros(3,1);
        % get everything ready for next feature point
        % delay states (for latency compensation) also need to be modified
        pexp = pexp + dx(1:3);
        vexp = vexp + dx(4:6);
        for j=1:gps_delay_i+1
            vexp_buffer(:,j) = vexp_buffer(:,j) + dx(4:6);
            pexp_buffer(:,j) = pexp_buffer(:,j) + dx(1:3);
        end
        % covariance
        P(:,:,k) = P(:,:,k) - K*C_nav*P(:,:,k);
    end % end new gps data
        
    % estimator process model
    Lib = quat2dcm(qh(:,k));  Lbi = Lib';
    xhdot(1:3) = xh(4:6,k);
    xhdot(4:6) = Lib*( accelerometers + Lbi*[0;0;gravity] );
    qhdot = quat_kin(gyros(1),gyros(2),gyros(3),qh(:,k));
        
    % covariance continuous update    
    A(1:3,4:6) = eye(3);
    A(4,7) = -Lib(1,2)*accelerometers(3) + Lib(1,3)*accelerometers(2);  A(4,8) = +Lib(1,1)*accelerometers(3) - Lib(1,3)*accelerometers(1);  A(4,9) = -Lib(1,1)*accelerometers(2) + Lib(1,2)*accelerometers(1);
    A(5,7) = -Lib(2,2)*accelerometers(3) + Lib(2,3)*accelerometers(2);  A(5,8) = +Lib(2,1)*accelerometers(3) - Lib(2,3)*accelerometers(1);  A(5,9) = -Lib(2,1)*accelerometers(2) + Lib(2,2)*accelerometers(1);
    A(6,7) = -Lib(3,2)*accelerometers(3) + Lib(3,3)*accelerometers(2);  A(6,8) = +Lib(3,1)*accelerometers(3) - Lib(3,3)*accelerometers(1);  A(6,9) = -Lib(3,1)*accelerometers(2) + Lib(3,2)*accelerometers(1);
	%rd[0] = (  p      - q*r[2] + r*r[1] );
	%rd[1] = (  p*r[2] + q      - r*r[0] );
	%rd[2] = ( -p*r[1] + q*r[0] + r      );
                         A(7,8) = +gyros(3);  A(7,9) = -gyros(2);
    A(8,7) = -gyros(3);                       A(8,9) = +gyros(1);
    A(9,7) = +gyros(2);  A(9,8) = -gyros(1);    
    Pdot = A*P(:,:,k) + P(:,:,k)*A' + Q;
    
    % numerically integrate everything
    if k==1
        x(:,k+1)    = x(:,k)    + xdot*dt;
        xh(1:6,k+1) = xh(1:6,k) + xhdot*dt;
        qh(:,k+1)   = qh(:,k)   + qhdot*dt;
        P(:,:,k+1)  = P(:,:,k)  + Pdot*dt;
    elseif k<points
        x(:,k+1)    = x(:,k)    + ( 1.5*xdot    - 0.5*oldxdot    )*dt;
        xh(1:6,k+1) = xh(1:6,k) + ( 1.5*xhdot   - 0.5*oldxhdot   )*dt;
        qh(:,k+1)   = qh(:,k)   + ( 1.5*qhdot   - 0.5*oldqhdot   )*dt;
        P(:,:,k+1)  = P(:,:,k)  + ( 1.5*Pdot    - 0.5*oldPdot    )*dt;
    end
    oldxdot    = xdot;
    oldxhdot   = xhdot;
    oldqhdot   = qhdot;
    oldPdot    = Pdot;
    Psave(:,k) = diag(P(:,:,k));
    
end

% plots

figure(1)
plot(t,2*sqrt(Psave(1,:)),t,2*sqrt(Psave(2,:)));
xlabel('time(sec)')
ylabel('2*sigma(ft)');
legend('North','East');

figure(2)
plot(x(2,:),x(1,:),'k',xh(2,:),xh(1,:),'b');
xlabel('East(ft)')
ylabel('North(ft)')
legend('truth','estimate')
%axis([-.25 1.25 -0.25 1]*1000*1.5)
axis equal

% figure(3)
% plot3(x(2,:),x(1,:),-x(3,:),'k',xh(2,:),xh(1,:),-xh(3,:),'b',f(2,:),f(1,:),-f(3,:),'k.',xfp_L(2,:),xfp_L(1,:),-xfp_L(3,:),'b.');
% xlabel('East(ft)')
% ylabel('North(ft)')
% zlabel('altitude(ft)');
% legend('truth','estimate','features','est features')
% axis equal

elapsed_time = etime(clock,start)