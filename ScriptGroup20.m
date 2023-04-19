% SPACECRAFT ATTITUDE DYNAMICS PROJECT - Group 20
clear
close all
clc

%--------------------------------------------------------------------------
% How to use this script:
% 1) Run each section below before simulations to load in the workspace all 
%    needed constants;
% 2) Run uncontrolled case (directly after data loading);
% 3) Run de-tumbling, slew maneuvering and tracking cases without closing
%    plots: important quantities of the 3 phases are shown and compared together
%    as subplots, instead some other results(related with the specific
%    simulation phase) will be shown alone in new figures.
% What you need:
%  -ModelGroup20.slx;
%  -Stars.txt
%--------------------------------------------------------------------------

%% Definition of the HEO orbit
    a=(39376)*1e3;          % Semi-major axis [m] 
    e=0.7596;               % Eccentricity []
    sigma=deg2rad(0);       % Right Ascension of the Ascending Node [rad]
    i=deg2rad(28.4252);     % Inclination [rad]
    omega=deg2rad(0);       % Argument of Perigee [rad]
    theta0orb=deg2rad(0);   % True Anomaly [rad]

Re=6371.01*1e3;  % Earth Radius [m]
mu=3.98600433e+5*1e9;  % Earth Gravity Constant [m^3/s^2]
Period=2*pi*sqrt(a^3/mu);   % Orbital Period [s]

%% Spacecraft Model
    % Normals to the Surfaces
        n1=[0;0;1]; n4=-n1;
        n2=[0;1;0]; n5=-n2;
        n3=[1;0;0]; n6=-n3;
        % Solar panels 
        n7=[1;0;0]; n8=-n7;
        n9=[1;0;0]; n10=-n9;
    normals=[n1,n2,n3,n4,n5,n6,n7,n8,n9,n10];

    % Materials Reflectivities
        rhos_sc=0.5;
        rhod_sc=0.1;
        % Solar panels 
        rhos_sp=0.1;
        rhod_sp=0.1;
    propmat=[rhos_sc;rhod_sc;rhos_sp;rhod_sp];

    % Area of the surfaces (LUMIO SC) [cm]
        A1=6*1e-2;  
        A2=A1;
        A3=4*1e-2;
        A7=12*1e-2;
        A9=A7;
    propsurf=[A1,A2,A3,A1,A2,A3,A7,A7,A9,A9];

% Definizione distanze cp-cm (worst case scenario cm=centro simmetrica geometrica, LUMIO SC)
    r1=[0;0;0.1];    r4=-r1;
    r2=[0;0.1;0];    r5=-r2;
    r3=[0.15;0;0];   r6=-r3;
    r7=[0;0.45;0];   r8=r7;
    r9=-r7;          r10=r9;

    cm=[0;0.015;0];
    r_cpTocm=[r1,r2,r3,r4,r5,r6,r7,r8,r9,r10]-ones(3,10).*cm;

% Inertia Tensors
    I_unpacked=1e-2.*[100.9,0,0;0,25.1,0;0,0,91.6]; %Inertia tensor of S/C with solar panels deployed
    I_packed=1e-2.*[30.5,0,0;0,20.9,0;0,0,27.6]; 
    I_tensor=I_unpacked;
Iinv_tensor=inv(I_tensor);

%% Disturbance Torques
% Magnetic Torque (Other variables are defined in the model)
    factorunitmass=1e-3;  %From tables calss I
    mass12U=22.82; 
    factor=factorunitmass*mass12U;
    m_sc=factor*[1;1;1];

%% Sensors
% Gyroscope
    ARW=4.36*1e-5;      % [rad/sqrt(s)]
    RRW=2.422*1e-11;    % [rad/s]
    bias=1.211*1e-3;
    fs=5;
    ts=1/fs;
    thetn=ARW/sqrt(ts);
    thetn=thetn^2;
    thetb=RRW/sqrt(ts);
    thetb=thetb^2;
    Aorthsens=1e-3.*[1000,0.2,0.2;0.2,1000,0.2;0.2,0.2,1000]; % [rad]
    thetsperc=5*1e-4;   % [ppm] 
    Res=1.066*1e-6;
    fgyro=5;
    fst=5;   


% Star tracker
    % Accuracy 
    alphaxy=4.8456*1e-5;    % [rad]
    alphaxy=(alphaxy/3)^2;
    alphaz=5.8148*1e-4;     % [rad]
    alphaz=(alphaz/3)^2;
    % Star Catalogue
    starcattab=readtable('Stars.txt');
    starcat=zeros(height(starcattab)-2,2);
    starcat(:,1)=starcattab.Var13(1:end-2);
    starcat(:,2)=starcattab.Var14(1:end-2);
    % The angle range in which the sun should not be into the sensor FOV
    burn_angle = deg2rad(30); 
    % Maximum angular velocity in order to be able to acquire the attitude
    w_lim = deg2rad(3);     % [rad/s]

%% Actuators

% Reaction Wheels
    % Max Angular Momentum 
    maxH=50*1e-3;  % [Nms]
    % Max Torque 
    maxT=8*1e-3;   % [Nm]
    hr0=[0;0;0;0];

% Thrusters
    % Distance between nozzles and x axis
    x=0.09;                 % [m]
    % Distance between nozzles and zy plane
    l=0.15;                 % [m]
    % Tilted Angle
    gamma=deg2rad(10);      % [rad]
    % Distribution Matrix
    R2=[1,0,0;0,0,1;0,-1,0];
    R1=[-1,0,0;0,-1,0;0,0,1];
    T=R2*R1*[x*sin(gamma),-x*sin(gamma),x*sin(gamma),-x*sin(gamma);...
        -x*cos(gamma),l*sin(gamma),x*cos(gamma),-l*sin(gamma);...
        -l*sin(gamma),x*cos(gamma),l*sin(gamma),-x*cos(gamma)];
    % Maximum Force of the thruster
    Fmax=0.325/2;            % [N]
    w=null(T);

%% Control Gains

        Kpdet=0.3;
        Kptrack=1;    %I'll use these fictious gains for uncontrolled and de-tumbling case.
        Kdtrack=1;

        Kptrack_slew=0.5;
        Kdtrack_slew=0.5;
        Kptrack_track=0.009;
        Kdtrack_track=0.009;

%% Simulations

% % Initial conditions for  and Euler Angles
%     A0=eye(3);  % Direction Cosine Matrix
% 
% % Control gains
%     Kptrack=1;
%     Kdtrack=1;

%% Uncontrolled simulation
% A first analysis on an entire orbital period is run without considering the
% possibility to control the spacecraft in order to show the difference
% between controlled and uncontrolled s/c

    ModCont=0;
    Controlled=0;

    % Initial conditions
    phi0=0;
    theta0=0;
    psi0=0;
    w0_vector=[0;0;0];
    theta0orb=0;

    % Time of the Simulation
    SimTime=Period;

fprintf('\n It is now running a simulation with the spacecraft in uncontrolled conditions.\n              ~ Check figure 1  ~\n')
out=sim('ModelGroup20.slx'); 

time=out.tout;
wgyro=squeeze(out.simout.data);
Asens=squeeze(out.simout1.data);
Dist=squeeze(out.simout6.data);
disctime=linspace(time(1),time(end),length(wgyro));

figure(1)
sgtitle('Uncontrolled Simulation')

subplot(3,1,1)
plot(time,Dist(1,:),'LineWidth',1);
hold on;
plot(time,Dist(2,:),'LineWidth',1);
plot(time,Dist(3,:),'LineWidth',1);
grid on;
title('Environment disturb over 1 period');

subplot(3,1,2)
plot(disctime,squeeze(Asens(1,1,:)),'LineWidth',1);
hold on;
plot(disctime,squeeze(Asens(1,2,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(1,3,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(2,1,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(2,2,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(2,3,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(3,1,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(3,2,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(3,3,:)),'LineWidth',1);
grid on;
title('Sensors DCM over 1 period');

subplot(3,1,3)
plot(disctime,wgyro(1,:),'LineWidth',1);
hold on;
plot(disctime,wgyro(2,:),'LineWidth',1);
plot(disctime,wgyro(3,:),'LineWidth',1);
grid on;
title('Gyro angular velocity over 1 period');

%% Controlled Simulation
% Now another simulation start by considering that the control system is activated 
% In this way we can separate the mission from the deployments in stages:
%   - De-Tumbling (the Angular Velocities)
%   - Slew Manoeuvring ( in order to reach the desidered attitude)
%   - Pointing (for the mission requirement)
fprintf('****************************************************************')
fprintf('      - Group 20\nModelling the dynamics of a 12U spacecraft in an HEO\n\n')

    %% De-tumbling

        timedet=30;                 % [s]
        I_tensor=I_packed;
        ModCont=0;
        Controlled=1;
        
        phi0=deg2rad(5);
        theta0=deg2rad(5);
        psi0=deg2rad(5);
        w0_vector=[0.0262;0.0262;0.0262];       % [rad/s]
        hr0=[0;0;0;0];
        
        SimTime=timedet;

fprintf('\n It is now running a simulation with the spacecraft in controlled conditions:\n              ~ Check figure 2  ~\n                  - De-Tumbling phase t= 0s -> 30s')
out=sim('ModelGroup20.slx'); 

time=out.tout;
wgyrodet=out.simout.data;
Asensdet=out.simout1.data;
hrwdet=squeeze(out.simout2.data);
Angtruedet=out.simout5.data;
Torquedet=squeeze(out.simout7.data);
forbit=squeeze(out.simout8.data);
disctimedet=linspace(time(1),time(end),length(hrwdet));

figure(2)

subplot(4,3,1)
plot(disctimedet,hrwdet(1,:),'LineWidth',1);
hold on;
plot(disctimedet,hrwdet(2,:),'LineWidth',1);
plot(disctimedet,hrwdet(3,:),'LineWidth',1);
plot(disctimedet,hrwdet(4,:),'LineWidth',1);
plot(disctimedet,maxH*ones(length(hrwdet),1),'--','LineWidth',1);
plot(disctimedet,-maxH*ones(length(hrwdet),1),'--','LineWidth',1);
grid on;
xlabel('Time [s]');
ylabel('h_{RW} [Nms]');
title('Angular momentum stored by RWs during de-tumbling');

subplot(4,3,4)
plot(disctimedet,squeeze(Asensdet(1,1,:)),'LineWidth',1);
hold on;
plot(disctimedet,squeeze(Asensdet(1,2,:)),'LineWidth',1);
plot(disctimedet,squeeze(Asensdet(1,3,:)),'LineWidth',1);
plot(disctimedet,squeeze(Asensdet(2,1,:)),'LineWidth',1);
plot(disctimedet,squeeze(Asensdet(2,2,:)),'LineWidth',1);
plot(disctimedet,squeeze(Asensdet(2,3,:)),'LineWidth',1);
plot(disctimedet,squeeze(Asensdet(3,1,:)),'LineWidth',1);
plot(disctimedet,squeeze(Asensdet(3,2,:)),'LineWidth',1);
plot(disctimedet,squeeze(Asensdet(3,3,:)),'LineWidth',1);
grid on;
xlabel('Time [s]');
title('Measured DCM during de-tumbling');

subplot(4,3,7)
plot(disctimedet,wgyrodet(1,:),'LineWidth',1);
hold on;
plot(disctimedet,wgyrodet(2,:),'LineWidth',1);
plot(disctimedet,wgyrodet(3,:),'LineWidth',1);
grid on;
xlabel('Time [s]');
ylabel('\omega [rad/s]');
title('Measured angular velocity during de-tumbling');

subplot(4,3,10)
plot(disctimedet,Torquedet(1,:),'LineWidth',1);
hold on;
plot(disctimedet,Torquedet(2,:),'LineWidth',1);
plot(disctimedet,Torquedet(3,:),'LineWidth',1);
grid on;
xlabel('Time [s]');
ylabel('Torque [Nm]');
title('Applied control torque during de-tumbling maneuvering');


    %% Slew Maneuvering
        timeslew=50;            % [s]
        I_tensor=I_unpacked;
        ModCont=1;
        Controlled=1;
        
        Kptrack=Kptrack_slew;
        Kdtrack=Kdtrack_slew;
        
        phi0=Angtruedet(1,1,end);
        theta0=Angtruedet(2,1,end);
        psi0=Angtruedet(3,1,end);
        w0_vector=[0;0;0];
        hr0=hrwdet(:,end);
        theta0orb=forbit(end);
        
        SimTime=timeslew;
        fprintf('\n                  - Slew-Manouvering phase t= 30s -> 70s')
        out=sim('ModelGroup20.slx'); 
time=out.tout;
wgyro=out.simout.data;
Asens=out.simout1.data;
hrw=out.simout2.data;
abserr=out.simout3.data;
forbitslew=squeeze(out.simout8.data);
Angtrueslew=out.simout5.data;
Torque=squeeze(out.simout7.data);
TorqueThrust=(squeeze(out.simout9.data))';
disctime=linspace(timedet,timedet+time(end),length(hrw));
lengthslew=length(hrw);

figure(2);
subplot(4,3,2)
plot(disctime,hrw(1,:),'LineWidth',1);
hold on;
plot(disctime,hrw(2,:),'LineWidth',1);
plot(disctime,hrw(3,:),'LineWidth',1);
plot(disctime,hrw(4,:),'LineWidth',1);
plot(disctime,maxH*ones(length(hrw),1),'--','LineWidth',1);
plot(disctime,-maxH*ones(length(hrw),1),'--','LineWidth',1);
grid on;
xlabel('Time [s]');
ylabel('h_{RW} [Nms]');
xlim([disctime(1),disctime(end)]);
title('Angular momentum stored by RWs during slew maneuvering');

subplot(4,3,5)
plot(disctime,squeeze(Asens(1,1,:)),'LineWidth',1);
hold on;
plot(disctime,squeeze(Asens(1,2,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(1,3,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(2,1,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(2,2,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(2,3,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(3,1,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(3,2,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(3,3,:)),'LineWidth',1);
grid on;
xlabel('Time [s]');
xlim([disctime(1),disctime(end)]);
title('Attitude determination DCM during slew maneuvering');

subplot(4,3,8)
plot(disctime,wgyro(1,:),'LineWidth',1);
hold on;
plot(disctime,wgyro(2,:),'LineWidth',1);
plot(disctime,wgyro(3,:),'LineWidth',1);
grid on;
xlabel('Time [s]');
ylabel('\omega [rad/s]');
xlim([disctime(1),disctime(end)]);
title('Measured angular velocity during slew maneuvering');

subplot(4,3,11)
plot(disctime,Torque(1,:),'LineWidth',1);
hold on;
plot(disctime,Torque(2,:),'LineWidth',1);
plot(disctime,Torque(3,:),'LineWidth',1);
grid on;
xlabel('Time [s]');
ylabel('Torque [Nm]');
xlim([disctime(1),disctime(end)]);
title('Applied control torque during slew maneuvering');

figure(3)
sgtitle('Slew Manoeuvring')
subplot(3,1,1)
semilogy(disctime,abserr,'LineWidth',1);
hold on;
grid on;
semilogy(disctime,0.1*ones(length(hrw),1),'--','LineWidth',1);
xlabel('Time [s]');
ylabel('APE [deg]');
legend('APE','Pointing requirements');
xlim([disctime(1),disctime(end)]);
title('Absolute Perfomance Error');

subplot(3,1,2)
plot(disctime,TorqueThrust(1,:),'LineWidth',1);
hold on;
plot(disctime,TorqueThrust(2,:),'LineWidth',1);
plot(disctime,TorqueThrust(3,:),'LineWidth',1);
grid on;
xlabel('Time [s]');
ylabel('Torque Thrusters [Nm]');
xlim([disctime(1),disctime(end)]);
title('Thrusters torque during slew maneuver');

%De-tumbling and slew situation RW
hrsemitot=[hrwdet(:,:),squeeze(hrw(:,1,2:end))];
disctimesemi=[disctimedet,disctime(2:end)];

subplot(3,1,3)
plot(disctimesemi,hrsemitot(1,:),'LineWidth',1);
hold on;
plot(disctimesemi,hrsemitot(2,:),'LineWidth',1);
plot(disctimesemi,hrsemitot(3,:),'LineWidth',1);
plot(disctimesemi,hrsemitot(4,:),'LineWidth',1);
plot(disctimesemi,maxH*ones(length(hrsemitot),1),'--','LineWidth',1);
plot(disctimesemi,-maxH*ones(length(hrsemitot),1),'--','LineWidth',1);
grid on;
xlabel('Time [s]');
ylabel('h_{RW} [Nms]');
title('Angular momentum stored by RWs de-tumbling/slew maneuvering');

%Storing

wgyroslew=wgyro;
hrwslew=hrw;

%% Tracking 
        
        ModCont=1;
        Controlled=1;
        
        Kptrack=Kptrack_track;
        Kdtrack=Kdtrack_track;
        
        phi0=Angtrueslew(1,1,end);
        theta0=Angtrueslew(2,1,end);
        psi0=Angtrueslew(3,1,end);
        w0_vector=wgyroslew(:,1,end);
        hr0=hrwslew(:,end);
        theta0orb=forbitslew(end);
        
        SimTime=2*Period-timedet-timeslew;   %2 periods simulation      
        fprintf('\n                  - Tracking phase t= 80s -> 2 Periods\n')
        out=sim('ModelGroup20.slx'); 

time=out.tout;
wgyro=out.simout.data;                  
Asens=out.simout1.data;                 
hrw=out.simout2.data;                    
abserr=out.simout3.data;                   
staberr=out.simout4.data;                
Torque=squeeze(out.simout7.data);          
disctime=linspace(timedet+timeslew,timedet+timeslew+time(end),length(hrw));

figure(2);
subplot(4,3,3)
plot(disctime,hrw(1,:),'LineWidth',1);
hold on;
plot(disctime,hrw(2,:),'LineWidth',1);
plot(disctime,hrw(3,:),'LineWidth',1);
plot(disctime,hrw(4,:),'LineWidth',1);
grid on;
xlabel('Time [s]');
ylabel('h_{RW} [Nms]');
xlim([disctime(1),disctime(end)]);
title('Angular momentum stored by RWs during tracking');

subplot(4,3,6)
plot(disctime,squeeze(Asens(1,1,:)),'LineWidth',1);
hold on;
plot(disctime,squeeze(Asens(1,2,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(1,3,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(2,1,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(2,2,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(2,3,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(3,1,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(3,2,:)),'LineWidth',1);
plot(disctime,squeeze(Asens(3,3,:)),'LineWidth',1);
grid on;
xlabel('Time [s]');
xlim([disctime(1),disctime(end)]);
title('Measured DCM during tracking');

subplot(4,3,9)
plot(disctime,wgyro(1,:),'LineWidth',1);
hold on;
plot(disctime,wgyro(2,:),'LineWidth',1);
plot(disctime,wgyro(3,:),'LineWidth',1);
grid on;
xlabel('Time [s]');
ylabel('\omega [rad/s]');
xlim([disctime(1),disctime(end)]);
title('Measured angular velocity during tracking');

subplot(4,3,12)
plot(disctime,Torque(1,:),'LineWidth',1);
hold on;
plot(disctime,Torque(2,:),'LineWidth',1);
plot(disctime,Torque(3,:),'LineWidth',1);
grid on;
xlabel('Time [s]');
ylabel('Torque [Nm]');
xlim([disctime(1),disctime(end)]);
title('Applied control torque during tracking');

figure(4)
sgtitle('Tracking')

subplot(3,1,1)
plot(disctime,abserr,'LineWidth',1);
hold on;
grid on;
xlabel('Time [s]');
ylabel('APE [deg]');
xlim([disctime(1),disctime(end)]);
title('APE in time just after slew-maneuver');

abserr=abserr(end/2:end);   %The first period after slew isn't 'nominal'
staberr=staberr(end/2:end); %So now I'll study only the second period simulated

meanerr=mean(abserr);
deverr=std(abserr);


subplot(3,1,2)
plot(disctime(end/2:end),abserr,'LineWidth',1);
hold on;
grid on;
plot(disctime(end/2:end),meanerr*ones(length(hrw)/2+1,1),'--','LineWidth',1);
plot(disctime(end/2:end),3*deverr*ones(length(hrw)/2+1,1),'--','LineWidth',1);
xlabel('Time [s]');
ylabel('APE [deg]');
xlim([disctime(end/2),disctime(end)]);
legend('APE','Mean value','3 \sigma (99.7%)');
title('APE during nominal tracking');


meanstab=mean(staberr);
devstab=std(staberr);

subplot(3,1,3)
plot(disctime(end/2:end),staberr,'LineWidth',1);
hold on;
grid on;
plot(disctime(end/2:end),meanstab*ones(length(hrw)/2+1,1),'--','LineWidth',1);
plot(disctime(end/2:end),3*devstab*ones(length(hrw)/2+1,1),'--','LineWidth',1);
plot(disctime(end/2:end),-3*devstab*ones(length(hrw)/2+1,1),'--','LineWidth',1);
xlabel('Time [s]');
ylabel('APE rate in time [deg/s]');
xlim([disctime(end/2),disctime(end)]);
legend('APE rate','Mean value','3 \sigma (99.7%)');
title('APE rate during nominal tracking');