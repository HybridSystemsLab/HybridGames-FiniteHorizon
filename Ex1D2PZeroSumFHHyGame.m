%--------------------------------------------------------------------------
% Matlab M-file Project: 2-Player Zero-Sum FHor HyGames @  Hybrid Systems Laboratory (HSL), 
% Filename: Ex1D2PZeroSumFHHyGame.m
%--------------------------------------------------------------------------
% Project: Motivational Example - 1D Linear Quadratic FH Hybrid Game with Nonunique Solutions
% Author: Santiago Jimenez Leudo
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%   Make sure to install HyEQ Toolbox (Beta) v3.0.0.22 from
%   https://www.mathworks.com/matlabcentral/fileexchange/102239-hybrid-equations-toolbox-beta 
%   (View Version History) 
%   Copyright @ Hybrid Systems Laboratory (HSL),
%   Revision: 0.0.0.1 Date: 04/05/2022 17:41:00

clear all
clc 
% --------------------------------------------------------------
%%% Initialization
% --------------------------------------------------------------
%   Paremeters: a, b1, b2, delta, QC, RC1, RC2, sigma, mu, P, xi 
%   Modify any parameter in this section to simulate the system/case of interest

% Simulation Horizon
TSPAN=[0 0.6];    %Second entry is the maximum amount of seconds allowed
JSPAN = [0 2];    %Second entry is the maximum amount of jumps allowed
Ts=0.001;         %Steptime
t=0:Ts:TSPAN(2);

%%% Continuous Dynamics
%f(x,uC)=a*x+b*uC     Flow Map
%b=[b1;b2];
%C=[0,delta]              Flow Set
%uC=(uc1,uc2)         Continuos Input
a=-1;
b1=1;
b2=1;
delta=2;

%%% Stage Cost during Flows
QC=1;  
RC1=1.304;
RC2=-4;
Lc=@(x,u1,u2) x^2*QC+u1^2*RC1+u2^2*RC2;

%%% Discrete Dynamics
%g(x,uD)=sigma
%D={mu}
sigma=0.5;
mu=1;

%%% Stage Cost during Jumps
P=0.4481; % Caluculated as the positive answer of the robust CARE:
%   syms Pv
%   P=simplify(solve(Q+2*Pv*a-Pv^2/R1-Pv^2/R2==0,Pv))
[tp,p]=   riccatiEquationRunner(P,TSPAN(2),a,QC,b1^2/RC1+b2^2/RC2);
Ld=@(x,pv) (pv)*(x^2-0.5^2);

%%% Discrete Input
ud=0;

%%% Lyapunov Function
V=@(x, pv) pv*x^2;

%%% Initial State 
xi = 2;

% --------------------------------------------------------------
%%% System Evolution + Cost Computation
% --------------------------------------------------------------
%
for jump=[0 1] %Run the continuous solution when j=0 and 
               %    the hybrid solution when j=1.
               %At x=1, the solution can either flow or jump
    if jump>0
        clear x uc1 uc2 J
    end
    
    x(1)=xi;                % Initial State
    uc1(1)=-b1*P*x(1)/RC1;  % Intial Input Player P1
    uc2(1)=-b2*P*x(1)/RC2;  % Intial Input Player P2
    J(1)=0;                 % Initial Cost
    
    for i=1:length(t)-1
        ip=min(find(abs(tp-t(i)) <= 0.016))
        if abs(x(i)-mu)>=0.01 && 0<=x(i)<=2    % Check if x is in the Flow Set
            x(i+1)=x(i)+(t(i+1)-t(i))*(a*x(i)+b1*uc1(i)+b2*uc2(i)); % Evolve via flow
            J(i+1)=J(i)+(t(i+1)-t(i))*(Lc(x(i),uc1(i),uc2(i)));     % Add Continuous Cost
        else                                   % Check if x is in the Jump Set
            if jump==1                         % Hybrid Solution
                J(i)=J(i)+Ld(x(i),p(ip));      % Add Discrete Cost
                x(i)=sigma;                    % Evolve via jump 
                uc1(i)=-b1*p(ip)*x(i)/RC1;     % Update Input P1 for flow
                uc2(i)=-b2*p(ip)*x(i)/RC2;     % Update Input P2 for flow
                jj=i                           % Save index of jump
            end
            x(i+1)=x(i)+(t(i+1)-t(i))*(a*x(i)+b1*uc1(i)+b2*uc2(i)); % Evolve via flow after jump
            J(i+1)=J(i)+(t(i+1)-t(i))*Lc(x(i),uc1(i),uc2(i));
        end
        
        uc1(i+1)=-b1*p(ip)*x(i+1)/RC1;          % Update Input P1 for next time step
        uc2(i+1)=-b2*p(ip)*x(i+1)/RC2;          % Update Input P2 for next time step
    end
    J(i+1)=J(i+1)+V(x(i+1),p(end));             % Terminal Cost
    
    if jump==0              % Store Continuous Solution Data 
        phik=x;
        uk=uc1;
        wk=uc2;
        Jk=J;
    else                    % Store Hybrid Solution Data
        phih=x;
        uh=uc1;
        wh=uc2;
        Jh=J;
    end
end

% Create Hybrid Solution by using HyEQ Toolbox
jv=[linspace(0,0,jj-1), linspace(1,1,size(t,2)-jj+1)];  % Discrete Time 

solphih=HybridArc(t',jv',phih');        % Hybrid Response
soluh=HybridArc(t',jv',uh');            % P1 Action for Hybrid Solution
solwh=HybridArc(t',jv',wh');            % P2 Action for Hybrid Solution
solcost=HybridArc(t',jv',Jh');          % Cost of Hybrid Solution

%
% --------------------------------------------------------------
%%% Plot
% --------------------------------------------------------------
%
clf
figure(1)
set(0,'defaultfigurecolor',[1 1 1])
set(0,'defaulttextinterpreter','latex')
set(gcf,'color','w');

plot_builder_bb = HybridPlotBuilder();

subplot(3,1,1)
plot_builder_bb.flowColor('#017daa') ...            $ Set Plot Properties  
    .legend('$\phi_h$')...
    .slice(1)...
    .labels('$x_1$') ...
    .autoSubplots('off')...
    .configurePlots(@apply_plot_settings)...
    .jumpColor('#e43d43')...
    .plotFlows(solphih);                            % Plot Hybrid Response
hold on
pk=plot(t,phik, 'color', '#3f9f38')                 % Plot Continuos Response
plot_builder_bb.addLegendEntry(pk,'$\phi_c$');
ylabel('$x$','Interpreter','Latex')


subplot(3,1,2)
plot_builder_bb.flowColor('#017daa') ...            $ Set Plot Properties
    .legend('$u_h$')...
    .slice([1])...
    .configurePlots(@apply_plot_settings)...
    .autoSubplots('off')...
    .jumpColor('#e43d43')...
    .plotFlows(soluh);                              % Plot P1 Action for HySolution
hold on
plot_builder_bb.flowColor('#017daa') ...            $ Set Plot Properties    
    .legend('')...
    .slice([1])...
    .configurePlots(@apply_plot_settings)...
    .autoSubplots('off')...
    .jumpColor('#e43d43')...
    .plotFlows(solwh);                              % Plot P2 Action for HySolution
hold on
puk=plot(t,uk,'color','#3f9f38')                    % Plot P1 Action for ContSolution
hold on
pwk=plot(t, wk,'color','#3f9f38')                   % Plot P2 Action for ContSolution
hold on
plot_builder_bb.addLegendEntry(puk,'$u_c$');
ylabel('$u_C$','Interpreter','Latex')


subplot(3,1,3)
plot_builder_bb.flowColor('#017daa') ...            % Set Plot Properties
    .legend('$J(\xi,u_h)$')...
    .labels('$J$') ...
    .configurePlots(@apply_plot_settings)...
    .autoSubplots('off')...
    .jumpColor('#e43d43')...
    .legend('$J(2,0,0,u_h)$','Location', 'southeast')...)...
    .plotFlows(solcost)                             % Plot Cost of Hybrid Solution
hold on
ck=plot(t,Jk, 'color', '#3f9f38')                   % Plot Cost of Continuous Solution
hold on
V0=V(xi,P);
X=ones(1,size(t,2))* V0;
vf=plot(t,X, 'color', 'black');                     % Estimated Optimal Cost
set(gca,'TickLabelInterpreter','latex')
plot_builder_bb.addLegendEntry(ck,'$J(2,0,0,u_c)$');
plot_builder_bb.addLegendEntry(vf,'$J^*(2,0,0)$');
if xi==2
    yticks([0 .5 1 1.5 V0])
    yticklabels({'0' '0.5' '1' '1.5' '$V(0,0,2)$'})
end

function apply_plot_settings(ax,component)
xlabel('$t$ [s]')
ax.TickLabelInterpreter='latex';
ax.LineWidth=0.25;
end


function [t,p]= riccatiEquationRunner(P,T,a,Q,B)
par = [-Q;-2*a;B];  % q0, q1, and q2
p0 = P;
ti = 0; tf = T;
opt = odeset('InitialStep',0.001,'AbsTol',1.0e-07,'RelTol',1.0e-07);
[t,p] = ode45( @riccatiEquation, [ti,tf], p0 ,opt, par);
% Visualize
%plot(t,p)
end
function dpdx = riccatiEquation(x,p,parameters)
q0 = parameters(1);
q1 = parameters(2);
q2 = parameters(3);
dpdx = q0 + q1*p + q2*p*p;
end