%========================================================================%
% 7자유도 ExSLeR의 Joint 1~5에 대해 motor torque to load angle 모델로 
% system identification 수행
% 
% 가진: 멀티사인, openloop, 중력보상만 존재
% 비고: 조인트별 위치, 속도 / (EE의 위치, 속도)를 그려냄
%========================================================================%

clear; clc; close all;
addpath('D:\MCL\Matlab_userdefined_codes')
addpath(genpath('D:\MCL\2025\2025_IROS_PID_optimization'))

%% Flags
DOF = 6;
joint = 3;

if joint == 3
    inertia_list = ["Min", "Mid10", "Mid20", "Mid30", "Mid40", "Mid50", "Mid60", "Max"]; % Min, Mid, Max
%     inertia_list = ["Min", "Mid", "Max"]; % Min, Mid, Max
elseif joint == 6
    inertia_list = "Max";
else
    inertia_list = ["Min", "Max"];
end

%% Data
FILES = cell(length(inertia_list), 1);
for i = 1:length(inertia_list)
    if joint == 3
        FILES{i} = "Data\\" + sprintf("ID_J3_%s", inertia_list(i)) + "_1222(Lang).mat";
    elseif joint == 6
        FILES{i} = "Data\\" + sprintf("ID_J6_%s", inertia_list(i)) + "_0404(Lang).mat";
    else
        FILES{i} = "Data\\" + sprintf("ID_J%d_%s", joint, inertia_list(i)) + "_1226(Lang).mat";
    end
end

%% Plant parameter
Nm = [100, 100, 50, 50, 50, 50];
Jm = [6.85, 14, 7.05, 3.1, 4.42, 3.39]*1e-5;
Bl = [0, 0, 0, 0, 0, 0];

Jl = [2.0045, 2.08, 0.3519, 0.6, 0.049, 0.07];
K  = [4.0, 10, 5.2, 4.0, 2.6, 3.7]*1e3;
Bm = [1.2, 25.3, 8.75, 2.4, 1.7, 1.82]*1e-4;

TFs_struct = struct();

fig = figure;
t = tiledlayout(2, 1, ...
'TileSpacing', 'compact', ...   % 타일 간 간격 최소화
'Padding',     'compact');    

for i = 1:length(inertia_list)
    DATA = load(FILES{i});
    nexttile(1);
    semilogx(DATA.f, 20*log10(abs(DATA.Tra)), 'LineWidth', 1.5);
    title(['TF Magnitude', '\_Joint ', num2str(joint)]);
    xlabel('Frequency [Hz]');
    ylabel('Magnitude [dB]');
    ylim([-120,40])
    xlim([0.1,99])
    grid on; hold on;
    
    nexttile(2);
    semilogx(DATA.f, unwrap(angle(DATA.Tra))*180/pi,'LineWidth', 1.5);
    xlabel('Frequency (Hz)');
    ylabel('Phase (degree)');
    title(['TF Phase', '\_Joint ', num2str(joint)]);
    %     xlim([f_min, f_max]);
    %     ylim([PHASE_min, PHASE_max]);
    legend('Measured', 'Model');
    grid on; hold on;

    INERTIA = inertia_list(i);
switch joint
    case 1
        switch INERTIA
            case "Min"
                Jl = 1.14; Bl = 2; 
                zeta = [0.1, 0.25]; 
                Wn = [31.5, 51.9]*2*pi;
                zeta_an = [0.25, 0.1];
                Wan = [4.5, 37.9]*2*pi;
            case "Max"
                Jl = 3.977; Bl = 8; 
                zeta = [0.05, 0.3]; 
                Wn = 15.3*2*pi;
                zeta_an = [0.07, 0.4];
                Wan = 8.1*2*pi;
        end

    case 2
        switch INERTIA
            case "Min"
                Jl = 0.01; Bl = 0.5; 
                zeta = [0.1, 0.25]; 
                Wn = [19.7, 41.9]*2*pi;
                zeta_an = [0.25, 0.1];
                Wan = [6.3, 23.1]*2*pi;
            case "Max"
                Jl = 4.15; Bl = 6; 
                zeta = [0.05, 0.3]; 
                Wn = [16.1, 43.9]*2*pi;
                zeta_an = [0.07, 0.4];
                Wan = [9.3, 30.7]*2*pi;
        end

    case 3
        switch INERTIA
            case "Min"
                Jl = 0.0138; Bl = 2; 
                zeta = [0.03, 0.05]; 
                Wn = [63.9]*2*pi;
                zeta_an = [0.1, 0.1];
                Wan = [73.1]*2*pi;
            case "Mid"
                Jl = 0.3519; Bl = 4; 
                zeta = [0.45, 0.1]; 
                Wn = [8.7,37.1]*2*pi;
                zeta_an = [0.15, 0.07];
                Wan = [5.7, 35.5]*2*pi;
            case "Mid10"
                Jl = 0.0382; Bl = 4; 
                zeta = [0.05, 0.1]; 
                Wn = [63.5]*2*pi;
                zeta_an = [0.3, 0.1];
                Wan = [71.1]*2*pi;
            case "Mid20"
                Jl = 0.1026; Bl = 3; 
                zeta = [0.6, 0.2]; 
                Wn = [5.7,64.5]*2*pi;
                zeta_an = [0.15, 0.5];
                Wan = [4.7, 53.5]*2*pi;
            case "Mid30"
                Jl = 0.2012; Bl = 4; 
                zeta = [0.7, 0.2]; 
                Wn = [8.1,66.1]*2*pi;
                zeta_an = [0.2, 0.2];
                Wan = [4.9, 72.7]*2*pi;
            case "Mid40"
                Jl = 0.322; Bl = 4; 
                zeta = [0.5, 0.15]; 
                Wn = [8.9,37.5]*2*pi;
                zeta_an = [0.15, 0.1];
                Wan = [5.5, 35.5]*2*pi;
            case "Mid50"
                Jl = 0.4506; Bl = 4; 
                zeta = [0.4, 0.05]; 
                Wn = [9.9,36.9]*2*pi;
                zeta_an = [0.15, 0.05];
                Wan = [6, 35.5]*2*pi;
            case "Mid60"
                Jl = 0.5714; Bl = 4; 
                zeta = [0.28, 0.05]; 
                Wn = [10.7,35.7]*2*pi;
                zeta_an = [0.15, 0.1];
                Wan = [6.7, 31.3]*2*pi;
            case "Max"
                Jl = 0.69; Bl = 6; 
                zeta = [0.23, 0.1]; 
                Wn = [11.9, 34.3]*2*pi;
                zeta_an = [0.1, 0.1];
                Wan = [7.3, 27.1]*2*pi;
        end

    case 4
    switch INERTIA
        case "Min"
            Jl = 0.5; Bl = 2.1; 
            zeta = [0.2, 0.05]; 
            Wn = [13.5, 56.7]*2*pi;
            zeta_an = [0.2, 0.2];
            Wan = [6.9, 50.5]*2*pi;
        case "Max"
            Jl = 0.7; Bl = 2.8; 
            zeta = [0.15, 0.2]; 
            Wn = [13.5, 40.3]*2*pi;
            zeta_an = [0.2, 0.2];
            Wan = [6.9, 30.7]*2*pi;
    end

    case 5
    switch INERTIA
        case "Min"
            Jl = 0.008; Bl = 0.4; 
            zeta = [0.045]; 
            Wn = [63.1]*2*pi;
            zeta_an = [0.05];
            Wan = [71.3]*2*pi;
        case "Max"
            Jl = 0.09; Bl = 1.5; 
            zeta = [0.1]; 
            Wn = [41.9]*2*pi;
            zeta_an = [0.2];
            Wan = [37.9]*2*pi;
    end

    case 6
        Jl = 0.07; Bl = 0.6; 
        zeta = [0.1]; 
        Wn = 51.9*2*pi;
        zeta_an = [0.05];
        Wan = 60.7*2*pi;
end


%% Plotting
s = tf('s');

switch joint
    case 1
        switch INERTIA
            case 'Min'
                TF = 8000/(Jl*s^2+Bl*s) ...
                    * (s^2 + 2*zeta_an(1)*Wan(1)*s + Wan(1)^2)/(s^2 + 2*zeta(1)*Wn(1)*s + Wn(1)^2)...
                    * (s^2 + 2*zeta_an(2)*Wan(2)*s + Wan(2)^2)/(s^2 + 2*zeta(2)*Wn(2)*s + Wn(2)^2)...
                    * 1/(1/20*s + 1)^2;
            case 'Max'
                TF = 250/(Jl*s^2+Bl*s) ...
                    * (s^2 + 2*zeta_an(1)*Wan(1)*s + Wan(1)^2)/(s^2 + 2*zeta(1)*Wn(1)*s + Wn(1)^2);
        end

    case 2
        switch INERTIA
            case 'Min'
                TF = 50/(Jl*s^2+Bl*s) ...
                    * (s^2 + 2*zeta_an(1)*Wan(1)*s + Wan(1)^2)/(s^2 + 2*zeta(1)*Wn(1)*s + Wn(1)^2)...
                    * (s^2 + 2*zeta_an(2)*Wan(2)*s + Wan(2)^2)/(s^2 + 2*zeta(2)*Wn(2)*s + Wn(2)^2)...
                    * 1/(1/20*s + 1)^2;
            case 'Max'
                TF = 200/(Jl*s^2+Bl*s) ...
                    * (s^2 + 2*zeta_an(1)*Wan(1)*s + Wan(1)^2)/(s^2 + 2*zeta(1)*Wn(1)*s + Wn(1)^2)...
                    * (s^2 + 2*zeta_an(2)*Wan(2)*s + Wan(2)^2)/(s^2 + 2*zeta(2)*Wn(2)*s + Wn(2)^2)...
                    * 1/(1/Wn(2)*s + 1);
        end

    case 3
        switch INERTIA
            case "Min"
                TF= 40/(Jl*s^2+Bl*s) ...
                    * (s^2 + 2*zeta_an(1)*Wan(1)*s + Wan(1)^2)/(s^2 + 2*zeta(1)*Wn(1)*s + Wn(1)^2)...
                    * 1/(1/20*s + 1);
            case "Mid10"
                TF = 80/(Jl*s^2+Bl*s) ...
                    * (s^2 + 2*zeta_an(1)*Wan(1)*s + Wan(1)^2)/(s^2 + 2*zeta(1)*Wn(1)*s + Wn(1)^2)...
                    * 1/(1/20*s + 1);
            case "Mid20"
                TF = 110/(Jl*s^2+Bl*s) ...
                    * (s^2 + 2*zeta_an(1)*Wan(1)*s + Wan(1)^2)/(s^2 + 2*zeta(1)*Wn(1)*s + Wn(1)^2)...
                    * (s^2 + 2*zeta_an(2)*Wan(2)*s + Wan(2)^2)/(s^2 + 2*zeta(2)*Wn(2)*s + Wn(2)^2)...
                    * 1/(1/(1.3*Wn(1))*s + 1);
            case "Mid30" %피팅 별로
                TF = 110/(Jl*s^2+Bl*s) ...
                    * (s^2 + 2*zeta_an(1)*Wan(1)*s + Wan(1)^2)/(s^2 + 2*zeta(1)*Wn(1)*s + Wn(1)^2)...
                    * (s^2 + 2*zeta_an(2)*Wan(2)*s + Wan(2)^2)/(s^2 + 2*zeta(2)*Wn(2)*s + Wn(2)^2)...
                    * 1/(1/60*s + 1);
            case "Mid40"
                TF = 110/(Jl*s^2+Bl*s) ...
                    * (s^2 + 2*zeta_an(1)*Wan(1)*s + Wan(1)^2)/(s^2 + 2*zeta(1)*Wn(1)*s + Wn(1)^2)...
                    * (s^2 + 2*zeta_an(2)*Wan(2)*s + Wan(2)^2)/(s^2 + 2*zeta(2)*Wn(2)*s + Wn(2)^2)...
                    * 1/(1/Wn(2)*s + 1);
            case "Mid50"
                TF = 110/(Jl*s^2+Bl*s) ...
                    * (s^2 + 2*zeta_an(1)*Wan(1)*s + Wan(1)^2)/(s^2 + 2*zeta(1)*Wn(1)*s + Wn(1)^2)...
                    * (s^2 + 2*zeta_an(2)*Wan(2)*s + Wan(2)^2)/(s^2 + 2*zeta(2)*Wn(2)*s + Wn(2)^2)...
                    * 1/(1/Wn(2)*s + 1);
            case "Mid60"
                TF = 110/(Jl*s^2+Bl*s) ...
                    * (s^2 + 2*zeta_an(1)*Wan(1)*s + Wan(1)^2)/(s^2 + 2*zeta(1)*Wn(1)*s + Wn(1)^2)...
                    * (s^2 + 2*zeta_an(2)*Wan(2)*s + Wan(2)^2)/(s^2 + 2*zeta(2)*Wn(2)*s + Wn(2)^2)...
                    * 1/(1/Wn(2)*s + 1);
            case "Max"
                TF = 150/(Jl*s^2+Bl*s) ...
                    * (s^2 + 2*zeta_an(1)*Wan(1)*s + Wan(1)^2)/(s^2 + 2*zeta(1)*Wn(1)*s + Wn(1)^2)...
                    * (s^2 + 2*zeta_an(2)*Wan(2)*s + Wan(2)^2)/(s^2 + 2*zeta(2)*Wn(2)*s + Wn(2)^2)...
                    * 1/(1/Wn(2)*s + 1);
        end

    case 4
        switch INERTIA
            case 'Min'
                TF = 150/(Jl*s^2+Bl*s) ...
                    * (s^2 + 2*zeta_an(1)*Wan(1)*s + Wan(1)^2)/(s^2 + 2*zeta(1)*Wn(1)*s + Wn(1)^2)...
                    * (s^2 + 2*zeta_an(2)*Wan(2)*s + Wan(2)^2)/(s^2 + 2*zeta(2)*Wn(2)*s + Wn(2)^2);
            case 'Max'
                TF = 250/(Jl*s^2+Bl*s) ...
                    * (s^2 + 2*zeta_an(1)*Wan(1)*s + Wan(1)^2)/(s^2 + 2*zeta(1)*Wn(1)*s + Wn(1)^2)...
                    * (s^2 + 2*zeta_an(2)*Wan(2)*s + Wan(2)^2)/(s^2 + 2*zeta(2)*Wn(2)*s + Wn(2)^2);
        end

    case 5
        switch INERTIA
            case 'Min'
                TF = 20/(Jl*s^2+Bl*s) ...
                    * (s^2 + 2*zeta_an(1)*Wan(1)*s + Wan(1)^2)/(s^2 + 2*zeta(1)*Wn(1)*s + Wn(1)^2)...
                    * 1/(1/35*s+1);
            case 'Max'
                TF = 80/(Jl*s^2+Bl*s) ...
                    * (s^2 + 2*zeta_an(1)*Wan(1)*s + Wan(1)^2)/(s^2 + 2*zeta(1)*Wn(1)*s + Wn(1)^2)...
                    * 1/(1/80*s+1);
        end  

    case 6
        TF = 20/(Jl*s^2+Bl*s) ...
            * (s^2 + 2*zeta_an(1)*Wan(1)*s + Wan(1)^2)/(s^2 + 2*zeta(1)*Wn(1)*s + Wn(1)^2);
end

TFs_struct.(INERTIA) = TF;

[magG phaseG w] = bode(TF,2*pi*DATA.f);
nexttile(1); semilogx(DATA.f,20*log10(abs(magG(:))),'r--','Linewidth',1.5);
nexttile(2); semilogx(DATA.f, squeeze(phaseG),'r--','Linewidth',1.5);

end

% save(sprintf("Joint%d_TFs_struct.mat", joint), 'TFs_struct');

%% H_inf based controller design 
P  = ss(TF);                % 7-state 식별 플랜트 (torque→load-angle)

Ws = makeweight(0.01,[2*pi*5,1],100);
Wt = makeweight(10^0.1,[2*pi*40,1],0.01,0,4);
% Wu = makeweight(0.01,1e2, 50);   % 구동토크 절제

% Paug = augw(P,Ws,[],Wt);    %   [u]←Wu← K  ┐
%                          %   [y]←Wt←  ─┘ │     (표준 H∞ 구조)
% size(Paug)
% [Khinf, CL, GAM] = hinfsyn(Paug);
% disp(['GAM = ', num2str(GAM)]); % GAM 값 출력
% Kpd   = balred(Khinf,1);    % 1-state → filtered-PD
% [A,B,C,D] = ssdata(Kpd);
% 
% N  = -A; Kp = D; Kd = (C*B)/N;   % (Kp,Kd,N) 추출
% fprintf('Kp = %g, Kd = %g, N = %g\n', Kp, Kd, N);

N  = 100; Kp = 5; Kd = 0.5; 
Cpd = Kp + Kd*s/(1+s/N);
L = P*Cpd;
S   = feedback(1, P*Cpd);
T   = 1 - S;

f = logspace(-1,2,500);       % 0.1 ~ 100 Hz
w = 2*pi*f;                   % rad/s

[magS,phS] = bode(S,w);   magS = squeeze(magS);
[magT,phT] = bode(T,w);   magT = squeeze(magT);
Lresp = squeeze(freqresp(L, w));  % 복소수 응답
[magWs,~ ] = bode(Ws,w);  magWs = squeeze(magWs);
[magWt,~ ] = bode(Wt,w);  magWt = squeeze(magWt);

figure;
subplot(2,1,1)
semilogx(f,20*log10([magS magT]))
xlabel('Frequency [Hz]'); ylabel('Magnitude [dB]'); grid on; hold on;
legend('S','T')

subplot(2,1,1)
semilogx(f,20*log10([magWs magWt]),'k--')
xlabel('Frequency [Hz]'); ylabel('Magnitude [dB]'); grid on
legend('S','T','W_s','W_t')

subplot(2,1,2)
semilogx(f,[phS(:) phT(:)]); grid on
xlabel('Frequency [Hz]'); ylabel('Phase [deg]')
legend('S','T')

figure;
plot(real(Lresp), imag(Lresp),'r', 'LineWidth',1.5); hold on;
n = 1000; 
r = 0.5;
theta = linspace(0, 2*pi, n);
x = r*cos(theta);
y = r*sin(theta);
plot(x-1, y, 'k'); grid on;

title('Nyquist of L(s)'); 
xlim([-3,0.5]), ylim([-2,0.5])

% setFigurePositions(1,3500,1200);
setFigurePositions(4,800,1000);
