clear; clc; close all;
addpath('D:\MCL\Matlab_userdefined_codes')

%% Flags
joint = 2;
mode_all = {'min', 'mid', 'max', 'mid15', 'mid30', 'mid40', 'mid45', 'mid50', 'mid60', 'mid75'};
mode = mode_all{3};

% J1
% Data_min = readmatrix('241226_FRF_J1,2\241226_J1_min_ID_0.1~100Hz_amp0.5.txt'); %750
% Data_mid = readmatrix('241226_FRF_J1,2\241226_J1_mid_ID_0.1~100Hz_amp0.3.txt'); %754
% Data_max = readmatrix('241226_FRF_J1,2\241226_J1_max_ID_0.1~100Hz_amp0.7.txt'); %602

% J2
% Data_min = readmatrix('241226_FRF_J1,2\241226_J2_min_ID_0.1~100Hz_amp0.5.txt'); %902
% Data_mid = readmatrix('241226_FRF_J1,2\241226_J2_mid_ID_0.1~100Hz_amp0.5.txt'); %703
% Data_max = readmatrix('241226_FRF_J1,2\241226_J2_max_ID_0.1~100Hz_amp0.6.txt'); %802
% Data_mid15 = readmatrix('250118_FRF_J1,2\250409_J2_mid15(J4_15,J6_45)_ID_0.1~100Hz_amp0.6.txt');
% Data_mid30 = readmatrix('250118_FRF_J1,2\250118_J2_mid30(J4_30,J6_90)_ID_0.1~100Hz_amp0.6.txt'); 
% Data_mid60 = readmatrix('250118_FRF_J1,2\250118_J2_mid60(J4_60,J6_90)_ID_0.1~100Hz_amp0.6.txt'); 
% Data_mid75 = readmatrix('250118_FRF_J1,2\250409_J2_mid75(J4_75,J6_45)_ID_0.1~100Hz_amp0.6.txt');

% J3
% Data_min = readmatrix('241222_FRF_J3 (fixed)\241222_J3_min_ID_0.1~100Hz_amp0.35.txt'); %851
% Data_mid10 = readmatrix('241222_FRF_J3 (fixed)\241222_J3_mid10_ID_0.1~100Hz_amp0.35.txt'); %651
% Data_mid20 = readmatrix('241222_FRF_J3 (fixed)\241222_J3_mid20_ID_0.1~100Hz_amp0.35.txt'); %1752
% Data_mid30 = readmatrix('241222_FRF_J3 (fixed)\241222_J3_mid30_ID_0.1~100Hz_amp0.35.txt'); %801
% Data_mid40 = readmatrix('241222_FRF_J3 (fixed)\241222_J3_mid40_ID_0.1~100Hz_amp0.35.txt'); %801
% Data_mid45 = readmatrix('241222_FRF_J3 (fixed)\241222_J3_mid45_ID_0.1~100Hz_amp0.35.txt'); %1054
% Data_mid50 = readmatrix('241222_FRF_J3 (fixed)\241222_J3_mid50_ID_0.1~100Hz_amp0.35.txt'); %951
% Data_mid60 = readmatrix('241222_FRF_J3 (fixed)\241222_J3_mid60_ID_0.1~100Hz_amp0.35.txt'); %950
% Data_max = readmatrix('241222_FRF_J3 (fixed)\241222_J3_max_ID_0.1~100Hz_amp0.35.txt'); %1301

% J4
% Data_min = readmatrix('241223_FRF_J4,5\241223_J4_min_ID_0.1~100Hz_amp0.2.txt'); %952
% Data_mid = readmatrix('241223_FRF_J4,5\241223_J4_mid_ID_0.1~100Hz_amp0.2.txt'); %1551
% Data_max = readmatrix('241223_FRF_J4,5\241223_J4_max_ID_0.1~100Hz_amp0.2.txt'); %595

% J5
% Data_min = readmatrix('241223_FRF_J4,5\241223_J5_min_ID_0.1~100Hz_amp0.2.txt'); %953
% Data_mid = readmatrix('241223_FRF_J4,5\241223_J5_mid_ID_0.1~100Hz_amp0.2.txt'); %701
% Data_max = readmatrix('241223_FRF_J4,5\241223_J5_max_ID_0.1~100Hz_amp0.2.txt'); %652

% J6
% Data_max = readmatrix('250404_FRF_J6\250404_J6_ID_0.1~100Hz_amp0.2.txt'); %699

if strcmp(mode, 'min')
    DATA = Data_min;
    init = 902;  
elseif strcmp(mode, 'mid15')
    DATA = Data_mid15;
    init = 2451;     
elseif strcmp(mode, 'mid20')
    DATA = Data_mid20;
    init = 1752; 
elseif strcmp(mode, 'mid30')
    DATA = Data_mid30;
    init = 801; 
elseif strcmp(mode, 'mid40')
    DATA = Data_mid40;
    init = 801; 
elseif strcmp(mode, 'mid45')
    DATA = Data_mid45;
    init = 1054; 
elseif strcmp(mode, 'mid50')
    DATA = Data_mid50;
    init = 951;     
elseif strcmp(mode, 'mid75')
    DATA = Data_mid75;
    init = 1199;     
elseif strcmp(mode, 'mid')
    DATA = Data_mid;
    init = 1753;
elseif strcmp(mode, 'max')  
    DATA = Data_max;
    init = 802; 
elseif strcmp(mode, 'ClosedID') 
    DATA = Data;
    init = 2052;
end

fin = init + 30000 - 1;

M_ang = DATA(init:fin,joint);
L_ang = DATA(init:fin,joint + 7);
input = DATA(init:fin,joint + 21);

M_vel = zeros(length(M_ang),1);
M_vel(1) = 0;
L_vel = zeros(length(L_ang),1);
L_vel(1) = 0;

for i = 1:length(M_ang)-1
    tau_v = 1/(2*pi*100);
    M_vel(i+1) = (2*(M_ang(i+1) - M_ang(i)) - (0.001 - 2*tau_v)*M_vel(i)) / (0.001 + 2*tau_v);
    L_vel(i+1) = (2*(L_ang(i+1) - L_ang(i)) - (0.001 - 2*tau_v)*L_vel(i)) / (0.001 + 2*tau_v);
end
M_vel=circshift(M_vel,-3);
L_vel=circshift(L_vel,-3);

output_mang = M_ang;
output_lang = L_ang;
output_mvel = M_vel;
output_lvel = L_vel;

[input_avg, output_lvel_avg] = ID_average(input, output_lvel, 3, 1);

N = 1000; % 몇 Hz까지 볼건지
[f, Tra] = FFT_func(input_avg, output_lvel_avg, N);

dt = 0.001; % 샘플링 시간 (1 kHz)
time = (0:dt:(length(output_lang)-1)*dt)'; % 시간 벡터 생성
l_ang_with_time = [time, output_lang]; % Nx2 배열 생성


if strcmp(mode, 'min')  
    save('ID_J4_Min_0814', 'Tra','f');
elseif strcmp(mode, 'mid15')
    save('ID_J2_Mid15_0409', 'Tra','f'); 
elseif strcmp(mode, 'mid20')
    save('ID_J3_Mid20_1222', 'Tra','f'); 
elseif strcmp(mode, 'mid30')
    save('ID_J3_Mid30_1222', 'Tra','f'); 
elseif strcmp(mode, 'mid40')
    save('ID_J3_Mid40_1222', 'Tra','f'); 
elseif strcmp(mode, 'mid45')
    save('ID_J3_Mid45_1222', 'Tra','f'); 
elseif strcmp(mode, 'mid50')
    save('ID_J3_Mid50_1222', 'Tra','f'); 
elseif strcmp(mode, 'mid75')
    save('ID_J2_Mid75_0409', 'Tra','f');
elseif strcmp(mode, 'mid')  
    save('ID_J4_Mid_0809', 'Tra','f');
elseif strcmp(mode, 'max')  
    save('ID_J4_Max_0814', 'Tra','f');
end


if strcmp(mode, 'min')
   legend('min') 
elseif strcmp(mode, 'mid15')
    legend('mid60')  
elseif strcmp(mode, 'mid20')
    legend('mid60')  
elseif strcmp(mode, 'mid30')
    legend('mid60')  
elseif strcmp(mode, 'mid40')
    legend('mid60')  
elseif strcmp(mode, 'mid45')
    legend('mid60')  
elseif strcmp(mode, 'mid50')
    legend('mid60')  
elseif strcmp(mode, 'mid75')
    legend('mid60')   
elseif strcmp(mode, 'mid')  
    legend('mid')
elseif strcmp(mode, 'max')  
    legend('max')
elseif strcmp(mode, 'ClosedID')  
legend('ClosedID')
end

fs = 1000;                  % 샘플링 주파수 [Hz]
window = hanning(1000);     % 또는 다른 길이로 조절
noverlap = 500;
% nfft = 2048;
nfft = 5000;

function [input_avg, output_avg] = ID_average(input, output, n_repeat, n_transient)
    % 입력 데이터 크기 확인
    data_length = length(input);
    
    % 각 반복의 데이터 길이 계산
    single_length = data_length / n_repeat;
    
    % 데이터 길이 검증
    if mod(data_length, n_repeat) ~= 0
        error('Data length must be divisible by n_repeat.');
    end
    
    % 유효 반복 구간 선택 (n_transient 제외)
    valid_repeats = (n_transient + 1):n_repeat;
    
    % 평균값 저장을 위한 변수 초기화
    input_sum = zeros(single_length, 1);
    output_sum = zeros(single_length, 1);
    valid_count = length(valid_repeats);
    
    % 유효 구간 반복하여 데이터 누적
    for i = valid_repeats
        start_idx = (i-1) * single_length + 1;
        end_idx = i * single_length;
        
        input_sum = input_sum + input(start_idx:end_idx);
        output_sum = output_sum + output(start_idx:end_idx);
    end
    
    % 평균값 계산
    input_avg = input_sum / valid_count;
    output_avg = output_sum / valid_count;
end

function [f,Tra] = FFT_func(input,output, N)
      
    fs = 1000;            % Sampling frequency
    T = 1/fs;             % Sampling period
    L = length(input);    % Length of signal
    t = (0:L-1)*T;        % Time vector
    f = fs/L*(0:(L/2));
    w = 2*pi*f;

    U = fft(input);
    Y = fft(output);
    puu= U.*conj(U);
    puy= Y.*conj(U);
    
    P2 = abs(U/L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1);

    P4 = abs(Y/L);
    P3 = P4(1:L/2+1);
    P3(2:end-1) = 2*P3(2:end-1);

    Tra = puy./puu;
    f = f(2:2:N+1);       % 0.1, 0.3, 0.5, ... (500개 데이터)       
    Tra = Tra(2:2:N+1);
    
    Mag_Tra = 20*log10(abs(Tra));
    Phase_Tra = angle(Tra)*180/pi;
    
    figure,
    subplot(2,1,1), semilogx(f,Mag_Tra)
    title('TF Mag')
    xlabel('f (Hz)')
    ylabel('Magnitude')
    set(gca, 'XLim', [0.1 100]);
    set(gca, 'YLim', [-80, 20]);
    hold on
    subplot(2,1,2), semilogx(f,Phase_Tra);
    set(gca, 'XLim', [0.1 100]);
end
