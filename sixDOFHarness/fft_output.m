%%% FIX NYQUIST%%%%

filename = 'outputs_+Fx_0glong.csv';
v = csvread(filename);
Fx = v(:,1);
time = 12.445106962962962; % seconds
L = length(v);
Fs = L/time;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
t = (0:L-1)*T;        % Time vector

%Fx = square(t/.01);

Y = fft(Fx);
Y(1)
% Y(1) = 0; % remove DC component
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;

hold on
figure
plot(t, Fx)
%plot(1000*t(1:L),Fx(1:L))
title('F_{x} measured over time','FontSize', 14)
xlabel('t (seconds)')
ylabel('F_{x} (N)')

figure
plot(f,10*log10(P1)) 
title('Power Spectrum of Fx voltage')
xlabel('f (Hz)')
ylabel('Power/Frequency (dB/Hz)')
hold off

%%
NFFT = length(Fx);
% Power spectrum is computed when you pass a 'power' flag input
[P,F] = periodogram(Fx,[],NFFT,Fs,'power');
helperFrequencyAnalysisPlot2(F,10*log10(P),'Frequency in Hz',...
  'Power spectrum (dBW)',[],[],[-1 1000])
title('Power Spectrum of F_{x}','FontSize', 14)