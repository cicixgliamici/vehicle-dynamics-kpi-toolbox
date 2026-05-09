function plotFrequencyResponse(frResults, outputFile)
%PLOTFREQUENCYRESPONSE Plot Bode and Coherence diagrams.
%
%   plotFrequencyResponse(frResults, outputFile) generates a 3-panel plot 
%   showing Magnitude (Gain), Phase, and Coherence.

figure('Visible', 'off');

f = frResults.Frequency_Hz;

% 1. Magnitude Plot
subplot(3,1,1);
semilogx(f, 20*log10(abs(frResults.TF_Yaw)), 'b', 'LineWidth', 1.5, 'DisplayName', 'Yaw Rate');
hold on;
semilogx(f, 20*log10(abs(frResults.TF_LatAcc)), 'r', 'LineWidth', 1.5, 'DisplayName', 'Lat Accel');
ylabel('Gain [dB]');
grid on;
title('Frequency Response (Steering Input)');
legend('Location', 'best');

% 2. Phase Plot
subplot(3,1,2);
semilogx(f, unwrap(angle(frResults.TF_Yaw))*180/pi, 'b', 'LineWidth', 1.5);
hold on;
semilogx(f, unwrap(angle(frResults.TF_LatAcc))*180/pi, 'r', 'LineWidth', 1.5);
ylabel('Phase [deg]');
grid on;

% 3. Coherence Plot
subplot(3,1,3);
semilogx(f, frResults.Coh_Yaw, 'b--', 'LineWidth', 1.0, 'DisplayName', 'Coh Yaw');
hold on;
semilogx(f, frResults.Coh_Lat, 'r--', 'LineWidth', 1.0, 'DisplayName', 'Coh Lat');
ylabel('Coherence');
xlabel('Frequency [Hz]');
ylim([0 1.1]);
grid on;

if nargin > 1 && ~isempty(outputFile)
    saveas(gcf, outputFile);
end

end
