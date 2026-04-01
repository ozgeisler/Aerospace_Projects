% =========================================================
% plot_bode_custom.m — Styled Bode Plot Utility Function
% Usage: plot_bode_custom(sys, w, channel_name, color)
% =========================================================

function plot_bode_custom(sys, w, channel_name, color)
% PLOT_BODE_CUSTOM  Publication-quality Bode plot
%   sys          — LTI system (tf or ss)
%   w            — frequency vector [rad/s]
%   channel_name — string label
%   color        — hex color string e.g. '#185FA5'

    if nargin < 3, channel_name = 'System'; end
    if nargin < 4, color = '#185FA5'; end

    [mag, phase, wout] = bode(sys, w);
    mag_db = 20*log10(squeeze(mag));
    ph_deg = squeeze(phase);

    % --- Gain crossover ---
    idx_gc = find(diff(sign(mag_db)) ~= 0, 1);
    if ~isempty(idx_gc)
        wgc = interp1(mag_db(idx_gc:idx_gc+1), wout(idx_gc:idx_gc+1), 0);
        pm  = 180 + interp1(wout, ph_deg, wgc);
    else
        wgc = NaN; pm = NaN;
    end

    % --- Phase crossover ---
    idx_pc = find(diff(sign(ph_deg + 180)) ~= 0, 1);
    if ~isempty(idx_pc)
        wpc = interp1(ph_deg(idx_pc:idx_pc+1)+180, wout(idx_pc:idx_pc+1), 0);
        gm  = -interp1(wout, mag_db, wpc);
    else
        wpc = NaN; gm = NaN;
    end

    figure('Name', sprintf('Bode — %s', channel_name), ...
           'Position', [100 100 800 600]);

    % Magnitude
    subplot(2,1,1);
    semilogx(wout, mag_db, 'Color', color, 'LineWidth', 2); hold on;
    yline(0, 'k--', 'LineWidth', 0.8);
    if ~isnan(wgc)
        xline(wgc, '--', sprintf('\\omega_{gc}=%.2f', wgc), ...
            'Color', color, 'Alpha', 0.7);
        plot(wgc, 0, 'o', 'Color', color, 'MarkerFaceColor', color, 'MarkerSize', 6);
    end
    grid on; set(gca, 'XMinorGrid','on');
    ylabel('|G(j\omega)| [dB]', 'FontSize', 11);
    title(sprintf('%s — Bode Diagram  |  GM = %.1f dB,  PM = %.1f°', ...
        channel_name, gm, pm), 'FontSize', 12);
    xlim([wout(1) wout(end)]);

    % Phase
    subplot(2,1,2);
    semilogx(wout, ph_deg, 'Color', color, 'LineWidth', 2); hold on;
    yline(-180, 'r--', '-180°', 'LabelHorizontalAlignment','left');
    if ~isnan(wpc)
        xline(wpc, '--', sprintf('\\omega_{pc}=%.2f', wpc), ...
            'Color','r','Alpha',0.7);
    end
    grid on; set(gca,'XMinorGrid','on');
    xlabel('\omega [rad/s]', 'FontSize', 11);
    ylabel('\angle G(j\omega) [deg]', 'FontSize', 11);
    xlim([wout(1) wout(end)]);

    % Annotations
    annotation('textbox', [0.72 0.44 0.20 0.10], ...
        'String', sprintf('GM = %.2f dB\nPM = %.2f°', gm, pm), ...
        'BackgroundColor', [0.97 0.97 0.97], ...
        'EdgeColor', [0.8 0.8 0.8], 'FontSize', 9);
end
