% =========================================================
% export_report.m — Auto-generate Summary Report (HTML)
% UAV Frequency Domain Analysis — Disturbance Sensitivity
% =========================================================

function export_report(output_path)

if nargin < 1
    output_path = fullfile('..','results','UAV_FDA_Report.html');
end

% Load results
load(fullfile('..','results','data','bode_results.mat'));
load(fullfile('..','results','data','sensitivity_results.mat'));

channels = {'Roll','Pitch','Yaw','Heave'};

%% Build HTML
html = ['<!DOCTYPE html><html><head>' ...
    '<meta charset="UTF-8">' ...
    '<title>UAV FDA Report</title>' ...
    '<style>' ...
    'body{font-family:Arial,sans-serif;max-width:900px;margin:40px auto;color:#222;}' ...
    'h1{color:#185FA5;border-bottom:2px solid #185FA5;padding-bottom:8px;}' ...
    'h2{color:#0F6E56;margin-top:32px;}' ...
    'table{border-collapse:collapse;width:100%;margin:16px 0;}' ...
    'th{background:#185FA5;color:#fff;padding:8px 12px;text-align:left;}' ...
    'td{padding:7px 12px;border-bottom:1px solid #ddd;}' ...
    'tr:nth-child(even){background:#f5f7fa;}' ...
    '.badge{display:inline-block;padding:2px 10px;border-radius:12px;font-size:13px;}' ...
    '.ok{background:#d4edda;color:#155724;}' ...
    '.warn{background:#fff3cd;color:#856404;}' ...
    '.fail{background:#f8d7da;color:#721c24;}' ...
    'img{max-width:100%;border:1px solid #ddd;border-radius:4px;margin:8px 0;}' ...
    '</style></head><body>'];

html = [html '<h1>UAV Frequency Domain Analysis — Disturbance Sensitivity</h1>'];
html = [html '<p><b>Model:</b> Generic 6-DOF UAV &nbsp;|&nbsp; ' ...
    '<b>Analysis:</b> Hover trim linearization &nbsp;|&nbsp; ' ...
    '<b>Date:</b> ' datestr(now, 'dd mmm yyyy') '</p>'];

%% Stability Margins Table
html = [html '<h2>1. Stability Margins</h2>'];
html = [html '<table><tr><th>Channel</th><th>GM [dB]</th><th>PM [°]</th>' ...
    '<th>ωcg [rad/s]</th><th>ωcp [rad/s]</th><th>Status</th></tr>'];

for i = 1:4
    gm = margin_table(i,1); pm = margin_table(i,2);
    if gm >= 6 && pm >= 30
        badge = '<span class="badge ok">STABLE</span>';
    elseif gm >= 3 && pm >= 15
        badge = '<span class="badge warn">MARGINAL</span>';
    else
        badge = '<span class="badge fail">UNSTABLE</span>';
    end
    html = [html sprintf('<tr><td>%s</td><td>%.2f</td><td>%.2f</td>' ...
        '<td>%.4f</td><td>%.4f</td><td>%s</td></tr>', ...
        channels{i}, gm, pm, margin_table(i,3), margin_table(i,4), badge)];
end
html = [html '</table>'];

%% Sensitivity Metrics Table
html = [html '<h2>2. Sensitivity Metrics</h2>'];
html = [html '<table><tr><th>Channel</th><th>||S||∞ [dB]</th>' ...
    '<th>Peak ω [rad/s]</th><th>||T||∞ [dB]</th>' ...
    '<th>Bandwidth [rad/s]</th><th>Status</th></tr>'];

for i = 1:4
    if hinf_S(i) <= 6
        badge = '<span class="badge ok">GOOD</span>';
    elseif hinf_S(i) <= 10
        badge = '<span class="badge warn">MARGINAL</span>';
    else
        badge = '<span class="badge fail">POOR</span>';
    end
    html = [html sprintf('<tr><td>%s</td><td>%.2f</td><td>%.4f</td>' ...
        '<td>%.2f</td><td>%.4f</td><td>%s</td></tr>', ...
        channels{i}, hinf_S(i), w_peak(i), hinf_T(i), bw_S(i), badge)];
end
html = [html '</table>'];

%% Figures
html = [html '<h2>3. Figures</h2>'];
figs = {'bode_openloop.png','stability_margins.png','nyquist.png', ...
        'sensitivity_all.png','sensitivity_comparison.png','waterbed.png', ...
        'psd_disturbance.png','psd_response.png'};
captions = {'Open-loop Bode diagrams','Stability margin summary', ...
            'Nyquist diagrams','Sensitivity functions (all channels)', ...
            'Sensitivity comparison','Waterbed effect (roll)', ...
            'Dryden turbulence PSD','Closed-loop response PSD'};

for i = 1:length(figs)
    fpath = fullfile('figures', figs{i});
    if exist(fullfile('..','results','figures',figs{i}), 'file')
        html = [html sprintf('<p><b>%s</b></p><img src="%s" alt="%s">', ...
            captions{i}, fpath, captions{i})];
    end
end

html = [html '</body></html>'];

%% Write file
fid = fopen(output_path, 'w');
fprintf(fid, '%s', html);
fclose(fid);

fprintf('Report saved: %s\n', output_path);
end
