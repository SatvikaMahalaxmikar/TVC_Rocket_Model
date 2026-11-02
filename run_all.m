%RUN_ALL  Headless pipeline: build Simulink model, verify against the script loop,
%   export reference data for Python, then run the robustness studies.
%   No figure window and no Simulink editor is ever opened; plots go to ../results/*.png.
set(0, 'DefaultFigureVisible', 'off');
compare_simulink_vs_script;
export_for_python;
study_sweeps;
study_montecarlo;
