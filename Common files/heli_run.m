function[] = heli_run()
% try
%     qc_start_model
% catch ME 
%     qc_clean_model
%     qc_build_model
%     qc_start_model
qc_clean_model
qc_build_model
qc_start_model
end