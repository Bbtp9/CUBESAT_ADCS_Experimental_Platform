function lh = safe_add_line(modelName, src, dest)
% SAFE_ADD_LINE Safely connects two ports in Simulink, clearing any existing
% connections to the destination port first to avoid "destination already connected" errors.

    % Parse destination: e.g., 'Switch_Tau/1'
    tokens = regexp(dest, '^([^/]+)/(\d+)$', 'tokens');
    if ~isempty(tokens)
        block = tokens{1}{1};
        portNum = str2double(tokens{1}{2});
        blockPath = [modelName '/' block];
        try
            ph = get_param(blockPath, 'PortHandles');
            if portNum <= length(ph.Inport)
                lineHandle = get_param(ph.Inport(portNum), 'Line');
                if lineHandle ~= -1
                    delete_line(lineHandle);
                end
            end
        catch
            % Ignore if block or port doesn't exist
        end
    end
    
    % Add the line with autorouting
    lh = add_line(modelName, src, dest, 'autorouting', 'on');
end
