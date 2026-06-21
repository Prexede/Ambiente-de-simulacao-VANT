function ValidateStructFields(s, requiredFields, structName)
% ValidateStructFields
% -------------------------------------------------------------------------
% Verifica se uma struct possui campos obrigatorios.
% -------------------------------------------------------------------------

    if nargin < 3 || isempty(structName)
        structName = "struct";
    end

    if ~isstruct(s)
        error('%s deve ser uma struct.', structName);
    end

    for i = 1:numel(requiredFields)
        fieldName = char(requiredFields(i));
        if ~isfield(s, fieldName)
            error('%s.%s nao foi definido.', structName, fieldName);
        end
    end
end
