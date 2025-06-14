classdef DataLogger < handle
    properties
        buffer       % 一時データバッファ（構造体の cell 配列）
        filePath     % 保存するMATファイルのパス
        f
        fileName
        useParpool = false;
        
    end
    
    methods
        function obj = DataLogger(filePath,fname)
            obj.buffer = {};
            obj.filePath = filePath;
            obj.f = parallel.FevalFuture;
            obj.fileName = fname;
        end
        
        function addData(obj, newData)
            obj.buffer{end+1,1} = newData;
        end
        
        function finish = saveData(obj)
            if isempty(obj.buffer)
                finish = 0;
                return; 
            end
            
            % 保存処理を実行            
            matObj = matfile(strcat(obj.filePath,filesep,"userLocal_",obj.fileName,".mat"), 'Writable', true);
            matObj.(obj.fileName) = obj.buffer; % ファイルに保存
            obj.buffer = {};
            finish = 1;
            
        end
        
        function stop(obj)
            if obj.useParpool
                obj.f = parfeval(@obj.saveData,1);
            else
                [~] = obj.saveData();
            end
        end

        function ok = isDone(obj)
            if obj.useParpool
                ok = fetchOutputs(obj.f);
            else
                ok = 1;
            end
        end
    end
end
