classdef SharedMem
    %==================================================================
    %   SharedMem  ─ ultra-light shared memory helper
    %   • Writer side passes VARS (name / type / maxSize)
    %   • Reader side needs only the file name
    %   • Double-buffer + busy-flag, memmapfile-based
    %==================================================================

    %% ────────────────────────── CONSTANTS ──────────────────────────
    properties (Constant, Access = private)
        typeBytes = struct( ...
            'double',8,'single',4, ...
            'uint64',8,'int64',8,'uint32',4,'int32',4, ...
            'uint16',2,'int16',2,'uint8',1,'int8',1);
    end

    %% ─────────────────────────── FIELDS ────────────────────────────
    properties (Access = private)
        mm                  % memmapfile object
        varInfo             % struct array per variable
        oneBufBytes double  % bytes per buffer (double for safe math)
        hdrLen      double  % header length
        buf0Start   double  % start index of buffer-0 in the payload
    end

    %% ────────────────────────── PUBLIC API ─────────────────────────
    methods
        %======================= CONSTRUCTOR =========================
        function obj = SharedMem(fname, vars)
            % Allow relative or absolute path; writer may delete old file
            if nargin == 2 && isfile(fname)
                delete(fname);
            end
            if nargin == 2                    % === Writer ===
                header  = SharedMem.makeHeader(vars);
                obj.hdrLen = double(numel(header));
                [obj.varInfo,obj.oneBufBytes] = SharedMem.buildInfo(vars);
                obj.buf0Start = 6 + obj.hdrLen;        % [busy buf hdrLen hdr]
                totalBytes = obj.buf0Start + 2*obj.oneBufBytes;
                SharedMem.createFile(fname, header, totalBytes);
            else                               % === Reader ===
                SharedMem.waitFile(fname);              % wait until writer ready
                [header,obj.hdrLen] = SharedMem.readHeader(fname);
                vars = SharedMem.json2vars(header);
                [obj.varInfo,obj.oneBufBytes] = SharedMem.buildInfo(vars);
                obj.buf0Start = 6 + obj.hdrLen;
                totalBytes = obj.buf0Start + 2*obj.oneBufBytes;
            end

            % Map entire file (1×totalBytes uint8 vector)
            fmt = {'uint8',[1 totalBytes],'data'};
            writableFlag = true;
            obj.mm = memmapfile(fname,'Writable',writableFlag,'Format',fmt);
        end

        %========================== WRITE ============================
        function write(obj,data)
            while obj.mm.Data(1).data(1) ==1, end      % uint8 row vector
            obj.mm.Data(1).data(1) = 1;

            tgt = 1 - obj.mm.Data(1).data(2);
            base = double(obj.buf0Start) + double(tgt)*obj.oneBufBytes;

            for k = 1:numel(obj.varInfo)
                vi  = obj.varInfo(k);
                val = data.(vi.name);

                % Bounds check
                if any(size(val) > vi.maxSz)
                    error("SharedMem:SizeExceeded", ...
                        "%s exceeds maxSize %s", vi.name, mat2str(vi.maxSz));
                end

                % ----- write size meta -----
                idxMeta = double(base) + double(vi.metaOff) + (1:double(vi.metaBytes));
                obj.mm.Data(1).data(idxMeta) = typecast(uint32(size(val)),'uint8');

                % ----- write data body -----
                raw = typecast(cast(val(:),vi.type),'uint8');
                idxDat = double(base) + double(vi.dataOff) + (1:double(numel(raw)));
                obj.mm.Data(1).data(idxDat) = raw;
            end

            obj.mm.Data(1).data(2) = tgt;   % buf
            obj.mm.Data(1).data(1) = 0;     % unlock
        end

        %========================== READ =============================
        function [out,ok] = read(obj,skipSame)
            if nargin < 2, skipSame = false; end
            persistent lastBuf
            out = []; ok = false;

            if obj.mm.Data(1).data(1)==1, return; end    % writer busy
            buf = obj.mm.Data(1).data(2);

            if skipSame && isequal(buf,lastBuf), return; end
            base = double(obj.buf0Start) + double(buf)*obj.oneBufBytes;

            for k = 1:numel(obj.varInfo)
                vi = obj.varInfo(k);
                idxMeta = double(base) + double(vi.metaOff) + (1:double(vi.metaBytes));
                sz      = typecast(obj.mm.Data(1).data(idxMeta),'uint32');
                if any(sz==0), return; end                % still empty

                ne  = prod(double(sz));
                idxRaw = base + vi.dataOff + (1:ne*vi.bpe);
                raw = obj.mm.Data(1).data(idxRaw);
                out.(vi.name) = reshape(typecast(raw,vi.type),sz);
            end
            ok = true;
            lastBuf = buf;
        end
    end

    %% ────────────────────── INTERNAL HELPERS ──────────────────────
    methods (Static, Access = private)

        %------------- make JSON header -----------------------------
        function bytes = makeHeader(vars)
            for i = 1:size(vars,1)
                s(i) = struct('name',vars{i,1}, ...
                              'type',vars{i,2}, ...
                              'maxSize',vars{i,3}); %#ok<AGROW>
            end
            bytes = uint8(jsonencode(s));
        end

        %------------- parse JSON header ----------------------------
        function vars = json2vars(hdrBytes)
            txt = char(hdrBytes(:).');
            s = jsondecode(txt);
            vars = cell(numel(s),3);
            for i = 1:numel(s)
                vars{i,1} = s(i).name;
                vars{i,2} = s(i).type;
                vars{i,3} = s(i).maxSize;
            end
        end

        %------------- build varInfo & buffer size ------------------
        function [info,one] = buildInfo(vars)
            offset = 0;
            for i = 1:size(vars,1)
                tp   = vars{i,2};
                sz   = vars{i,3};
                bpe  = double(SharedMem.typeBytes.(tp));
                mb   = double(numel(sz)*4);
                db   = double(prod(sz)*bpe);

                info(i) = struct( ...
                    'name',      vars{i,1}, ...
                    'type',      tp, ...
                    'bpe',       bpe, ...
                    'maxSz',     sz, ...
                    'metaOff',   double(offset), ...
                    'metaBytes', mb, ...
                    'dataOff',   double(offset)+mb, ...
                    'totalBytes',mb+db); %#ok<AGROW>

                offset = offset + mb + db;
            end
            one = double(offset);
        end

        %------------- create & zero-fill file ----------------------
        function createFile(fname,hdr,totalBytes)
            hdrLen = uint32(numel(hdr));
            f = fopen(fname,'w+b');
            fwrite(f, zeros(1,totalBytes,'uint8'),'uint8');  % allocate
            fseek(f,0,'bof');
            fwrite(f,uint8([0 0]),'uint8');  % busy, buf
            fwrite(f,hdrLen,'uint32');
            fwrite(f,hdr,'uint8');
            fclose(f);
        end

        %------------- read header ---------------------------------
        function [hdr,hLen] = readHeader(fname)
            f = fopen(fname,'r');
            fseek(f,2,'bof');
            hLen = fread(f,1,'uint32');
            hdr  = fread(f,hLen,'uint8');
            fclose(f);
        end

        %------------- wait until writer created file --------------
        function waitFile(fname)
            while ~isfile(fname), pause(0.01); end
            while true
                f = fopen(fname,'r');
                b = fread(f,2,'uint8=>uint8');
                ok = ~isempty(b) && b(1)==0;
                hLen = fread(f,1,'uint32');
                fclose(f);
                if ok && hLen>0, break; end
                pause(0.01);
            end
        end
    end
end
