function q = readMOT(fname)
% q = readMOT(fname)
% Input:    fname is the name of the ascii datafile to be read 
% Output:   q returns a structure with the following format:
%				q.labels 	= array of column labels
%				q.data 		= matrix of data
%				q.nr 		= number of matrix rows
%				q.nc 		= number of matrix columns

% Open ascii data file for reading.
fid = fopen(fname, 'r');	
if fid == -1								
	error(['unable to open ', fname])		
end
% Process the file header;
% store # data rows, # data columns.
q.nr = 0; % Added to ensure that the q structures from reading a motion file
q.nc = 0; % are always the same, even if nr and nc are different orders in file.
nextline = fgetl(fid);	
while ~strncmpi(nextline, 'endheader', length('endheader'))
	if strncmpi(nextline, 'datacolumns', length('datacolumns'))
		q.nc = str2num(nextline(findstr(nextline, ' ')+1 : length(nextline)));
	elseif strncmpi(nextline, 'datarows', length('datarows'))
		q.nr = str2num(nextline(findstr(nextline, ' ')+1 : length(nextline)));
	elseif strncmpi(nextline, 'nColumns', length('nColumns'))
		q.nc = str2num(nextline(findstr(nextline, '=')+1 : length(nextline)));
	elseif strncmpi(nextline, 'nRows', length('nRows'))
		q.nr = str2num(nextline(findstr(nextline, '=')+1 : length(nextline)));
    elseif strncmpi(nextline, 'inDegrees', length('inDegrees'))
		q.inDeg = nextline(findstr(nextline, '=')+1 : length(nextline));
	end
	nextline = fgetl(fid);
end
% Process the column labels.
nextline = fgetl(fid);
if (all(isspace(nextline))) % Blank line, so the next one must be the one containing the column labels
	nextline = fgetl(fid);
end
a=textscan(nextline,'%s','MultipleDelimsAsOne',1,'Delimiter',sprintf('\t'));
q.labels = [a{:}]';
% Process the data.
% Note:  transpose is needed since fscanf fills columns before rows.
% Text = fscanf(fid, '%c');fclose(fid);
Data = fscanf(fid, '%f', [q.nc, q.nr])';fclose(fid);
% Data = textscan(Text, '%f');
% length(Data{1})/q.nc
% reshape(Data{1},[q.nc, q.nr])
if any(size(Data)~=[q.nr, q.nc]) || (q.nr==0 && q.nc==1)
    fid = fopen(fname, 'r');
    Text = fscanf(fid,'%c');fclose(fid);
    if any(strfind(Text,'1.#QNAN0000000'))
        Text=strrep(Text,'1.#QNAN0000000','0.000000000000');
    end
    if any(strfind(Text,'-1.#IND00000000'))
        Text=strrep(Text,'-1.#IND00000000','0.000000000000');
    end
    if any(strfind(Text,'-1.#INF00000000'))
        Text=strrep(Text,'-1.#INF00000000','0.000000000000');
    end
    if any(strfind(Text,'242373565591244840000000000000000000000000000000000000000000000000000000000000000.000000000000'))
        Text=strrep(Text,'242373565591244840000000000000000000000000000000000000000000000000000000000000000.000000000000','0.000000000000');
    end    
    
    fid = fopen(fname, 'w');
    fprintf(fid,'%s',Text);fclose(fid);
    DataStart = strfind(Text,q.labels{end});
    DataStart=DataStart-1+min(strfind(Text(DataStart:end),char(13)))+1;
    DataText = Text(DataStart:end);
    LettersFound = intersect(regexp(DataText,'\w'),regexp(DataText,'\D'));
    if any(LettersFound)
        warning('letters found in data :')
        DataText(LettersFound)
    end
    DataText = strrep(DataText,DataText(LettersFound),'');
    Data = textscan(DataText, '%f');
    Data = Data{1};
    if length(Data) == q.nc*q.nr
        Data = reshape(Data,q.nc,q.nr)';
    else
        warning('not correct size of data specified in header, using size of Data vector and length of column labels')
        q.nc = length(q.labels);
        q.nr = length(Data)./q.nc;
        Data = reshape(Data,q.nc,q.nr)';
    end
    if any(size(Data)~=[q.nr q.nc])
        warning('File read error still'),beep
        keyboard
    end
end
q.data = Data;