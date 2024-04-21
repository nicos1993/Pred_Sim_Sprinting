function write_storageFile(q, fname)

fid = fopen(fname, 'w');
if fid == -1
    error(['unable to open ', fname])
end

if length(q.labels) ~= size(q.data,2)
	error('Number of labels doesn''t match number of columns')
end

if q.labels{1} ~= 'time'
	error('Expected ''time'' as first column')
end

fprintf(fid, 'name %s\n', fname);
fprintf(fid, 'version=1\n');
fprintf(fid, 'nRows=%d\n', size(q.data,1));
fprintf(fid, 'nColumns=%d\n', size(q.data,2));
fprintf(fid, 'inDegrees=no\n');
fprintf(fid, 'endheader\n');

for i=1:length(q.labels)
	fprintf(fid, '%20s\t', q.labels{i});
end
fprintf(fid, '\n');

for i=1:size(q.data,1)
	fprintf(fid, '%20.8f\t', q.data(i,:));
	fprintf(fid, '\n');
end

fclose(fid);
return;