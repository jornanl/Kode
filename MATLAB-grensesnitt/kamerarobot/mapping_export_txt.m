function mapping_export_txt(matrix)

filename = strcat(matrix, '.txt');
dlmwrite('filename.txt',matrix,'delimiter', ';', 'newline', 'pc');

end