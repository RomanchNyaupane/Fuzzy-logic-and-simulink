clc
col_header = {'time','angle','voltage'};
xlswrite('logged_data',col_header,'Sheet1','A1');
xlswrite('logged_data',out.data,'Sheet1','A2');