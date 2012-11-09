function COM_MakeBTConfigFile()
% Creates a Bluetooth configuration file which is needed to open a Bluetooth connection.
%
% Syntax
%   COM_MakeBTConfigFile()
%
% Description
%   COM_MakeBTConfigFile() starts a user guided dialog window to select the output directory,
%   the file name and the Bluetooth paramters like e.g. COM port.
%
%   The little program creates a specific Bluetooth configuration file for the current PC system.
%   Make sure the configuration file is accessible under MATLAB if you try to open a Bluetooth
%   connection using COM_OpenNXT and the correct file name.
%
% Example
%   COM_MakeBTConfigFile();
%
%   handle = COM_OpenNXT('bluetooth.ini', 'check');
%
% See also: COM_OpenNXT, COM_CloseNXT, COM_OpenNXTEx
%
% Signature
%   Author: Alexander Behrens, Linus Atorf (see AUTHORS)
%   Date: 2008/07/09
%   Copyright: 2007-2008, RWTH Aachen University
%
%
% ***********************************************************************************************
% *  This file is part of the RWTH - Mindstorms NXT Toolbox.                                    *
% *                                                                                             *
% *  The RWTH - Mindstorms NXT Toolbox is free software: you can redistribute it and/or modify  *
% *  it under the terms of the GNU General Public License as published by the Free Software     *
% *  Foundation, either version 3 of the License, or (at your option) any later version.        *
% *                                                                                             *
% *  The RWTH - Mindstorms NXT Toolbox is distributed in the hope that it will be useful,       *
% *  but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS  *
% *  FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.             *
% *                                                                                             *
% *  You should have received a copy of the GNU General Public License along with the           *
% *  RWTH - Mindstorms NXT Toolbox. If not, see <http://www.gnu.org/licenses/>.                 *
% ***********************************************************************************************

dialog_title = 'Creating Bluetooth Configuration File';

%% Question Dialog
my_answer = questdlg('Do you want to create a new Bluetooth configuration file?', dialog_title);


if strcmpi(my_answer,'YES')
%% Set Directory Dialog
    my_dir_name = uigetdir(pwd,'Select a directory where to save the Bluetooth configuration file. (Should be the application directory!)');


%% Set Parameter Dialog
    if my_dir_name ~= 0
        dlg_title = 'Configuration File Settings';
        prompt    = {'Filename:', 'SerialPort:', 'BaudRate:', 'DataBits:', 'SendPause:', 'ReceivePause:', 'Timeout:'};
        num_lines = 1;
        options.Resize='on';
        options.WindowStyle='normal';
        
        default_parameters = {'bluetooth.ini','COM3','9600', '8', '5', '30', '2'};

        my_parameters  = inputdlg(prompt, dlg_title, num_lines, default_parameters, options);
        
        if ~isempty(my_parameters)
            filename     = my_parameters{1};
            port         = my_parameters{2};
            baudrate     = str2double(my_parameters{3});
            databits     = str2double(my_parameters{4});
            sendpause    = str2double(my_parameters{5});
            receivepause = str2double(my_parameters{6});
            timeout      = str2double(my_parameters{7});


%% Write Configuration File

            % maybe consider using fullfile() instead of if ispc etc?
            if ispc
                h_file = fopen(sprintf('%s\\%s', my_dir_name, filename), 'w');
            else
                h_file = fopen(sprintf('%s/%s', my_dir_name, filename), 'w');
            end

            % Use CRLF (\r\n) on Windows system as linebreak for
            % compatibility with notepad (standard application for ini
            % files). Linux tools do not care about \r\n, so that's fine.
            
            fwrite(h_file, sprintf('[Bluetooth]\r\n'));
            fwrite(h_file, sprintf('\r\n'));

            fwrite(h_file, sprintf('SerialPort=%s\r\n', port));
            fwrite(h_file, sprintf('BaudRate=%d\r\n',   baudrate));
            fwrite(h_file, sprintf('DataBits=%d\r\n',   databits));
            fwrite(h_file, sprintf('\r\n'));

            fwrite(h_file, sprintf('SendSendPause=%d\r\n',    sendpause));
            fwrite(h_file, sprintf('SendReceivePause=%d\r\n', receivepause));
            fwrite(h_file, sprintf('\r\n'));
            
            fwrite(h_file, sprintf('Timeout=%d\r\n', timeout));
            fwrite(h_file, sprintf('\r\n'));

            fclose(h_file);
            
            %TODO add dialog 'file successfully written'?
            
        else % if ~isempty(my_parameters)
             msgbox('No configuration parameters are set!', dialog_title, 'error');
        end 
    else % if my_dir_name ~= 0
        msgbox('No directory is selected!', dialog_title, 'error');
    end 
else % if strcmpi(my_answer,'YES')
    msgbox('User aborted!', dialog_title, 'error');
end  

end % end function
