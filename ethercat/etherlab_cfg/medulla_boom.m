% This returns the EtherCAT Slave Configuration struct the EtherLab custom slave Simulink block expects.
% Documentation for this structure can be found by looking at the help for EtherLab's Generic Slave Simulink block.
% This structure contains much of the same data as the ENI file (the XML file) used by Beckhoff's tools.

function slave = medulla_boom()
	% Basic slave information
	slave.SlaveConfig.vendor      = 1551;           % This is our Vendor ID
	slave.SlaveConfig.product     = hex2dec('3');   % Product 3 is the Boom Medulla.
	slave.SlaveConfig.description = 'Boom Medulla'; % This is optional, but it might be nice.

	% SyncManager configuration. We use a fairly standard EtherCAT setup: Syncmanagers 0 and 1 are
	% disabled; 2 and 3 are enabled. SM 2 has outputs (RxPDOs) and SM 3 has inputs (TxPDOs).

	% Our outputs
	Entries_RxPDO1 = [ hex2dec('5') 1  8    % Command
	                   hex2dec('6') 1 16 ]; % Counter
	RxPDO1 = {hex2dec('1600') Entries_RxPDO1};

	% and inputs (We have several PDOs here...)
	Entries_TxPDO1 = [ hex2dec('5') 2  8    % ID
	                   hex2dec('5') 3  8    % State
	                   hex2dec('5') 4  8    % Counter
	                   hex2dec('5') 5  8 ]; % Error Flags
	Entries_TxPDO2 = [ hex2dec('7') 1 32    % X Encoder
	                   hex2dec('6') 2 16 ]; % X Encoder timestamp
	Entries_TxPDO3 = [ hex2dec('7') 1 32    % Pitch Encoder
	                   hex2dec('6') 2 16 ]; % Pitch Encoder timestamp
	Entries_TxPDO4 = [ hex2dec('7') 1 32    % Z Encoder
	                   hex2dec('6') 2 16 ]; % Z Encoder timestamp
	Entries_TxPDO5 = [ hex2dec('6') 2 16 ]; % Logic Voltage
	TxPDO1 = {hex2dec('1A0F') Entries_TxPDO1};
	TxPDO2 = {hex2dec('1A10') Entries_TxPDO2};
	TxPDO3 = {hex2dec('1A11') Entries_TxPDO3};
	TxPDO4 = {hex2dec('1A12') Entries_TxPDO4};
	TxPDO5 = {hex2dec('1A13') Entries_TxPDO5};

	% Package things together
	Sm2PDO = {RxPDO1};
	Sm3PDO = {TxPDO1, TxPDO2, TxPDO3, TxPDO4, TxPDO5};
	Sm2 = {2, 0, Sm2PDO}; % SyncManager 2; Direction = 0 (output)
	Sm3 = {3, 1, Sm3PDO}; % SyncManager 3; Direction = 1 (input)

	% Place the SM config into SlaveConfig
	slave.SlaveConfig.sm = {Sm2, Sm3};

	% Distributed Clock (DC for short) configuration. We use a cycle time
	% of 1000000 nanoseconds, with a shift of -300000 nanoseconds
	%slave.SlaveConfig.dc = hex2dec('300');
	slave.SlaveConfig.dc = [hex2dec('300'), 1000000, 0, -300000, 0, 0, 0, 0, 0, 0];

	% These are data types for ports on the Simulink block
	UINT8  = 1008;
	UINT16 = 1016;
	UINT32 = 1032;

	% PortConfig configures the ports on the Simulink block
	% pdo setup: [ SyncManager PDOIndex EntryIndex ElementEndex]
	% pdo is 0-indexed.
	% We don't need to set input data types here because we're just forwarding
	% the PDOs directly through. Output data types are still necessary, however
	% Note: output refers to the block's outputs, which are TxPDOs! (Inputs in
	% EtherCAT jargon!)
	slave.PortConfig.input(1).pdo             = [ 0 0 0 0 ]; % Command
	slave.PortConfig.input(1).pdo_data_type   = UINT8;
	slave.PortConfig.input(2).pdo             = [ 0 0 1 0 ]; % Counter
	slave.PortConfig.input(2).pdo_data_type   = UINT16;
	slave.PortConfig.output(1).pdo            = [ 1 0 0 0 ]; % ID
	slave.PortConfig.output(1).pdo_data_type  = UINT8;
	slave.PortConfig.output(2).pdo            = [ 1 0 1 0 ]; % State
	slave.PortConfig.output(2).pdo_data_type  = UINT8;
	slave.PortConfig.output(3).pdo            = [ 1 0 2 0 ]; % Counter
	slave.PortConfig.output(3).pdo_data_type  = UINT8;
	slave.PortConfig.output(4).pdo            = [ 1 0 3 0 ]; % Error Flags
	slave.PortConfig.output(4).pdo_data_type  = UINT8;
	slave.PortConfig.output(5).pdo            = [ 1 1 0 0 ]; % X Encoder
	slave.PortConfig.output(5).pdo_data_type  = UINT32;
	slave.PortConfig.output(6).pdo            = [ 1 1 1 0 ]; % X Encoder timestamp
	slave.PortConfig.output(6).pdo_data_type  = UINT16;
	slave.PortConfig.output(7).pdo            = [ 1 2 0 0 ]; % Pitch Encoder
	slave.PortConfig.output(7).pdo_data_type  = UINT32;
	slave.PortConfig.output(8).pdo            = [ 1 2 1 0 ]; % Pitch Encoder timestamp
	slave.PortConfig.output(8).pdo_data_type  = UINT16;
	slave.PortConfig.output(9).pdo            = [ 1 3 0 0 ]; % Z Encoder
	slave.PortConfig.output(9).pdo_data_type  = UINT32;
	slave.PortConfig.output(10).pdo           = [ 1 3 1 0 ]; % Z Encoder timestamp
	slave.PortConfig.output(10).pdo_data_type = UINT16;
	slave.PortConfig.output(11).pdo           = [ 1 4 0 0 ]; % Logic Voltage
	slave.PortConfig.output(11).pdo_data_type = UINT16;
end
