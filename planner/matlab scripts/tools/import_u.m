function u = import_u(filename, dataLines)
%IMPORTFILE Import data from a text file
%  U = IMPORTFILE(FILENAME) reads data from text file FILENAME for the
%  default selection.  Returns the numeric data.
%
%  U = IMPORTFILE(FILE, DATALINES) reads data for the specified row
%  interval(s) of text file FILENAME. Specify DATALINES as a positive
%  scalar integer or a N-by-2 array of positive scalar integers for
%  dis-contiguous row intervals.
%
%  Example:
%  u = importfile("/home/marc/git_personal/colab_mpc/ColaborativeMPC-/experiments/test-bench/catkin_mrs/src/colab_mpc/src/NonLinearControllerObject/dist/1/u.dat", [1, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 13-Jun-2023 12:50:40

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [1, Inf];
end

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 2);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = " ";

% Specify column names and types
opts.VariableNames = ["e02", "e00"];
opts.VariableTypes = ["double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
opts.ConsecutiveDelimitersRule = "join";
opts.LeadingDelimitersRule = "ignore";

% Import the data
u = readtable(filename, opts);

%% Convert to output type
u = table2array(u);
end