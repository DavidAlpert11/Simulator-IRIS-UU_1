function [uniqueIndexes] = extractUniquePOI(vertex)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Initialize an empty cell array to store the unique indexes
allIndexes = {};

% Iterate over each row in vertex
for i = 1:size(vertex, 1)
    % Extract the string from the cell
    vertexString = cell2mat(vertex{i, 1});

    % Split the string into individual numbers
    vertexNumbers = str2double(strsplit(vertexString));

    % Extract indexes starting from the 4th element
    indexes = vertexNumbers(4:end);

    % Append the indexes to the cell array
    allIndexes = [allIndexes, indexes];
end

% Convert cell array to a single numeric array
allIndexes = cell2mat(allIndexes);

% Extract unique indexes
uniqueIndexes = unique(allIndexes);



end