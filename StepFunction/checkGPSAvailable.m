function isGPSAvailable = checkGPSAvailable(State)

X = [-100,100];
Y = [-22,2];
Z = [-20,0];
X = [50,100];
Y = [-11,1];
Z = [-20,0];
% X = [-25,25];
% Y = [-1,1];
% Z = [-20,20];
X = [-100,100];
Y = [-11,1];
Z = [-20,0];

if (State.X(1) >= X(1) && State.X(1) <= X(2) &&...
        State.X(2) >= Y(1) && State.X(2) <= Y(2) &&...
        State.X(3) >= Z(1) && State.X(3) <= Z(2))
    isGPSAvailable = 0;
else
    isGPSAvailable = 1;
end


end