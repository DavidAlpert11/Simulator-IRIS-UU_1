function [ RotateMatrix ] = Euler2Dcm( phi,theta,psi )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%   Inputs validation
validateattributes(phi,{'numeric'},{'>=',-pi,'<=',pi});
validateattributes(theta,{'numeric'},{'>=',-pi/2,'<=',pi/2});
validateattributes(psi,{'numeric'},{'>=',-pi,'<=',pi});

%   Matrix calculation
phiMatrix = [   1,  0,          0           ;
                0,  cos(phi),   -sin(phi)   ;
                0,  sin(phi),   cos(phi)    ];

thetaMatrix =   [   cos(theta),  0,  sin(theta) ;
                    0,           1,  0          ;
                    -sin(theta), 0, cos(theta)  ];
           
psiMatrix = [   cos(psi),  -sin(psi),  0   ;
                sin(psi),  cos(psi),   0   ;
                0,         0,          1   ];

RotateMatrix = psiMatrix*thetaMatrix*phiMatrix;

end