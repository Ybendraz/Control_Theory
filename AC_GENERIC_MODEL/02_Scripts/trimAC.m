function [LinAC,Xeq,Ueq] = trimAC(SYS,X,U)
% TRIMAC finds the trim point Xeq & Ueq based on X & U
% and linearizes the model SYS around it.
[Xeq,Ueq,~] = trim(SYS,X,U);
LinAC= linmod(SYS,Xeq,Ueq);
end

