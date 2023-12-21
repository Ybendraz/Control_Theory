function [fval] = RCAM_model_implicit(Xdot,X,U)
%RCAM_MODEL_IMPLICIT Summary of this function goes here
    fval = RCAM_model(X,U) - Xdot;
end

