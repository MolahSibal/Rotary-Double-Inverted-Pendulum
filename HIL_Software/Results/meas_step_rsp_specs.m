%
%% MEAS_STEP_RSP_SPECS
%
% Measures the peak time, settling time, steady-state error, and percentage
% overshoot of a given response.
%
% ************************************************************************
% Input parameters:
% t         time                                            (s)
% yd        response setpoint
% y         response
% c_ts      settling time percentage
% tol_p     tolerance to find peak of response wo/ overshoot
%
% ************************************************************************
% Output parameters:
% tp        Peak time                                       (s)
% ts        Settling time                                   (s)
% e_ss      Steady-state error
% PO        Percentage overshoot                            (%)
%
% Copyright (C) 2007 Quanser Consulting Inc.
% Quanser Consulting Inc.
%
%
%%
function [ tp, ts, e_ss, PO ] = meas_step_rsp_specs( t, yd, y, c_ts, tol_p )
%
%% Measure Steady-State Error 
% Init index
i = 1;
% Number of elements in data
N = length(yd);
% Variable indicating when to stop while loop
stop_loop = 0;
%
while (i < N) && (stop_loop < 2)
    yd_last = yd(i);
    y_last = y(i);
    i = i + 1;
    % Reference step start time
    if ( yd(i) > yd_last ) && (stop_loop == 0)        
        stop_loop = 1;
        y_ss_last = y_last;
    end    
    % Reference step stop time
     if (yd(i) < yd_last ) && (stop_loop == 1)
         % Steady-state response
         y_ss = y_last;
         % Measured steady-state error before next cycle
         e_ss = yd_last - y_ss;
         % Stop the loop
         stop_loop = 2;
     end
end
%
%% Measure Peak Time, Settling Time, and Overshoot
% Reset index
i = 1;
% Reset variable indicating when to stop while loop
stop_loop = 0;
% Setpoint peak (cm)
yd_p = max(yd);
% Response peak (cm)
yp = max(y);
% Set flag to indicate if there is no overshoot
if ( yp <= yd_p )
    ov = 0;
else
    ov = 1;
end

% Settling value (cm)
if ( ov == 1 )
    yd_s = ( 1 + c_ts ) * y_ss;
else
    yd_s = ( 1 - c_ts ) * y_ss;
end
%
while (i < N) && (stop_loop < 3)
    yd_last = yd(i);
    i = i + 1;
    % Reference step start time
    if ( yd(i) > yd_last ) && (stop_loop == 0)        
        t0 = t(i);
        stop_loop = 1;        
    end
    % Peak and settling time calculation
    if (ov == 1) % rsp w/ overshoot
        % Peak value and time (rad, s)    
        if (y(i) > yd_p ) && ( y(i) >= abs(yp)-tol_p ) && (stop_loop == 1)
            t1 = t(i);
            stop_loop = 2;
        end
        % After finding peak, find settling value and time (s)
        if ((y(i) <= yd_s) && (stop_loop == 2))
            t2 = t(i);
            stop_loop = 3;
        end
    else % rsp wo/ overshoot
        % After finding peak, find settling value and time (s)
        if ((y(i) >= yd_s) && (stop_loop == 1))
            t2 = t(i);
            stop_loop = 2;
        end
        % No overshoot, so find time of first peak (s)    
        if ( y(i) >= abs(yp)-tol_p ) && (stop_loop == 2)
            t1 = t(i);
            stop_loop = 3;
        end
    end
 end
% Measured peak time (s)
tp = t1 - t0;
% Amplitude of response
dy = y_ss-y_ss_last;
% Percentage overshoot (%)
PO = 100 * (yp - y_ss) / dy;
% Measured settling time (s)
ts = t2 - t0;