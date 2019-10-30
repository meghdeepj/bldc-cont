%this is an S function that calculates the current profile that is required
%
function [sys,x0,str,ts] = control1(t,x,u,flag,J,DF,N,BM,Rr,Rl,delta,KP,KI,KD,basecurr)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%               THE INPUTS EXPECTED ARE:

%(1):    wn:        THE VALUE OF ROTOR ELECTRICAL SPEED
%(2):    THETA:     THE VALUE OF ROTOR ELECTRICAL ANGLE(NORMALISED TO 2*PI)
%(3):    wreq:      THE DESIRED ROTOR ELECTRICAL SPEED
%(4):    ERR_1:     THE VALUE OF THE ERROR AT THE PREVIOUS TIME STEP


%               THE OUTPUTS ARE:
%(1):   IUstar:     THE COMMAND PHASE U CURRENT
%(2):   IVstar:     THE COMMAND PHASE V CURRENT
%(3):   IWstar:     THE COMMAND PHASE W CURRENT
%(4):   err:        THE VALUE OF THE ERROR AT CURRENT TIME STEP
%(5):   Treq:       THE REQUIRED VALUE OF TORQUE TO FULFIL THE SPEED
%COMMAND

%                       

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%   Copyright 2003-2004, Devendra Rai,NITK.
%   $Revision: xx

%
% The following outlines the general structure of an S-function.
%
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(J,DF,N,BM,Rr,Rl,delta,KP,KI,KD,basecurr);
    

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,J,DF,N,BM,Rr,Rl,delta,KP,KI,KD,basecurr);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,J,DF,N,BM,Rr,Rl,delta,KP,KI,KD,basecurr);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(J,DF,N,BM,Rr,Rl,delta,KP,KI,KD,basecurr)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 5;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 4;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%

x0  =[]; 

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [4e-6 0];
%   SAMPLING FREQUENCY 250KHz; OFFSET =0



% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u,J,DF,N,BM,Rr,Rl,delta,KP,KI,KD,basecurr)

sys = [];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,J,DF,N,BM,Rr,Rl,delta,KP,KI,KD,basecurr)

%=============================================================================
% CALCULATE THE TORQUE REQUIRED
sys(4)= u(3)-u(1); 
Treq= sys(4)*(KP + KI*0.5*4e-6 + KD/4e-6 ) + u(4)*(KI*0.5*4e-6 - KD/4e-6);     %calculate the requisite torque
%Treq= sys(4)*KP;
if (abs(Treq) > 100)
    sys(5)=sign(Treq)*100;                                                      %saturate the torque estimate
else
    sys(5)=Treq;
end;

Ireq= sys(5)/(N*BM*Rl*Rr);                                                      %calculate the requires current command
 
 %CALCULATE THE DIRECT AND QUADRATURE COMPONENTS OF THE CURRENT COMMAND
 
 I_QD=Ireq*[sin(delta);cos(delta)];             %I_QD is a column vector that contains the required components
 %----------------------------------------------------------------------------
 %I_T: The torque producing component of stator current is equal to I_QD(2,1)
 %If:  The flux producing component of the stator current is equal to I_QD(1,1)
 %THE TORQUE ANGLE IS GIVEN BY DELTA.
 %----------------------------------------------------------------------------
 %LOOK UP TABLE FOR A MODIFIED PARK'S TRANSFORM
 theta = u(2);                                 
if  (theta>=0 & theta<pi/6)             % 0 TO 30 DEGREES
    
    SINTHETA= 6*theta/pi;
    SINTHETARS=-1;
    SINTHETALS=1;
   
    COSTHETA=1;
    COSTHETARS=(6*theta/pi)-1;
    COSTHETALS=-1;
    
    
end;

    
if (theta>=pi/6 & theta<pi/3)           %30 TO 60 DEGREES
    
    SINTHETA=1;
    SINTHETARS=-1;
    SINTHETALS=-6*(theta-(2*pi/6))/pi;
    
    COSTHETA=1;
    COSTHETARS=(6*theta/pi)-1;
    COSTHETALS=-1;
    
end;

if (theta>=pi/3 & theta<pi/2)           %60 TO 90 DEGREES
    
    SINTHETA=1;
    SINTHETARS=-1;
    SINTHETALS=-6*(theta-(2*pi/6))/pi;
    
    COSTHETA=-(6*theta/pi)+3;
    COSTHETARS=1;
    COSTHETALS=-1;
end;

if (theta>=pi/2 & theta<2*pi/3)         %90 TO 120 DEGREES
    
    SINTHETA=1;
    SINTHETARS=(  theta-(4*pi/6) )*6/pi;
    SINTHETALS=-1;
    
    COSTHETA=-(6*theta/pi)+3;
    COSTHETARS=1;
    COSTHETALS=-1;
end;

if(theta>=2*pi/3 & theta<5*pi/6)        %120 TO 150 DEGREES
    
    SINTHETA=1;
    SINTHETARS=(  theta-(2*pi/3) )*6/pi;
    SINTHETALS=-1;
    
    COSTHETA=-1;
    COSTHETARS=1;
    COSTHETALS=(6*theta/pi)-5;
    
end;

if (theta>=5*pi/6 & theta<pi)           %150 TO 180 DEGREES
    
    SINTHETA=(pi-theta)*6/pi;
    SINTHETARS=1;
    SINTHETALS=-1;
    
    COSTHETA=-1;
    COSTHETARS=1;
    COSTHETALS=(6*theta/pi)-5;
end;

if(theta>=pi & theta<7*pi/6)            %180 TO 210 DEGREES
    
    SINTHETA=(pi-theta)*6/pi;
    SINTHETARS=1;
    SINTHETALS=-1;
    
    COSTHETA=-1;
    COSTHETARS=-(6*theta/pi)+7;
    COSTHETALS=1;
end;

if(theta>=7*pi/6 & theta<4*pi/3)        %210 TO 240 DEGREES
    SINTHETA= -1;
    SINTHETARS=1;
    SINTHETALS=( theta-(4*pi/3))*6/pi;
    
    COSTHETA=-1;
    COSTHETARS=-(6*theta/pi)+7;
    COSTHETALS=1;
end;

if (theta>=4*pi/3 & theta<3*pi/2)       %240 TO 270 DEGREES
   
   SINTHETA= -1;
   SINTHETARS=1;
   SINTHETALS=( theta -(4*pi/3))*6/pi; 
   
   COSTHETA=(6*theta/pi)-9;
   COSTHETARS=-1;
   COSTHETALS=1;
   
end;

if(theta>=3*pi/2 & theta<5*pi/3)        %270 TO 300 DEGREES
   
    SINTHETA= -1;
    SINTHETARS=((5*pi/3)-theta)*6/pi;
    SINTHETALS=1;
    
    COSTHETA=(6*theta/pi)-9;
    COSTHETARS=-1;
    COSTHETALS=1;
end;

if(theta>=5*pi/3 & theta<11*pi/6)       %300 TO 330 DEGREES
    
    SINTHETA= -1;
    SINTHETARS=((5*pi/3)-theta)*6/pi;
    SINTHETALS=1;    
    
    COSTHETA=1;
    COSTHETARS=-1;
    COSTHETALS=-(6*theta/pi)+11;
    
end;

if(theta>=11*pi/6 & theta<2*pi)         %330 TO 360 DEGREES
    
    SINTHETA= 6*(theta-2*pi)/pi;
    SINTHETARS=-1;
    SINTHETALS=1;
    
    COSTHETA=1;
    COSTHETARS=-1;
    COSTHETALS=-(6*theta/pi)+11;
end;
%%%-------------CALCULATION FOR SYS(4)<0---------------------------------------------------
if  (theta>=-2*pi  & theta<-11*pi/6)    %-360 TO -330 DEGREES
    
    SINTHETA= (theta+2*pi)*6/pi;
    SINTHETARS=-1;
    SINTHETALS=1;
    
    COSTHETA=1;
    COSTHETARS=(6*theta/pi)+11;
    COSTHETALS=-1;
    
    
end;

    
if (theta>=-11*pi/6 & theta<-5*pi/3)    %-330 TO -300 DEGREES
    
    SINTHETA=1;
    SINTHETARS=-1;
    SINTHETALS=-(theta+(10*pi/6))*6/pi;
    
    COSTHETA=1;
    COSTHETARS=(6*theta/pi)+11;
    COSTHETALS=-1;
end;

if (theta>=-5*pi/3 & theta<-3*pi/2)     %-300 TO -270 DEGREES
    
    SINTHETA=1;
    SINTHETARS=-1;
    SINTHETALS=-(theta+(10*pi/6))*6/pi;
    
    COSTHETA=-(6*theta/pi)-9;
    COSTHETARS=1;
    COSTHETALS=-1;
end;

if (theta>=-3*pi/2 & theta<-4*pi/3)     %-270 TO -240 DEGREES

    SINTHETA=1;
    SINTHETARS=(  theta+(4*pi/3) )*6/pi;
    SINTHETALS=-1;
    
    COSTHETA=-(6*theta/pi)-9;
    COSTHETARS=1;
    COSTHETALS=-1;
end;

if(theta>=-4*pi/3 & theta<-7*pi/6)      %-240 TO -210 DEGREES

    SINTHETA=1;
    SINTHETARS=(  theta+(4*pi/3) )*6/pi;
    SINTHETALS=-1;
 
    COSTHETA=-1;
    COSTHETARS=1;
    COSTHETALS=(6*theta/pi)+7;
end;

if (theta>=-7*pi/6 & theta<-pi)         %-210 TO -180 DEGREES

    SINTHETA=-(theta+pi)*6/pi;
    SINTHETARS=1;
    SINTHETALS=-1;
    
    COSTHETA=-1;
    COSTHETARS=1;
    COSTHETALS=(6*theta/pi)+7;
end;

if(theta>=-pi & theta<-5*pi/6)          %-180 TO -150 DEGREES

    SINTHETA=-(theta+pi)*6/pi;
    SINTHETARS=1;
    SINTHETALS=-1;
    
    COSTHETA=-1;
    COSTHETARS=-(6*theta/pi)-5;
    COSTHETALS=1;
    
end;

if(theta>=-5*pi/6 & theta<-4*pi/6)      %-150 TO -120 DEGREES

    SINTHETA= -1;
    SINTHETARS=1;
    SINTHETALS=( theta+(4*pi/6))*6/pi;
    
    COSTHETA=-1;
    COSTHETARS=-(6*theta/pi)-5;
    COSTHETALS=1;
end;

if (theta>=-4*pi/6 & theta<-3*pi/6)     %-120 TO -90 DEGREES

   SINTHETA= -1;
   SINTHETARS=1;
   SINTHETALS=( theta+(4*pi/6))*6/pi;
   
   COSTHETA=(6*theta/pi)+3;
   COSTHETARS=-1;
   COSTHETALS=1;
end;

if(theta>=-3*pi/6 & theta<-2*pi/6)      %-90 TO -60 DEGREES

    SINTHETA= -1;
    SINTHETARS=-(theta+(2*pi/6))*6/pi;
    SINTHETALS=1;
    
    COSTHETA=(6*theta/pi)+3;
    COSTHETARS=-1;
    COSTHETALS=1;
end;

if(theta>=-2*pi/6 & theta<-pi/6)        %-60 TO -30 DEGREES

    SINTHETA= -1;
    SINTHETARS=-(theta+(2*pi/6))*6/pi;
    SINTHETALS=1;      
    
    COSTHETA=1;
    COSTHETARS=-1;
    COSTHETALS=-(6*theta/pi)-1;
    
end;

if(theta>=-pi/6 & theta<0)              %-30 TO 0 DEGREES

    SINTHETA= 6*theta/pi;
    SINTHETARS=-1;
    SINTHETALS=1;
    
    COSTHETA=1;
    COSTHETARS=-1;
    COSTHETALS=-(6*theta/pi)-1;
end;

 
 %--------------------------END OF THE LOOK UP TABLE--------------------------

 % CALCULATE THE REQUIRED PHASE CURRENTS  -- Istar
 matrix=[COSTHETA          SINTHETA           %the transformation matrix
         COSTHETARS        SINTHETARS
         COSTHETALS        SINTHETALS];
 
 Istar=matrix*I_QD;
 
 %OUTPUT THE VALUES
 %sys(1)=Istar(1,1);
 %sys(2)=Istar(2,1);
 %sys(3)=Istar(3,1);
 ss1=Istar(1,1);
 ss2=Istar(2,1);
 ss3=Istar(3,1);
 if (abs(ss1)>basecurr)           %saturate;
     sys(1)=sign(ss1)*abs(basecurr);
 else
     sys(1)=ss1;
 end;
 if (abs(ss2)>basecurr)           %saturate;
     sys(2)=sign(ss2)*basecurr;
 else
     sys(2)=ss2;
 end;
 if (abs(ss3)>basecurr)           %saturate;
     sys(3)=sign(ss3)*basecurr;
 else
     sys(3)=ss3;
 end;
 
% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

%  Example, set the next hit to be one second later.
sys = [];

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
