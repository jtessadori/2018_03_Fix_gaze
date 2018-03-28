classdef MI_BCmouse
    properties
        fs=512; % WARNING: DO NOT change this. Amplifier acquisition rate can only be changed from its panel in Simulink
        rawData;
        cursorPos=0;
        ETcursorPos=[0,0];
        targetPos;
        trialDuration;
        figureParams;
        ETdata;
        modelName;
        timeTriggeredEvents;
        clsfr;
        outputLog;
        recLength;
        currTrial=0;
        inETregion;
        selCounter=0;
        selThreshold=.5;
        selInertia=.5;
        selDecay=0;
        UDPchannels;
        timingParams;
        targetType;
        activeTime=0;
    end
    properties (Dependent)
        currTime;
    end
    properties (Hidden)
        CDlength;
        isExpClosed=0;
        tform; % Calibration transformation from pixel coordinates of gaze to image coordinates
        gazePosBuffer; % Buffer of recent gaze points, per target
        posID; % Characters sent over udp port to signal target pos
        targetID; % Identify number of currently active target
        ETpos; % Cell of possible eye-tracking area centers
        trialStart; % Temporary vector of active time starts of current targets
        shortTargetType=[]; % Temporary vector of active targets type
        isResetting=0; % Should be one while mouse position is being reset
    end
    methods
        %% Constructor
        function obj=MI_BCmouse(varargin)
            % If an argument is passed, it must be a structure matching
            % clsfr template (i.e. basically, an output of another
            % session). Some parameters (e.g. sampling frequency of
            % amplifier and buffer window length) cannot be changed
            % directly from here, so please do make sure that they're
            % matching with the current settings of relevant Simulink
            % model.
            % Set length of initial countdown, in seconds
            obj.CDlength=2;
            
            % Set desired length of recording. 
            obj.recLength=6000;
            
            % Set colors for different objects
            obj.figureParams.bg=[.05,.05,.05];
            obj.figureParams.ETColorOn=[.1,.1,.1];
            obj.figureParams.ETColorOff=[.2,.2,.2];
            obj.figureParams.cursorColor=[.4,0,0];
            obj.figureParams.barColor=[.3,.3,.3];
            obj.figureParams.targetColor{1}=[0,0,.4];
            obj.figureParams.targetColor{2}=[.4,0,0];
            obj.figureParams.ETcursorColor=[.3,.3,.3];
            
            % Set shape and pos for cursor
            obj.figureParams.cursorShape.X=[-.05,.05,.05,-.05];
            obj.figureParams.cursorShape.Y=[-.05,-.05,.05,.05];
            obj.cursorPos=[0,0];
            
            % Set shape for ETcursor
            obj.figureParams.ETcursorRadius=.02;
            obj.figureParams.ETcursorShape.X=cos(linspace(0,2*pi,40))*obj.figureParams.ETcursorRadius;
            obj.figureParams.ETcursorShape.Y=sin(linspace(0,2*pi,40))*obj.figureParams.ETcursorRadius;
            
            % Set shape for targets
            obj.figureParams.targetRadius=.08;
            obj.figureParams.targetShape.X=cos(linspace(0,2*pi,40))*obj.figureParams.targetRadius;
            obj.figureParams.targetShape.Y=sin(linspace(0,2*pi,40))*obj.figureParams.targetRadius;
            
            % Set empty vector for current targets
            obj.figureParams.target=[];
            
            % Set eye-tracking region area
            obj.figureParams.ETradius=.35;
            ETX=(1:3)'*ones(1,3);
            ETY=ones(3,1)*(1:3);
            ETX=(ETX-2)*(1-obj.figureParams.ETradius);
            ETY=(ETY-2)*(1-obj.figureParams.ETradius);
            nETs=numel(ETX);
            obj.ETpos=cell(nETs,1);
            for currT=1:nETs
                obj.ETpos{currT}=[ETX(currT),ETY(currT)];
            end
            obj.figureParams.ETshape.X=cos(linspace(0,2*pi,40))*obj.figureParams.ETradius;
            obj.figureParams.ETshape.Y=sin(linspace(0,2*pi,40))*obj.figureParams.ETradius;
            
            % Set vertical bar positions
            obj.figureParams.barLeftPos.X=[-.4,-.4];
            obj.figureParams.barRightPos.X=[.4,.4];
            obj.figureParams.barLeftPos.Y=[-1,1];
            obj.figureParams.barRightPos.Y=[-1,1];
                        
            % Set duration of each object pass on screen and distance in
            % time of appearance of subsequent target
            obj.timingParams.targetPassLength=20;
            obj.timingParams.targetDistance=10;
            
            % Default parameters
            if nargin==0
                clsfrDefault.fs=obj.fs;
                clsfrDefault.winLength=.4; % Window length, in seconds
                clsfrDefault.relChannels=1:16; % Use all channels
                clsfrDefault.nChannels=length(clsfrDefault.relChannels);
                clsfrDefault.isTrained=0;
                obj.clsfr=clsfrDefault;
            else
                obj.clsfr=varargin{1}.clsfr;
            end
            
            % Define expected inputs and output ports for udp communication
            obj.UDPchannels.outPort=7500;
            obj.UDPchannels.inPort=6500;
            obj.UDPchannels.udps=dsp.UDPSender('RemoteIPPort',obj.UDPchannels.outPort);
            obj.UDPchannels.udpr=dsp.UDPReceiver('LocalIPPort',obj.UDPchannels.inPort,'ReceiveBufferSize',1);
            
            % Initialize a few things
            obj.outputLog.time=[];
            obj.outputLog.cursorPos=[];
            obj.outputLog.feats=[];
            obj.outputLog.targetsReached=[];
            obj.outputLog.isInTarget=[];
            obj.outputLog.selCounter=[];
            obj.outputLog.currEst=[];
            obj.outputLog.currTarget=[];
            obj.outputLog.lbls=[];
            
            % Ask user whether to start experiment right away
            clc;
            if ~strcmpi(input('Start experiment now? [Y/n]\n','s'),'n')
                obj=runExperiment(obj);
            end
        end
        
        % Other methods
        function obj=runExperiment(obj)
            % Variables on base workspace will be used to trigger closing
            % of experiment
            assignin('base','isExpClosing',0);
            
            % Start eye-tracking
            MI_BCmouse.startEyeTracking;
            
            % Launch BC mouse controller (for reasons unclear, application
            % will launch only from its directory)
            currDir=pwd;
            cd('c:\GoogleDrive\_2_Programs\_21 Processing\2018_03_21_Mouse_fix_gaze\Fix_gaze\application.windows64\')
            !Fix_gaze.exe &
            cd(currDir)
            
            % Ask user whether to calibrate eye-tracking. Cursor should not
            % be visible during calibration
            if exist('cTrans.mat','file')
                userChoice=input('Perform gaze tracking calibration? [y/N]\n','s');
            else
                userChoice='y';
            end
                        
            % Opens black figure as background
            obj=createExpFigure(obj);
            
            % Perform calibration, if needed
            if strcmpi(userChoice,'y');
                obj=obj.performETcalibration;
                calibrationTransform=obj.tform; %#ok<NASGU>
                save('cTrans.mat','calibrationTransform');
            else
                load('cTrans.mat');
                obj.tform=calibrationTransform; %#ok<NODEF>
            end
            
            % Sets name of Simulink model to be used for acquisition
            obj.modelName='SimpleAcquisition_16ch_2014a_RT_preProc';
            
            % Place eye-tracking region in the middle of the screen
            set(obj.figureParams.ET,'XData',obj.ETpos{5}(1)+obj.figureParams.ETshape.X,'YData',obj.ETpos{5}(2)+obj.figureParams.ETshape.Y);
            
            % Draw bars
            obj.figureParams.barLeft=line(obj.figureParams.barLeftPos.X,obj.figureParams.barLeftPos.Y,'color',obj.figureParams.barColor,'LineWidth',2);
            obj.figureParams.barRight=line(obj.figureParams.barRightPos.X,obj.figureParams.barRightPos.Y,'color',obj.figureParams.barColor,'LineWidth',2);
                        
            % Generate targets and put them out
            drawnow;
            
            % Prepares Simulink model (i.e. starts recording, basically)
            obj.prepareSimulinkModel;
            
            % Generates array of time triggered events
            obj.timeTriggeredEvents{1}=timeTriggeredEvent('expCallback',0);
            obj.timeTriggeredEvents{2}=timeTriggeredEvent('setTarget',Inf);
            
            % Shows a countdown
            obj.startCountdown(obj.CDlength);
            
            % Perform bulk of experiment
            obj=manageExperiment(obj);
            
            % Closes exp window and saves data
            obj.closeExp;
        end
        
        function obj=manageExperiment(obj)
            % Generate file name used to save experiment data
            fileName=datestr(now,30);
            while ~evalin('base','isExpClosing')&&obj.currTime<=(obj.recLength+obj.CDlength)
                pause(0.001);
                for currTTevent=1:length(obj.timeTriggeredEvents);
                    obj=checkAndExecute(obj.timeTriggeredEvents{currTTevent},obj.currTime,obj);
                    pause(0.001);
                end
            end
            obj.isExpClosed=1;
            delete(gcf);
            set_param(obj.modelName,'SimulationCommand','Stop');
            set_param(obj.modelName,'StartFcn','')
            obj.rawData=evalin('base','rawData');
            save(fileName,'obj');
            
            % Reset BC mouse position
            obj.UDPchannels.udps.step(uint8('0'));
            
            % Wait, then tell BCmouse controller to close
            pause(5);
            obj.UDPchannels.udps.step(uint8('exit'));
            
            % Clear variables from base workspace
            evalin('base','clear listener*');
        end
        
        function obj=createExpFigure(obj)
            % Set figure properties
            obj.figureParams.handle=gcf;
            set(obj.figureParams.handle,'Tag',mfilename,...
                'Toolbar','none',...
                'MenuBar','none',...
                'Units','normalized',...
                'Resize','off',...
                'NumberTitle','off',...
                'Name','',...
                'Color',obj.figureParams.bg,...
                'RendererMode','Manual',...
                'Renderer','OpenGL',...
                'WindowKeyPressFcn',@KeyPressed,...
                'CloseRequestFcn',@OnClosing,...
                'WindowButtonMotionFcn',@onMouseMove);
            
            % Create cursor outside of visible field
            obj.figureParams.cursor=patch(obj.figureParams.cursorShape.X+100,obj.figureParams.cursorShape.Y+100,obj.figureParams.cursorColor);
            
            % Create eye-tracker cursor outside of visible field
            obj.figureParams.ETcursor=patch(obj.figureParams.ETcursorShape.X+100,obj.figureParams.ETcursorShape.Y+100,obj.figureParams.ETcursorColor);
            
            % Create target
            obj.figureParams.ET=patch(obj.figureParams.ETshape.X,obj.figureParams.ETshape.Y,obj.figureParams.bg,'EdgeColor',obj.figureParams.ETColorOn,'FaceAlpha',0);
            
            % Set and remove figure axis
            ylim([-1,1]);
            xlim([-1,1]);
            set(gcf,'units','normalized','position',[0,0,1,1]);
            axis square
            axis('off')
            
            % Remove box around figure
            %             undecorateFig;
        end
        
        function obj=expCallback(obj)
            global udpr
            % Evaluate cursor position from gaze
            coords=str2num(char(udpr.step)'); %#ok<ST2NM>
            if ~isempty(coords)
                obj.ETcursorPos=obj.tform.transformPointsForward(coords');
            end
            
            % Filter gaze coordinates
            persistent previousGazePos
            if isempty(previousGazePos)||sum(isnan(previousGazePos))
                previousGazePos=obj.ETcursorPos;
            end
            gazeSpeed=sqrt(sum((previousGazePos-obj.ETcursorPos).^2));
            obj.ETcursorPos=previousGazePos*(1-sqrt(gazeSpeed))+obj.ETcursorPos*sqrt(gazeSpeed);
            previousGazePos=obj.ETcursorPos;
            set(obj.figureParams.ETcursor,'XData',obj.figureParams.ETcursorShape.X+obj.ETcursorPos(1),'YData',obj.figureParams.ETcursorShape.Y+obj.ETcursorPos(2));
            
            % Check if cursor is within eye-tracking region
            previousETstate=obj.inETregion;
            if sqrt(sum(obj.ETcursorPos.^2))<obj.figureParams.ETradius                
                % Set target state
                obj.inETregion=1;
                set(obj.figureParams.ET,'EdgeColor',obj.figureParams.ETColorOn);
            else
                obj.inETregion=0;
                set(obj.figureParams.ET,'EdgeColor',obj.figureParams.ETColorOff);
            end
            
            % Recover data buffer from base workspace (Simulink puts them
            % there)
            BP=evalin('base','currData')';

            % If a classifier exists, process incoming data
            if obj.clsfr.isTrained
                % If only a subset of feature has been chosen, remove the rest
                if isfield(obj.clsfr,'featsIdx')
                    BP=BP(obj.clsfr.featsIdx);
                end
                
                % Estimate current state
                [~,currEst]=predict(obj.clsfr.svm,BP);
                currEst=currEst(1);
            else
                currEst=0;
            end
            
            % Recover current mouse position
            BCmousePos=-str2double(char(obj.UDPchannels.udpr.step)');
            
            % Update cursor pos accordingly
            if ~isnan(BCmousePos)
                obj.cursorPos=BCmousePos/1e5;
                set(obj.figureParams.cursor,'XData',obj.figureParams.cursorShape.X,'YData',obj.figureParams.cursorShape.Y+obj.cursorPos*.6);
            end
            
            if obj.isResetting
                % Stop everything until mouse is back to starting position
                if BCmousePos==0
                    obj.isResetting=BCmousePos;
                end
                
                % Use specific label to indicate resetting period
                currLbl=0;
            else
                % As long as gaze rests within prescribed region, update cursor
                % position
                if obj.inETregion
                    % Update counter of time-in-ETregion
                    if ~isempty(obj.outputLog.time)
                        obj.activeTime=obj.activeTime+obj.currTime-obj.outputLog.time(end);
                    end
                    
                    % Test whether a new target needs creating
                    if obj.activeTime>=obj.timingParams.targetDistance*obj.currTrial
                        obj.currTrial=obj.currTrial+1;
                        obj.targetType(obj.currTrial)=(randn>-20)+1;
                        obj.shortTargetType(end+1)=obj.targetType(obj.currTrial);
                        obj.figureParams.target(end+1)=patch(obj.figureParams.targetShape.X+1,obj.figureParams.targetShape.Y,obj.figureParams.targetColor{obj.targetType(obj.currTrial)});
                        obj.trialStart(end+1)=obj.activeTime;
                    end
                    
                    % Update targets pos
                    for currTarget=1:length(obj.figureParams.target)
                        obj.targetPos(currTarget)=(obj.activeTime-obj.trialStart(currTarget))/obj.timingParams.targetPassLength*2-1;
                        set(obj.figureParams.target(currTarget),'XData',obj.figureParams.targetShape.X+obj.targetPos(currTarget),'YData',obj.figureParams.targetShape.Y+.6);
                    end
                    
                    % Test whether target needs removing
                    if ~isempty(obj.trialStart)&&obj.activeTime>obj.trialStart(1)+obj.timingParams.targetPassLength
                        delete(obj.figureParams.target(1));
                        obj.figureParams.target(1)=[];
                        obj.trialStart(1)=[];
                        obj.shortTargetType(1)=[];
                        obj.targetPos(1)=[];
                    end
                    
                    % Update log (i.e. check if target to be selected is
                    % between bars)
                    if sum((obj.shortTargetType==2).*(obj.targetPos>obj.figureParams.barLeftPos.X(1)).*(obj.targetPos<obj.figureParams.barRightPos.X(1)))
                        currLbl=2;
                    else
                        currLbl=1;
                    end
                    
                    % Use trained classifier when available, otherwise assume
                    % training session is underway
                    if obj.clsfr.isTrained
                        % Low-pass estimations and start moving mouse, if
                        % applicable
                        obj.selCounter=obj.selCounter*obj.selInertia+currEst*(1-obj.selInertia);
                        if obj.selCounter>obj.selThreshold
                            obj.UDPchannels.udps.step(uint8('f'));
                        else
                            obj.UDPchannels.udps.step(uint8('s'));
                        end
                        
                        % Check whether a target is leaving selection zone
                        if ~isempty(obj.targetPos)&&obj.targetPos(1)>obj.figureParams.barRightPos.X(1)
                            % Remove target 
                            delete(obj.figureParams.target(1));
                            obj.figureParams.target(1)=[];
                            obj.trialStart(1)=[];
                            obj.shortTargetType(1)=[];
                            obj.targetPos(1)=[];
                            
                            % Reset robot position
                            obj.UDPchannels.udps.step(uint8('0'));
                            obj.isResetting=1;
                        end
                    else
                        % If relevant target is between bars, start moving
                        % mouse and cursor
                        if currLbl==2
                            % Start forward mouse movement on state switch
                            if length(obj.outputLog.lbls)>1&&(obj.outputLog.lbls(end)==1||~previousETstate)
                                obj.UDPchannels.udps.step(uint8('f'));
                                fprintf('f\n');
                            end
                        end
                    end
                    
                    % Check whether target has been reached
                    if obj.cursorPos>=1
                        % Send click command
                        obj.UDPchannels.udps.step(uint8('1'));
                        pause(.1);
                        
                        % Reset robot position
                        obj.UDPchannels.udps.step(uint8('0'));
                        obj.isResetting=1;
                        
                        % Remove target just reached
                        removingTarget=find((obj.targetPos>obj.figureParams.barLeftPos.X(1)).*(obj.targetPos<obj.figureParams.barRightPos.X(1)));
                        delete(obj.figureParams.target(removingTarget));
                        targetLog.time=obj.currTime;
                        targetLog.selStart=obj.trialStart(removingTarget);
                        targetLog.type=obj.shortTargetType(removingTarget);
                        obj.figureParams.target(removingTarget)=[];
                        obj.trialStart(removingTarget)=[];
                        obj.shortTargetType(removingTarget)=[];
                        obj.targetPos(removingTarget)=[];
                        
                        % Log target reached time
                        obj.outputLog.targetsReached=cat(1,obj.outputLog.targetsReached,targetLog);
                    end
                else
                    % Stop mouse movement
                    if previousETstate
                        obj.UDPchannels.udps.step(uint8('s'));
                    end
                    currLbl=0;
                end
            end
            drawnow;
            % Add relevant info to log
            obj.outputLog.cursorPos=cat(1,obj.outputLog.cursorPos,obj.cursorPos');
            obj.outputLog.time=cat(1,obj.outputLog.time,obj.currTime);
            obj.outputLog.isInTarget=cat(1,obj.outputLog.isInTarget,obj.inETregion);
            obj.outputLog.selCounter=cat(1,obj.outputLog.selCounter,obj.selCounter);
            obj.outputLog.feats=cat(1,obj.outputLog.feats,BP);
            obj.outputLog.currEst=cat(1,obj.outputLog.currEst,currEst);
            obj.outputLog.lbls=cat(1,obj.outputLog.lbls,currLbl);
            
            % Set next evaluation time for this function
            obj.timeTriggeredEvents{1}.triggersLog=[obj.timeTriggeredEvents{1}.triggersLog,obj.currTime];
            obj.timeTriggeredEvents{1}.nextTrigger=obj.currTime+.05;
        end
                                        
        function obj=performETcalibration(obj)
            % Generate one target in each position for about 200 steps
            % (each step is 10 ms).
            global udpr
            obj.gazePosBuffer=cell(9,1);
            pause(2);
            for currTarget=1:9
                set(obj.figureParams.ET,'XData',obj.ETpos{currTarget}(1)+obj.figureParams.ETshape.X,'YData',obj.ETpos{currTarget}(2)+obj.figureParams.ETshape.Y);
                drawnow;
                obj.gazePosBuffer{currTarget}=[];
                for currRep=1:200
                    coords=str2num(char(udpr.step)'); %#ok<ST2NM>
                    obj.gazePosBuffer{currTarget}=cat(2,obj.gazePosBuffer{currTarget},coords);
                    pause(0.01);
                end
            end
            
            % Move target off-screen
            set(obj.figureParams.ET,'XData',100+obj.figureParams.ETshape.X,'YData',obj.ETpos{currTarget}(2)+obj.figureParams.ETshape.Y);
            
            % Perform actual calibration
            obj=obj.calibrateEyeTracker;
        end
       
        function obj=calibrateEyeTracker(obj)
            % Use coordinates of gaze during each target fixation to
            % calibrate transformation
            fixedPoints=cell2mat(obj.ETpos);
            movingPoints=[cellfun(@(x)median(x(1,:)),obj.gazePosBuffer),cellfun(@(x)median(x(2,:)),obj.gazePosBuffer)];
            obj.tform=fitgeotrans(movingPoints,fixedPoints,'Projective');
        end
        
        function prepareSimulinkModel(obj)
            % Check whether simulink model file can be found
            if ~exist(obj.modelName,'file')
                warning('Cannot find model %s.\nPress Enter to continue.\n',obj.modelName);
                input('');
                [fileName,pathName]=uigetfile('*.slx','Select Simulink model to load:');
                obj.modelName=sprintf('%s\\%s',pathName,fileName);
            end
            % Load model
            load_system(obj.modelName);
            
            % Check whether simulation was already running, and, in case,
            % stop it
            if bdIsLoaded(obj.modelName)&&strcmp(get_param(obj.modelName,'SimulationStatus'),'running')
                set_param(obj.modelName,'SimulationCommand','Stop');
            end
            
            % Add event listener to triggered buffer event.
            set_param(obj.modelName,'StartFcn',sprintf('simulinkModelStartFcn(''%s'')',obj.modelName))
            set_param(obj.modelName,'StopTime','inf');
            set_param(obj.modelName,'FixedStep',['1/',num2str(obj.fs)]);
            set_param(obj.modelName,'SimulationCommand','Start');
        end
        
        function wait(obj,pauseLength)
            startTime=get_param(obj.modelName,'SimulationTime');
            while strcmp(get_param(obj.modelName,'SimulationStatus'),'running')&&get_param(obj.modelName,'SimulationTime')<=startTime+pauseLength
                pause(1/(2*obj.fs));
            end
        end
        
        function startCountdown(obj,nSecs)
            % countdown to experiment start
            figure(obj.figureParams.handle)
            for cntDown=nSecs:-1:1
                if ~exist('textHandle','var')
                    textHandle=text(-.05,.5,num2str(cntDown));
                else
                    set(textHandle,'String',num2str(cntDown));
                end
                set(textHandle,'Color','white','FontSize',64);
                pause(1);
            end
            delete(textHandle);
        end
                
        function obj=computeMIclassifierSVM(obj)
            % Perform feature selection
            [obj,feats,lbls]=performFeatureSelection(obj);
            
            % Train classifier
            fprintf('Training MI classifier. Please be patient, it will take some time...\n');
            obj.clsfr.svm=fitcsvm(feats,lbls,'Standardize',true,'KernelScale','auto','KernelFunction','polynomial','PolynomialOrder',2);
            obj.clsfr.svm=fitPosterior(obj.clsfr.svm);
            obj.clsfr.isTrained=1;
        end
        
        function [obj,feats,lbls]=performFeatureSelection(obj)
            relevantData=obj.outputLog.isInTarget&(obj.outputLog.lbls>0);
            allFeats=obj.outputLog.feats;
            allFeats=allFeats(relevantData,:);
            lbls=obj.outputLog.lbls(relevantData);
            
            % Make a first selection of relevant features
            classLbls=unique(lbls);
            m=zeros(length(classLbls),size(allFeats,2));
            md=zeros(size(m));
            for currClass=1:length(classLbls)
                % Use median and mad as proxy for mean and sd, to reduce
                % relevance of artifacts
                m(currClass,:)=median(allFeats(lbls==classLbls(currClass),:));
                md(currClass,:)=1.4826*mad(allFeats(lbls==classLbls(currClass),:),1);
            end
            computeWorth=@(m1,m2,md1,md2)abs((m1-m2)./sqrt(md1.^2+md2.^2));
            featWorth=computeWorth(m(1,:),m(2,:),md(1,:),md(2,:));
            
            % Keep features with a worth greater than 0.3 (keep at least
            % 2)
            [sortedWorth,featOrdr]=sort(featWorth,'descend');
            goodFeatsNumber=sum(sortedWorth>.3);
            goodFeatsIdx=featOrdr(1:max(2,goodFeatsNumber));
            feats=allFeats(:,goodFeatsIdx);
            obj.clsfr.featsIdx=goodFeatsIdx;
        end
        
        %% Dependent properties
        function cTime=get.currTime(obj)
            if obj.isExpClosed
                cTime=obj.rawData.Time(end);
            else
                cTime=get_param(obj.modelName,'SimulationTime');
            end
        end        
    end
    methods (Static)
        function closeExp
            % Signals experiment to close
            assignin('base','isExpClosing',1);
        end
        
        function startEyeTracking
            % Losing udpr is bad (cannot close port from Matlab anymore).
            % Declare it as global so that it can be recovered outside of
            % this class, as well
            global udpr
            % Local port numeber is given by GazeTrackEyeXGazeStream
            % Buffer size is pretty much arbitrarily chosen: it should be
            % so that it contains little more than one entry
            if isempty(udpr)
                udpr=dsp.UDPReceiver('LocalIPPort',11000,'ReceiveBufferSize',20);
            end
            
            % Launch GazeTrackEyeXGazeStream in async mode, if not already
            % running (prompts user, I have no idea how to check if
            % external code is running)
            clc
            startET=input('WARNING: select Yes only if GazeStream is not running already.\nDo you want to start eye tracking? [y/N]: ','s');
            if isempty(startET)
                startET='n';
            end
            if strcmpi(startET,'y')
                !C:\Code\Sources\GazeTrackEyeXGazeStream\GazeTrackEyeXGazeStream.exe &
            end
        end
    end
end

function simulinkModelStartFcn(modelName) %#ok<DEFNU>
% Start function for Simulink model.
blockName=sprintf('%s/filterBlock/log',modelName);
assignin('base','listener',add_exec_event_listener(blockName,'PostOutputs',@acquireBufferedData));
end

function acquireBufferedData(block,~)
assignin('base','currData',block.OutputPort(1).Data);
assignin('base','currTime',block.SampleTime);
end

function onMouseMove(~,~)
% Makes mouse pointer invisible
if ~strcmp(get(gcf,'Pointer'),'custom')
    set(gcf,'PointerShapeCData',NaN(16));
    set(gcf,'Pointer','custom');
end
end

function KeyPressed(~,eventdata,~)
% This is called each time a keyboard key is pressed while the mouse cursor
% is within the window figure area
if strcmp(eventdata.Key,'escape')
    MI_BCmouse.closeExp;
end
if strcmp(eventdata.Key,'p')
    keyboard;
    %     assignin('base','pauseNextTrial',1)
end
end

function OnClosing(~,~)
% Overrides normal closing procedure so that regardless of how figure is
% closed logged data is not lost
MI_BCmouse.closeExp;
end