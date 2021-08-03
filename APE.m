clear

bGetTFTopic   = false;
bGetGTTopic   = false;
bGetTFFrames  = false;
bGetGTFrames  = false;
bConstructAPE = true;
bagFilePath   = '~/.ros/KartoSLAM__2021-08-02-13-17-31_reference_bag.bag';

filename = "Base_Variables.mat";
save(filename)


%% Gets the information for TF messages processing

clear
load("Base_Variables.mat")

if bGetTFTopic

    fprintf("Getting the information for TF pre-processing... \n")

    SLAMTransTopic = '/tf';

    bagselect      = rosbag(bagFilePath);
    SLAMTFTopic    = select(bagselect, 'Topic', SLAMTransTopic );
    SLAMTFMessages = readMessages(SLAMTFTopic, 'DataFormat', 'struct');

    filename = "Bag_Reader_TF.mat";
    save(filename)
    
end


%% Gets the information for GT messages processing

clear
load("Base_Variables.mat")

if bGetGTTopic

    fprintf("Getting the information for GT pre-processing... \n")

    GTPoseTopic    = '/robotPosePublisher';

    bagselect      = rosbag(bagFilePath);
    GTPoseTopic    = select(bagselect, 'Topic', GTPoseTopic);
    GTPoseMessages = readMessages(GTPoseTopic, 'DataFormat', 'struct');

    filename = "Bag_Reader_GT.mat";
    save(filename)

end


%% Data processing for TF takes place: Relevant information to pick base_footprint to map transformation is taken.

clear
load("Base_Variables.mat")

if bGetTFFrames
    
    load("Bag_Reader_TF.mat")

    clearvars -except SLAMTFMessages

    fprintf("Getting all the individual transform messages... \n")

    % Saves all the individual tranformations (only one child frame), based on 
    % the tf tree therewill be only map to odom and odom to base_footprint 
    % transformations.
    TF_Frames = ["ChildFrameID", "FrameID", "Time Sec", "Time Nsec", "X", "Y"];
    for message = 1:length(SLAMTFMessages)
        if size(SLAMTFMessages{message,1}.Transforms,2) == 1
            ChildFrameID = SLAMTFMessages{message,1}.Transforms.ChildFrameId;
            FrameID      = SLAMTFMessages{message,1}.Transforms.Header.FrameId;
            Sec          = string(SLAMTFMessages{message,1}.Transforms.Header.Stamp.Sec);
            Nsec         = string(SLAMTFMessages{message,1}.Transforms.Header.Stamp.Nsec);
            X            = string(SLAMTFMessages{message,1}.Transforms.Transform.Translation.X);
            Y            = string(SLAMTFMessages{message,1}.Transforms.Transform.Translation.Y);

            TF_Frames = [TF_Frames; ChildFrameID, FrameID, Sec, Nsec, X, Y];
        end
    end

    clearvars -except TF_Frames

    fprintf("Generating map to base_footprint transforms... \n")

    % Makes the transformation between map and base_footprint, according to the
    % transformation tree.
    BaseFootprint_to_Map = ["Time Sec", "Time Nsec", "X", "Y"];
    for message = 2:length(TF_Frames)
       if TF_Frames(message,1) == "odom"
           if TF_Frames(message-1,1) == "base_footprint"
               X = string(str2double(TF_Frames(message,5)) + str2double(TF_Frames(message-1,5)));
               Y = string(str2double(TF_Frames(message,6)) + str2double(TF_Frames(message-1,6)));
               BaseFootprint_to_Map = [BaseFootprint_to_Map; TF_Frames(message,3), TF_Frames(message,4), X, Y];
           end
       end
    end

    clearvars -except BaseFootprint_to_Map

    filename = "TF_Position.mat";
    save(filename)
    
end


%% Data processing for GT takes place: Relevant information to pick the ground truth X and Y poses is taken.

clear
load("Base_Variables.mat")

if bGetGTFrames

    load("Bag_Reader_GT.mat")
    
    clearvars -except GTPoseMessages

    fprintf("Getting the GT pose messages... \n")

    GT_Frames = ["Time Sec", "Time Nsec", "X", "Y"];
    for message = 1:length(GTPoseMessages)
       Sec  = string(GTPoseMessages{message, 1}.Header.Stamp.Sec);
       Nsec = string(GTPoseMessages{message, 1}.Header.Stamp.Nsec);
       X    = string(GTPoseMessages{message, 1}.Pose.Position.X);
       Y    = string(GTPoseMessages{message, 1}.Pose.Position.Y);
       GT_Frames = [GT_Frames; Sec, Nsec, X, Y];
    end

    clearvars -except GT_Frames

    filename = "GT_Position.mat";
    save(filename)
    
end


%% APE calculations

clear
load("Base_Variables.mat")

if bConstructAPE
       
    load("TF_Position.mat")
    load("GT_Position.mat")
    
    APE_Matrix = ["Time Sec", "Time Nsec", "X TF", "X GT", "X error", "Y TF", "Y GT", "Y error", "Pose error"];
    for TFFrame = 2:length(BaseFootprint_to_Map)
        TFTime = str2double(BaseFootprint_to_Map(TFFrame,1)) + str2double(BaseFootprint_to_Map(TFFrame,2))*10^(-9);
        
        % Filtering by Sec
        MatchingSec = find(GT_Frames == BaseFootprint_to_Map(TFFrame,1));
        MinNsecDifference = 999999999;
        for iter = 1:length(MatchingSec)
            ActualNsecDifference = abs(str2double(GT_Frames(MatchingSec(iter),2)) - str2double(BaseFootprint_to_Map(TFFrame,2)));
            if ActualNsecDifference < MinNsecDifference
                GT_X = GT_Frames(MatchingSec(iter),3);
                GT_Y = GT_Frames(MatchingSec(iter),4);
                MinNsecDifference = ActualNsecDifference;
            end
        end
        
        X_error = abs(str2double(GT_X) - str2double(BaseFootprint_to_Map(TFFrame,3)));
        Y_error = abs(str2double(GT_Y) - str2double(BaseFootprint_to_Map(TFFrame,4)));
        PoseError = sqrt(X_error^2 + Y_error^2);
        
        APE_Matrix = [APE_Matrix; BaseFootprint_to_Map(TFFrame,1), BaseFootprint_to_Map(TFFrame,2), BaseFootprint_to_Map(TFFrame,3), GT_X, X_error, BaseFootprint_to_Map(TFFrame,4), GT_Y, Y_error, PoseError];
        
    end



    fprintf("Plotting the TF and GT poses together... \n")
    
    XPlotter_TF = str2double(APE_Matrix(2:end, 3));
    YPlotter_TF = str2double(APE_Matrix(2:end, 6));
    XPlotter_GT = str2double(APE_Matrix(2:end, 4));
    YPlotter_GT = str2double(APE_Matrix(2:end, 7));
    TPlotter_TF = str2double(APE_Matrix(2:end, 1)) + str2double(APE_Matrix(2:end, 2))*10^(-9) - (str2double(APE_Matrix(2, 1)) + str2double(APE_Matrix(2, 2))*10^(-9));

    f = figure('visible','off');
    plot(XPlotter_TF, YPlotter_TF, XPlotter_GT, YPlotter_GT)
    title("SLAM pose vs Ground Truth pose")
    hold on
    plot(XPlotter_TF(2)  , YPlotter_TF(2)  , '-pentagram', 'MarkerSize', 20, 'MarkerFaceColor', 'blue')
    plot(XPlotter_TF(end), YPlotter_TF(end), '-hexagram' , 'MarkerSize', 20, 'MarkerFaceColor', 'red')
    hold off
    Legend = legend("SLAM", "Ground truth", "Init", "End");
    Legend.Location = 'southwest';
    Legend.Color = 'none';
    Legend.EdgeColor = 'none';
    saveas(f,'TF_vs_GT_PosePlot','png')
    
    fprintf("Plotting the TF and GT poses together with time as 3D plot... \n")
    
    f = figure('visible','off');
    plot3(XPlotter_TF, YPlotter_TF, TPlotter_TF, XPlotter_GT, YPlotter_GT, TPlotter_TF)
    title("SLAM pose vs Ground Truth pose vs Time")
    hold on
    grid on
    plot3(XPlotter_TF(2)  , YPlotter_TF(2)  , TPlotter_TF(2), '-pentagram', 'MarkerSize', 20, 'MarkerFaceColor', 'blue')
    plot3(XPlotter_TF(end), YPlotter_TF(end), TPlotter_TF(end), '-hexagram' , 'MarkerSize', 20, 'MarkerFaceColor', 'red')
    zlabel("t axis (s)")
    xlabel("x axis (m)")
    ylabel("y axis (m)")
    Legend = legend("SLAM", "Ground truth", "Init", "End");
    Legend.Location = 'northwest';
    Legend.Color = 'none';
    Legend.EdgeColor = 'none';
    saveas(f,'TF_vs_GT_PosePlot_vs_Time','png')
    
    
    fprintf("Creating the statistics data charts... \n")

    f = figure('visible','off');
    subplot(2,2,1)
    histogram(str2double(APE_Matrix(2:end, 9)))
    title('Histogram of pose error')
    ylabel('Frequency')
    xlabel('Pose error (m)')
    subplot(1,2,2)
    boxplot(str2double(APE_Matrix(2:end, 9)))
    ylabel('Pose error (m)')
    title('Boxplot of pose error')
    subplot(2,2,3)
    text(0,0.96,"\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_"  ); axis off
    text(0,0.95,"\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_"  ); axis off
    text(0,0.8 ,"Mean: "                                          ); axis off
    text(0.4,0.8, string(mean(str2double(APE_Matrix(2:end, 9))))  ); axis off
    text(0,0.7 ,"Samples: "                                       ); axis off
    text(0.4,0.7, string(length(APE_Matrix(2:end, 9)))            ); axis off
    text(0,0.6 ,"Stdev: "                                         ); axis off
    text(0.4,0.6, string(std(str2double(APE_Matrix(2:end, 9))))   ); axis off
    text(0,0.5 ,"Min: "                                           ); axis off
    text(0.4,0.5, string(min(str2double(APE_Matrix(2:end, 9))))   ); axis off
    text(0,0.4 ,"Max: "                                           ); axis off
    text(0.4,0.4, string(max(str2double(APE_Matrix(2:end, 9))))   ); axis off
    text(0,0.3 ,"Mode: "                                          ); axis off
    text(0.4,0.3, string(mode(str2double(APE_Matrix(2:end, 9))))  ); axis off
    text(0,0.2 ,"Median: "                                        ); axis off
    text(0.4,0.2, string(median(str2double(APE_Matrix(2:end, 9))))); axis off
    text(0,0.16,"\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_"  ); axis off
    text(0,0.15,"\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_"  ); axis off
    title("Descriptive statistics")
    saveas(f,'APE_Statistics','png')
    
    
    fprintf("Creating the time series plot... \n")
   
    time      = str2double(APE_Matrix(2:end, 1)) + str2double(APE_Matrix(2:end, 2))*10^(-9) - str2double(APE_Matrix(2, 1));
    maxtime   = str2double(APE_Matrix(end, 1)) + str2double(APE_Matrix(end, 2))*10^(-9) - str2double(APE_Matrix(2, 1));
    stdevuprl = mean(str2double(APE_Matrix(2:end, 9))) + std(str2double(APE_Matrix(2:end, 9)));
    stdevdnrl = mean(str2double(APE_Matrix(2:end, 9))) - std(str2double(APE_Matrix(2:end, 9)));
    
    f = figure('visible','off');
    plot(time, str2double(APE_Matrix(2:end, 9)))
    xlim([0 maxtime])
    meanrl    = yline(mean(str2double(APE_Matrix(2:end, 9))), 'r--', 'LineWidth', 2);
    medianrl  = yline(median(str2double(APE_Matrix(2:end, 9))), 'g--', 'LineWidth', 2);
    stdevrl   = yline(stdevuprl, 'r', 'LineWidth', 2);
    rectstdev = rectangle('Position',[0,stdevdnrl,maxtime,stdevuprl-stdevdnrl],'FaceColor',[.5 0 0, .4],'EdgeColor','r', 'LineWidth',2);
    ylabel("Pose error (m)");
    xlabel("Time (s)");
    Legend = legend("APE", "Mean", "Median", "Stdev");
    Legend.Location = 'northwest';
    Legend.Color = 'none';
    Legend.EdgeColor = 'none';
    saveas(f,'APE_Time_Series','png')

    
end
