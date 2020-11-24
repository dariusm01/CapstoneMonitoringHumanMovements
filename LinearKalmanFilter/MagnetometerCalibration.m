function [MagXSI, MagYSI, MagZSI] = MagnetometerCalibration(MagX, MagY, MagZ)

    %% Plotting the uncalibrated magnetic field
    % figure(1)
    % plot(MagX,MagY)
    % xlabel("Magnetic Measurements X");
    % ylabel("Magnetic Measurements Y");
    % title("Uncalibrated Magnetic Field");
    % grid on

    %% Hard Iron Correction
%     MagXOffset = (max(MagX)+min(MagX))/2;
%     MagYOffset = (max(MagY)+min(MagY))/2;
%     MagZOffset = (max(MagZ)+min(MagZ))/2;

    D  = [MagX MagY MagZ];

    % b is correction vector for the hard-iron effect, returned as a 3-by-1 array.
    [~,b,~] = magcal(D, 'auto');
    
    MagXOffset = b(1);
    MagYOffset = b(2);
    MagZOffset = b(3);

    MagXHI = MagX-MagXOffset;
    MagYHI = MagY-MagYOffset;
    MagZHI = MagZ-MagZOffset;

    % figure(2)
    % plot(MagXHI,MagYHI)
    % xlabel("Magnetic Measurements X");
    % ylabel("Magnetic Measurements Y");
    % title("Hard Iron Corrected Magnetic Field Measurements");
    % grid on

    %% Soft Iron Correction 
    chordX = (max(MagXHI) - min(MagXHI))/2;
    chordY = (max(MagYHI) - min(MagYHI))/2;
    chordZ = (max(MagZHI) - min(MagZHI))/2;

    chord_average = (chordX + chordY + chordZ)/3;

    MagXScale = chord_average/chordX;
    MagYScale = chord_average/chordY;
    MagZScale = chord_average/chordZ;

    MagXSI = MagXHI*MagXScale;
    MagYSI = MagYHI*MagYScale;
    MagZSI = MagZHI*MagZScale;

    % figure(3)
    % plot(MagXSI,MagYSI)
    % xlabel("Magnetic Measurements X");
    % ylabel("Magnetic Measurements Y");
    % title("Hard Iron & Soft Iron Corrected Magnetic Field Measurements");
    % grid on

%     figure(4)
%     plot(MagXSI,MagYSI)
%     xlabel("Magnetic Measurements X");
%     ylabel("Magnetic Measurements Y");
%     title("Hard Iron & Soft Iron Corrected Magnetic Field Measurements");
%     grid on
%     hold on 
%     plot(MagX,MagY)
%     legend('Calibrated Samples', 'Uncalibrated Samples')
% 
%     figure(5)
%     plot3(MagXSI,MagYSI,MagZSI)
%     xlabel("Magnetic Measurements X");
%     ylabel("Magnetic Measurements Y");
%     zlabel("Magnetic Measurements Z");
%     title("Hard Iron & Soft Iron Corrected Magnetic Field Measurements");
%     grid on
%     hold on 
%     % axis equal
%     plot3(MagX,MagY,MagZ)
%     legend('Calibrated Samples', 'Uncalibrated Samples')
end 