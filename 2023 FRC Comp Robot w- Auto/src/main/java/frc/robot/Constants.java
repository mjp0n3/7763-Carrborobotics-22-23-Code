// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this c     lass (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DrivetrainConstants{
        //motor id's
        public static final int RightFrontID = 2; //motor 3
        public static final int RightBackID = 6; //motor 4
        public static final int LeftFrontID = 7; //motor 1
        public static final int LeftBackID = 1;  //motor 2
        //sensitivity for robot driving with joysticks
        public static final double maxdrivespeed = 0.75;
        public static final double maxturnspeed = 0.75;
        //encoders
         public static final int rightEncoder = 6; 
         public static final int leftEncoder = 2;



    }

        
    public static final class DriveConstants{
        //CisID constants
        public static final double ksVolts = 0;
        public static final double kvVoltSecondsPerMeter = 0;
        public static final double kaVoltSecondsSquaredPerMeter = 0;
        public static final double kPDriveVel = 0;
        
        public static final double kTrackWidthMeters = Units.inchesToMeters(23);
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackWidthMeters);
            
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kGearRatio = 10.71;
        public static final double kWheelRadiusInches = 6;

        public static final double kLinearDistanceConversionFactor = (Units.inchesToMeters(1 / (kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches)) * 10));


    }
    
    public static final int JoystickAxis1 = 0;
    public static final int ButtonA = 1;
    public static final int ButtonB = 2;
    public static final int ButtonX = 3;
    public static final int ButtonY= 4;
    public static final int ButtonZ= 5;

    public static final class ElectronicsConstants {
        public static PneumaticsModuleType kPneumaticsModuleType = PneumaticsModuleType.REVPH;
        public static int kLEDPort = 0;
    }
    public static final class GrabberConstants {
        //solenoid ports
        public static final int GrabberDeployedPort = 2;
        public static final int GrabberRetractedPort = 3;
    }
    public static final class ArmConstants {
        //arm motor ports (change)
        public static final int ArmLeftID = 0;
        public static final int ArmRightID = 9;
    }
}
