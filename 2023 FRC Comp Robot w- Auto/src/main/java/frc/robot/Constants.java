// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
        public static final double maxdrivespeed = 0.70;
        public static final double maxturnspeed = 0.70;
        // right encoder
         public static final int rightEncoderA = 2;
         public static final int rightEncoderB = 3; 
        //left encoder
         public static final int leftEncoderA = 4;
         public static final int leftEncoderB = 5; 


    }

    public static final class DriveConstants{
        //CisID constant
        public static final double ksVolts = 0.99606;
        public static final double kvVoltSecondsPerMeter = 366.2;
        public static final double kaVoltSecondsSquaredPerMeter = 132.93;
        public static final double kPDriveVel = 25.418;
        
        public static final double kTrackWidthMeters = Units.inchesToMeters(21.25);
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackWidthMeters);
            
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kGearRatio = 12.75;
        public static final double kWheelRadiusInches = 6;

        public static final double kLinearDistanceConversionFactor = (Units.inchesToMeters(1 / (kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches)) * 10));


    }
    
    public static final int JoystickAxis1 = 0;
    public static final int JoystickAxis2 = 1;
    public static final int ButtonA = 1; //up
    public static final int ButtonB = 2;
    public static final int ButtonX = 3; //down
    public static final int ButtonY= 5;
    public static final int ButtonTL= 6;
    public static final int ButtonTR= 7;
    public static final int ButtonBack= 8;
    public static final int ButtonFwd= 9;



    // public static final JoystickButton driverBumperLeft = new JoystickButton(JoystickAxis1, 5);
    // public static final JoystickButton driverBumperRight = new JoystickButton(JoystickAxis1, 6);
    
    


    public static final class ElectronicsConstants {
        public static PneumaticsModuleType kPneumaticsModuleType = PneumaticsModuleType.REVPH;
        public static int kLEDPort = 0;
    }
    public static final class GrabberConstants {
        //solenoid ports
        public static final int GrabberDeployedPort = 2;
        public static final int GrabberRetractedPort = 3;
        //intake motors
        public static final int RightIntakeMotor = 3;
        public static final int LeftIntakeMotor = 4;
    }
    public static final class ArmConstants {
        //arm motor ports (change)
        public static final int ArmRightID = 0;
        // public static final int ArmLeftID = 69;
        public static final int ArmEncoder = (1);
    }

    //autonomous paths 
    public static final class PathPlannerConstants  {

        //Follow Trajectory Values
        public static final int autoMaxVelocity = 3;
        public static final int autoMaxAcceleration = 3;
        
        //2cube feeder side no climb auto           
        public static final String fs_2cubepath1 = "FS_2Cube1";
        public static final String fs_2cubepath2 = "FS_2Cube2";
        public static final String fs_2cubepath3 = "FS_2Cube3";
        public static final String fs_2cubepath4 = "FS_2Cube4";
        public static final String fs_2cubepath5 = "FS_2Cube5";
       
        //1 cube 1 pickup other side climb auto
        public static final String os_cclimbpath1 = "src/main/deploy/deploy/pathplanner/generatedJSON/OS_CClimb1.wpilib.json";
        public static final String os_cclimbpath2 = "src/main/deploy/deploy/pathplanner/generatedJSON/OS_CClimb2.wpilib.json";
        public static final String os_cclimbpath3 = "src/main/deploy/deploy/pathplanner/generatedJSON/OS_CClimb3.wpilib.json";
        public static final String os_cclimbpath4 = "src/main/deploy/deploy/pathplanner/generatedJSON/OS_CClimb4.wpilib.json";
        public static final String os_cclimbpath5 = "src/main/deploy/deploy/pathplanner/generatedJSON/OS_CClimb5.wpilib.json";
    }
    //Charge Station Balance PID values
    public static final class Balanceconstants  {

        public static final int kP_balance = 0;

        public static final int kI_balance = 0;

        public static final int kD_balance = 0;
    }


    public static final class FeedforwardConstants  {

        public static final int KsFeedForwardValue = 0;

        public static final int KvFeedForwardValue = 0;

        public static final int KaFeedForwardValue = 0;
        
        public static final int KgFeedForwardValue = 0;

    }
}
