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
        public static final int RightFrontID = 4; //motor 3
        public static final int RightBackID = 3; //motor 4
        public static final int LeftFrontID = 2; //motor 1
        public static final int LeftBackID = 1;  //motor 2
        //sensitivity for robot driving with joysticksp
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
        public static final double ksVolts = 0.99606; //0.99606 old
        public static final double kvVoltSecondsPerMeter = 1.282; //366.2 old
        public static final double kaVoltSecondsSquaredPerMeter = 0.237; //132.93 old
        public static final double kPDriveVel = 1.572; //25.418 old
        
        public static final double kTrackWidthMeters = Units.inchesToMeters(21.25);
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackWidthMeters);
            
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

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
    public static final int ButtonLB=5;
    public static final int ButtonRB= 6;
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
        public static final int RightIntakeMotor = 8;

    }
    public static final class ArmConstants {
        //arm motor ports (change)
        public static final int ArmRightID = 11;
        // public static final int ArmEncoder = (1);
        //need to update
        public static final double kConversionFactor = 1;
        public static final int kCurrentLimit = 1;
        public static final double kP = 0.1;
        public static final double kI = 0.1;
        public static final double kD = 0.1;
        public static final double kFF = 0;
        public static final double kMinOutput = -0.5;
        public static final double kMaxOutput = 0.5;
        public static final double kRampRate = 0;      //change to true when add value
        public static final boolean kEnableForwardLimit = false;
        public static final boolean kEnableReverseLimit = false;
        public static final float kForwardLimit = 0;
        public static final float kReverseLimit = 0;
        public static final double khighcubepos = 0.01075;
        public static final double kmediumcubepos = 0.07117;
        public static final double kgroundpos = 0.273;
     


        //mid level = 0.07117
        //high level = 0.01075
        //store and intake and low is 0.273 (pos i think? idk, double check later)
    }

    //autonomous paths 
    public static final class PathPlannerConstants  {

        //Follow Trajectory Values
        public static final int autoMaxVelocity = 2;
        public static final int autoMaxAcceleration = 2;
        
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

        // public static final int kP_balance = 0;

        // public static final int kI_balance = 0;

        // public static final int kD_balance = 0;

        public static final double platformMaxAngle = 15;  //or maybe its 11.5? or maybe something else entirely lol

        public static final double balanceSpeed = 0.05;  //might need to ajust

        public static final double angleCoefficient = 1.5; //again, might need to ajust

        public static final int polyCoeff = 0;

        
    }


    public static final class FeedforwardConstants  {

        public static final int KsFeedForwardValue = 0;

        public static final int KvFeedForwardValue = 0;

        public static final int KaFeedForwardValue = 0;
        
        public static final int KgFeedForwardValue = 0;

    }
}
