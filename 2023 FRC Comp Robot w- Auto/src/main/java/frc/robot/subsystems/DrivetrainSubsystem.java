// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DrivetrainConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public class DrivetrainSubsystem extends SubsystemBase {
//motor controllers declared here
   WPI_TalonSRX RightFront = new WPI_TalonSRX(Constants.DrivetrainConstants.RightFrontID);
   WPI_TalonSRX RightBack = new WPI_TalonSRX(Constants.DrivetrainConstants.RightBackID);
   WPI_TalonSRX LeftFront = new WPI_TalonSRX(Constants.DrivetrainConstants.LeftFrontID);
   WPI_TalonSRX LeftBack = new WPI_TalonSRX(Constants.DrivetrainConstants.LeftBackID);

  RelativeEncoder leftEncoder = LeftFront.getEncoder();
  RelativeEncoder rightEncoder = RightFront.getEncoder();
  
  MotorControllerGroup rightControllerGroup = new MotorControllerGroup(RightBack, RightFront);
  MotorControllerGroup leftControllerGroup = new MotorControllerGroup(LeftBack, LeftFront);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);
//navx port
  public final static Gyro navX = new AHRS(SPI.Port.kMXP);

  //gyro offset 
  private int gyroOffset = 0;

  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new ExampleSubsystem. */
  public DrivetrainSubsystem() {
    LeftBack.configFactoryDefault();
    LeftFront.configFactoryDefault();
    RightBack.configFactoryDefault();
    RightFront.configFactoryDefault();

    leftEncoder.setPosition (0);
    rightEncoder.setPosition(0);

    rightEncoder.setPositionConversionFactor(DriveConstants.kLinearDistanceConversionFactor);
    leftEncoder.setPositionConversionFactor(DriveConstants.kLinearDistanceConversionFactor);
    rightEncoder.setVelocityConversionFactor(DriveConstants.kLinearDistanceConversionFactor / 60);
    leftEncoder.setVelocityConversionFactor(DriveConstants.kLinearDistanceConversionFactor / 60);

    LeftBack.follow(LeftFront);
    RightBack.follow(RightFront);

    RightFront.setInverted(false);
    RightBack.setInverted(false);
    LeftFront.setInverted(false);
    LeftBack.setInverted(false);


    navX.reset();
    navX.calibrate();
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(navX.getRotation2d(), gyroOffset, gyroOffset);
    // m_odometry.resetPosition(new Pose2d(), navX.getRotation2d());
    setBreakMode();
  }
  //coast mod
  public void setCoastMode() {
    LeftBack.setNeutralMode(NeutralMode.Coast);
    LeftFront.setNeutralMode(NeutralMode.Coast);
    RightBack.setNeutralMode(NeutralMode.Coast);
    RightFront.setNeutralMode(NeutralMode.Coast);
  }
  //break mode 
  public void setBreakMode() {
    LeftBack.setNeutralMode(NeutralMode.Brake);
    LeftFront.setNeutralMode(NeutralMode.Brake);
    RightBack.setNeutralMode(NeutralMode.Brake);
    RightFront.setNeutralMode(NeutralMode.Brake);
  }
//reset encoders
  public void resetEncoders() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
    }
  public void arcadeDrive(double fwd, double rot) {
      differentialDrive.arcadeDrive(fwd, rot);
    }
  public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    public double getRightEncoderPosition() {
      return rightEncoder.getPosition();
    }

    public double getLeftEncoderPosition() {
      return leftEncoder.getPosition();
    }
  
  public double getRightEncoderVelocity() {
    return rightEncoder.getVelocity();
    }  
    public double getLeftEncoderVelocity() {
      return leftEncoder.getVelocity();
    }
    

    
    
      public DifferentialDriveOdometry getOdometry() {
        return m_odometry;
      }
      public void resetOdometry(Pose2d pose) {
        resetEncoders();
        // m_odometry.resetPosition(pose, navX.getRotation2d());
      }
      public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
      }
    
      public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftControllerGroup.setVoltage(leftVolts);
        rightControllerGroup.setVoltage(rightVolts);
        differentialDrive.feed();
      }
    
      public double getAverageEncoderDistance() {
        return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0);
      }

      public RelativeEncoder getLeftEncoder() {
        return leftEncoder;
      }
      public RelativeEncoder getRightEncoder() {
        return rightEncoder;
      }
      public double getTurnRate() {
        return -navX.getRate();
      }
    
      public static double getHeading() {
        return navX.getRotation2d().getDegrees();
      }
      public void setMaxOutput (double maxOutput) {
        differentialDrive.setMaxOutput(maxOutput);
      }

      public static void zeroHeading()  {
        navX.calibrate();
        navX.reset();
      }
      public Gyro getGyro() {
        return getGyro();
      }

    @Override
    public void periodic() {

    // This method will be called once per scheduler run
      m_odometry.update(navX.getRotation2d(), leftEncoder.getPosition(),
      rightEncoder.getPosition());
    
    SmartDashboard.putNumber("Left encoder value meters", getLeftEncoderPosition());
    SmartDashboard.putNumber("RIGHT encoder value meters", getRightEncoderPosition());
    SmartDashboard.putNumber("Gyro heading", getHeading());
  }

}