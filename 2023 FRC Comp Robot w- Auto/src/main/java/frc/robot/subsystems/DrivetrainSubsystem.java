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
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public class DrivetrainSubsystem extends SubsystemBase {
//motor controllers declared here
   WPI_TalonSRX RightFront = new WPI_TalonSRX(Constants.DrivetrainConstants.RightFrontID);
   WPI_TalonSRX RightBack = new WPI_TalonSRX(Constants.DrivetrainConstants.RightBackID);
   WPI_TalonSRX LeftFront = new WPI_TalonSRX(Constants.DrivetrainConstants.LeftFrontID);
   WPI_TalonSRX LeftBack = new WPI_TalonSRX(Constants.DrivetrainConstants.LeftBackID);

   //update the ports to be in constants later
   Encoder leftEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k2X); 
   Encoder rightEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X); 


  MotorControllerGroup rightControllerGroup = new MotorControllerGroup(RightBack, RightFront);
  MotorControllerGroup leftControllerGroup = new MotorControllerGroup(LeftBack, LeftFront);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);
//navx port
  public final static Gyro navX = new AHRS(SPI.Port.kMXP);

  //gyro offset 
  private int gyroOffset = 0;

  private final DifferentialDriveOdometry m_odometry;

  static AHRS m_gyro = new AHRS(SPI.Port.kMXP);


  /** Creates a new ExampleSubsystem. */
  public DrivetrainSubsystem() {
    LeftBack.configFactoryDefault();
    LeftFront.configFactoryDefault();
    RightBack.configFactoryDefault();
    RightFront.configFactoryDefault();

    leftEncoder.reset();
    rightEncoder.reset();

    // no longer needed since using quadrature encoders
    // rightEncoder.setPositionConversionFactor(DriveConstants.kLinearDistanceConversionFactor);
    // leftEncoder.setPositionConversionFactor(DriveConstants.kLinearDistanceConversionFactor);
    // rightEncoder.setVelocityConversionFactor(DriveConstants.kLinearDistanceConversionFactor / 60);
    // leftEncoder.setVelocityConversionFactor(DriveConstants.kLinearDistanceConversionFactor / 60);

    
    LeftBack.follow(LeftFront);
    RightBack.follow(RightFront);

    RightFront.setInverted(false);
    RightBack.setInverted(false);
    LeftFront.setInverted(false);
    LeftBack.setInverted(false);

    //reset and calibrate navx
    m_gyro.reset();
    m_gyro.calibrate();
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(navX.getRotation2d(), gyroOffset, gyroOffset);
    // m_odometry.resetPosition(new Pose2d(), navX.getRotation2d());
    setBreakMode();


    //lots of encoder stuff below -----------------------------------------------------

    // Configures the encoder to return a distance of 4 for every 256 pulses
    // Also changes the units of getRate
    rightEncoder.setDistancePerPulse(4.0/256.0);
    leftEncoder.setDistancePerPulse(4.0/256.0);

    // Configures the encoder to consider itself stopped after .1 seconds
    rightEncoder.setMinRate(0.1);
    leftEncoder.setMinRate(0.1);

    // Configures the encoder to consider itself stopped when its rate is below 10
    rightEncoder.setMinRate(10);
    leftEncoder.setMinRate(10);

    // Reverses the direction of the encoder
    rightEncoder.setReverseDirection(false);
    leftEncoder.setReverseDirection(false);

    // Configures an encoder to average its period measurement over 5 samples
    // Can be between 1 and 127 samples
    rightEncoder.setSamplesToAverage(5);
    leftEncoder.setSamplesToAverage(5);
  }
  // end of lots of encoder stuff ----------------------------------------------------

    



  //coast mode
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
    // Resets the encoder to read a distance of zero
  leftEncoder.reset();
  rightEncoder.reset();
    }
  public void arcadeDrive(double fwd, double rot) {
      differentialDrive.arcadeDrive(fwd, rot);
    }
  public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    public double getRightEncoderPosition() {
      return rightEncoder.getDistance();
    }

    public double getLeftEncoderPosition() {
      return leftEncoder.getDistance();
    }
  
  public double getRightEncoderVelocity() {
    return rightEncoder.getDistance();
    }  
    public double getLeftEncoderVelocity() {
      return leftEncoder.getDistance();
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

      public Encoder getLeftEncoder() {
        return leftEncoder;
      }
      public Encoder getRightEncoder() {
        return rightEncoder;
      }
      public double getTurnRate() {
        return -m_gyro.getRate();
      }
    
      public static double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
      }
      public void setMaxOutput (double maxOutput) {
        differentialDrive.setMaxOutput(maxOutput);
      }

      public static void zeroHeading()  {
        m_gyro.calibrate();
        m_gyro.reset();
      }
      public Gyro getGyro() {
        return getGyro();
      }

    @Override
    public void periodic() {

    // This method will be called once per scheduler run
      m_odometry.update(navX.getRotation2d(), leftEncoder.getDistance(),
      rightEncoder.getDistance());
    
    SmartDashboard.putNumber("Left encoder value meters", getLeftEncoderPosition());
    SmartDashboard.putNumber("RIGHT encoder value meters", getRightEncoderPosition());
    SmartDashboard.putNumber("Gyro heading", getHeading());
  }

}