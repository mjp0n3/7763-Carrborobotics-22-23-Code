// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
      //motor controller for arm declared here
      CANSparkMax ArmRight = new CANSparkMax(Constants.ArmConstants.ArmRightID, MotorType.kBrushed);

  private static final double KvFeedForwardValue = 0;

  private static final double KsFeedForwardValue = 0;


  private final AbsoluteEncoder ArmEncoder;

  double requestedSpeed = 0;

  private final SparkMaxPIDController arm_PIDController;

  public double setpoint;


  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    ArmEncoder = ArmRight.getAbsoluteEncoder(Type.kDutyCycle);

    arm_PIDController = ArmRight.getPIDController();
    arm_PIDController.setFeedbackDevice(ArmEncoder);

    // Probably dont need this
    arm_PIDController.setPositionPIDWrappingEnabled(false);

    arm_PIDController.setP(ArmConstants.kP);
    arm_PIDController.setI(ArmConstants.kI);
    arm_PIDController.setD(ArmConstants.kD);
    arm_PIDController.setFF(ArmConstants.kFF);

    arm_PIDController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);
    ArmEncoder.setPositionConversionFactor(ArmConstants.kConversionFactor);
  
    //enable soft limits
    ArmRight.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ArmConstants.kEnableForwardLimit);
    ArmRight.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ArmConstants.kEnableReverseLimit);
    //set soft limits
    ArmRight.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)ArmConstants.kForwardLimit);
    ArmRight.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)ArmConstants.kReverseLimit);

    ArmRight.setSmartCurrentLimit(ArmConstants.kCurrentLimit);

    ArmRight.setOpenLoopRampRate(ArmConstants.kRampRate);

    ArmRight.setClosedLoopRampRate(ArmConstants.kRampRate);

    ArmRight.setInverted(false);
  //idk if we need this
    // ArmRight.burnFlash();

    // ArmLeft.configFactoryDefault();
    ArmRight.restoreFactoryDefaults();
    ArmRight.setInverted(false);

    //setting some stuff up
    // ArmEncoder.set(360); 
    ArmRight.setInverted(false);
    // ArmRight.maxOutput(ArmConstants.kCurrentLimit);


    // ArmRight.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ArmConstants.ksoftforwardlimit);
    // ArmRight.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ArmConstants.ksoftreverselimit);
 
    
  }
 
  //get encoder position
  public double getPosition() {
    return ArmEncoder.getPosition();
  }

    //reset encoder
    public void resetarm() {
    // ArmEncoder.resetEncoders();
    }
    
  
  //arm up
  public void setArmHigh() {
    ArmRight.set(-.60);
  }
  //arm down
  public void setArmDown() {
    ArmRight.set(.60);
  }

  
  // Creates a PIDController with gains kP, kI, and kD
PIDController pid = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

// Calculates the output of the PID algorithm based on the sensor reading
// and sends it to a motor


  //arm break mode
  public void setArmBreak() {
    // ArmLeft.setNeutralMode(NeutralMode.Brake);
    ArmRight.setIdleMode(IdleMode.kBrake);
  }
   //arm normal mode
   public void setArmCoast() {
    // ArmLeft.setNeutralMode(NeutralMode.Coast);
    ArmRight.setIdleMode(IdleMode.kCoast);
  }

  //arm off
  public void setmotorsoff() {
    ArmRight.set(0);
  }
  //set positions

  //arm high cone command
  public void setArmHighCone() {}
  
  //arm mid cone command
  public void setMidHighCone() {}

  //arm high cube command
  public void setArmHighCube() {}

  //arm mid cube command
  public void setArmMidCube() {}

  //arm ground intake command
  public void setArmGround() {}

  //arm double feeder command
  public void setArmFeeder() {}

  ///arm store command
  public void setArmStore() {}

  //arm single feeder command 
  //nothing since we dont do single feeder L


  //feedforward stuff 

  ArmFeedforward feedforward = new ArmFeedforward(
  Constants.FeedforwardConstants.KsFeedForwardValue,
  Constants.FeedforwardConstants.KgFeedForwardValue,
  Constants.FeedforwardConstants.KvFeedForwardValue, 
  Constants.FeedforwardConstants.KaFeedForwardValue);

  
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm encoder value meters", ArmEncoder.getPosition());
    SmartDashboard.putNumber("Applied Speed", ArmRight.getAppliedOutput());
    // SmartDashboard.putNumber("SparkMaxState", ArmRight.getIdleMode());
    SmartDashboard.putNumber("Desired Speeed", requestedSpeed);
  }
}
  