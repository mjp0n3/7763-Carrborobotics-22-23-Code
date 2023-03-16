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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
  private static final double KvFeedForwardValue = 0;

  private static final double KsFeedForwardValue = 0;

    //motor controllers for arm declared here
    CANSparkMax ArmRight = new CANSparkMax(Constants.ArmConstants.ArmRightID, MotorType.kBrushed);
    // WPI_TalonSRX ArmLeft = new WPI_TalonSRX(Constants.ArmConstants.ArmLeftID);
  
    DutyCycleEncoder ArmEncoder = new DutyCycleEncoder(Constants.ArmConstants.ArmEncoder);

  MotorControllerGroup armControllerGroup = new MotorControllerGroup(ArmRight); //, armleft

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // ArmLeft.configFactoryDefault();
    ArmRight.restoreFactoryDefaults();
    ArmRight.setInverted(false);

    //setting some stuff up
    ArmEncoder.setDistancePerRotation(360); 
    ArmRight.setInverted(false);
    // ArmRight.maxOutput(ArmConstants.kCurrentLimit);
    ArmRight.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ArmConstants.enabletreverselimit);
    ArmRight.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ArmConstants.enableforwardlimit);

    // ArmRight.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ArmConstants.ksoftforwardlimit);
    // ArmRight.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ArmConstants.ksoftreverselimit);

  }
  double requestedSpeed = 0;

  //get encoder position
  public double getPosition() {
    return ArmEncoder.getAbsolutePosition();
  }

    //reset encoder
    public void resetarm() {
    ArmEncoder.reset();
    }
    
  
  //arm up
  public void setArmHigh() {
    ArmRight.set(.60);
  }
  //arm down
  public void setArmDown() {
    ArmRight.set(-.60);
  }



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
    armControllerGroup.set(0);
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
    SmartDashboard.putNumber("Arm encoder value meters", ArmEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Applied Speed", ArmRight.getAppliedOutput());
    // SmartDashboard.putNumber("SparkMaxState", ArmRight.getIdleMode());
    SmartDashboard.putNumber("Desired Speeed", requestedSpeed);
  }
}
  