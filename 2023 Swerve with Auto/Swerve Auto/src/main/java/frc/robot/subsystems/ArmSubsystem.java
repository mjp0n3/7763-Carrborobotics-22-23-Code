// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  CANSparkMax ArmRight = new CANSparkMax(Constants.ArmConstants.kArmRightId, MotorType.kBrushless);
  CANSparkMax ArmLeft = new CANSparkMax(Constants.ArmConstants.kArmLeftId, MotorType.kBrushless);

  private SparkMaxPIDController arm_pidController;
  


  // private Abs = Armleft.AbsoluteEncoder;
  
  

  
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    AbsoluteEncoder arm_encoder = ArmLeft.getAbsoluteEncoder(Type.kDutyCycle);
    

    // ArmRight.setInverted(true); 
    // ArmLeft.setInverted(true); 

    //Left motor will follow Right motor
    // ArmRight.follow(ArmLeft);

    ArmRight.follow(ArmLeft, true);

    arm_pidController = ArmLeft.getPIDController();
    


  // PID coefficients
    kP = 0.8;
    kI = 0;
    kD = 0.0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    // double kgear = 0.5; //accounts for the 2:1 sprocket gearing on arm
    
    // set PID coefficients
    arm_pidController.setP(kP);
    arm_pidController.setI(kI);
    arm_pidController.setD(kD);
    arm_pidController.setIZone(kIz);
    arm_pidController.setFF(kFF);
    arm_pidController.setOutputRange(kMinOutput, kMaxOutput);


    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
    //disable pid wrapping
    arm_pidController.setPositionPIDWrappingEnabled(false);

arm_encoder.setPositionConversionFactor(1);
arm_pidController.setFeedbackDevice(arm_encoder);


  }
  

  
      
  @Override  //might have to be teleop periodic?
  public void periodic() {
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { arm_pidController.setP(p); kP = p; }
    if((i != kI)) { arm_pidController.setI(i); kI = i; }
    if((d != kD)) { arm_pidController.setD(d); kD = d; }
    if((iz != kIz)) { arm_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { arm_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      arm_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }


    
    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    // arm_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
    // SmartDashboard.putNumber("SetPoint", rotations);
    // SmartDashboard.putNumber("ProcessVariable", arm_encoder.getAbsoluteEncoder());
    }

       //arm break mode
       public void setArmBreak() {
        // ArmLeft.setNeutralMode(NeutralMode.Brake);
        ArmRight.setIdleMode(IdleMode.kBrake);
        ArmLeft.setIdleMode(IdleMode.kBrake);
    
      }
       //arm coast mode
       public void setArmCoast() {
        // ArmLeft.setNeutralMode(NeutralMode.Coast);
        ArmRight.setIdleMode(IdleMode.kCoast);
        ArmRight.setIdleMode(IdleMode.kCoast);
        
      }
      
      //arm off
      public void setmotorsoff() {
        ArmRight.set(0);
        ArmLeft.set(0);

      }

      //arm up raw
      public void setArmHigh() {
        ArmRight.set(-0.3);
        ArmLeft.set(-0.3);
      }
      //arm down raw
      public void setArmDown() {
        ArmRight.set(0.35);
        ArmLeft.set(0.35);
      }

     //set arm to stow setpoint also low outake
    public void setArmstow() {
      arm_pidController.setReference(0.09, CANSparkMax.ControlType.kPosition);
      setArmBreak();
    }
     //set arm to ground intake 
    public void setArmintake() {
      arm_pidController.setReference(0.2, CANSparkMax.ControlType.kPosition);
      setArmBreak();
    }
     //set arm to feeder intake 
    public void setArmfeeder() {
      arm_pidController.setReference(0.25, CANSparkMax.ControlType.kPosition);
      setArmBreak();

    }
    //set arm to outake mid (close)
    public void setArmmid() {
     arm_pidController.setReference(0.15, CANSparkMax.ControlType.kPosition);
     setArmBreak();
    }
    //set arm to outake (far) *lowercase* high 
    public void setArmhigh() {
     arm_pidController.setReference(0.35, CANSparkMax.ControlType.kPosition);
     setArmBreak();
    }

     
     //set arm to outake low (useless)
    public void setArmlow() {
     arm_pidController.setReference(0.4, CANSparkMax.ControlType.kPosition);
     setArmBreak();
     
    }


       
  }
  

