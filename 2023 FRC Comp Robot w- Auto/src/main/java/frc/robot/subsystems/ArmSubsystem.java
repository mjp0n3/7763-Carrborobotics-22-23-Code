// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class ArmSubsystem extends SubsystemBase {
  //motor controllers for arm declared here
    WPI_TalonSRX ArmRight = new WPI_TalonSRX(Constants.ArmConstants.ArmRightID);
    WPI_TalonSRX ArmLeft = new WPI_TalonSRX(Constants.ArmConstants.ArmLeftID);
  
  //encoders go here
  //encoders go here

  MotorControllerGroup armControllerGroup = new MotorControllerGroup(ArmRight, ArmLeft);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    ArmLeft.configFactoryDefault();
    ArmRight.configFactoryDefault();

    //encoder zeroing
    //encoder zeroing

    ArmRight.setInverted(false);
    ArmLeft.setInverted(false);
  }
      //solenoid retract
  public void setArmHigh() {
    armControllerGroup.set(.50);
  }
  //arm down
  public void setArmDown() {
    armControllerGroup.set(-.50);
  }

  //arm break mode
  public void setArmBreak() {
    ArmLeft.setNeutralMode(NeutralMode.Brake);
    ArmRight.setNeutralMode(NeutralMode.Brake);
  }

  //arm off
  public void setmotorsoff() {
    armControllerGroup.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
