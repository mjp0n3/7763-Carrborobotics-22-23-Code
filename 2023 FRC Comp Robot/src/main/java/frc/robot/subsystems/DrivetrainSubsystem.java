// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DrivetrainSubsystem extends SubsystemBase {
//motor controllers declared here
   WPI_TalonSRX RightFront = new WPI_TalonSRX(Constants.DrivetrainConstants.RightFrontID);
   WPI_TalonSRX RightBack = new WPI_TalonSRX(Constants.DrivetrainConstants.RightBackID);
   WPI_TalonSRX LeftFront = new WPI_TalonSRX(Constants.DrivetrainConstants.LeftFrontID);
   WPI_TalonSRX LeftBack = new WPI_TalonSRX(Constants.DrivetrainConstants.LeftBackID);

  //encoders were here
  //encoders were here
  
  MotorControllerGroup rightControllerGroup = new MotorControllerGroup(RightBack, RightFront);
  MotorControllerGroup leftControllerGroup = new MotorControllerGroup(LeftBack, LeftFront);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

  /** Creates a new ExampleSubsystem. */
  public DrivetrainSubsystem() {
    LeftBack.configFactoryDefault();
    LeftFront.configFactoryDefault();
    RightBack.configFactoryDefault();
    RightFront.configFactoryDefault();

    //encoder zeroing
    //encoder zeroing

    //encoder zeroing
    //encoder zeroing

    RightFront.setInverted(true);
    RightBack.setInverted(true);
    LeftFront.setInverted(false);
    LeftBack.setInverted(false);

  }

  public void arcadeDrive(double fwd, double rot) {
      differentialDrive.arcadeDrive(fwd, rot);
    }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  // @Override
  // public void simulationPeriodic() {
  //   // This method will be called once per scheduler run during simulation
  // }
}
