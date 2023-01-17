// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class DrivetrainSubsystem extends SubsystemBase {
//motor controllers declared here
   VictorSP Right = new VictorSP(Constants.DrivetrainConstants.RightID);
   VictorSP Left = new VictorSP(Constants.DrivetrainConstants.LeftID);


  //encoders were here
  //encoders were here
  
  // MotorControllerGroup leftControllerGroup = new MotorControllerGroup(Right, Rightno);
  // MotorControllerGroup rightControllerGroup = new MotorControllerGroup(Left,Leftno);

  DifferentialDrive differentialDrive = new DifferentialDrive(Right, Left);

  /** Creates a new ExampleSubsystem. */
  public DrivetrainSubsystem() {
    //restore defaults
    //restore defaults
    //restore defaults
    //restore defaults

    //encoder zeroing
    //encoder zeroing

    //encoder zeroing
    //encoder zeroing

    Right.setInverted(true);
    Left.setInverted(false);
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
