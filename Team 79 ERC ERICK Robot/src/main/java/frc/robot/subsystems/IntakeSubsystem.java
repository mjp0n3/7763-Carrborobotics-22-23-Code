// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.Constants;
public class IntakeSubsystem extends SubsystemBase {
     //motor controllers declared here
     VictorSP intakemotor = new VictorSP(Constants.DrivetrainConstants.IntakeID);
     
    
  
  public IntakeSubsystem() {

 

  }

  //intake motor in
  public void setintakemotor()  {
        intakemotor.set(0.75);
  }
  //intake motor out
  public void setouttakemotor()  {
        intakemotor.set(-0.75);      
  }
  //intake motor off
  public void setmotoroff() {
    intakemotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  }

