// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.Constants;
public class ArmSubsystem extends SubsystemBase {
   //motor controllers declared here
     VictorSP armmotor = new VictorSP(Constants.DrivetrainConstants.ArmID);
     


  public ArmSubsystem() {



  }

  //arm motor up
  public void setarmmotorup()  {
    armmotor.set(-0.80);
}
//arm motor down
  public void setarmmotordown()  {
    armmotor.set(0.4);      
}
//arm motor off
public void setmotoroff() {
armmotor.set(0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
