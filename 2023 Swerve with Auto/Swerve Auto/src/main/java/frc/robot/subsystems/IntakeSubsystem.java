// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax IntakeMotor = new CANSparkMax(Constants.kIntakeMotorId, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}


     //Intake Cube
     public void setIntakeON() {
      IntakeMotor.set(-0.5);
    
  
    }
    //Outake Cube
    public void setOutakeON() {
      IntakeMotor.set(0.5);
  
  
    }
  
      //arm break mode
    public void setIntakeBreak() {
      // ArmLeft.setNeutralMode(NeutralMode.Brake);
      IntakeMotor.setIdleMode(IdleMode.kBrake);
    
  
    }
     //arm coast mode
     public void setIntakeCoast() {
      // ArmLeft.setNeutralMode(NeutralMode.Coast);
      IntakeMotor.setIdleMode(IdleMode.kCoast);

  
    }
  
    //arm off
    public void setIntakeOFF() {
      IntakeMotor.set(0);
   

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
