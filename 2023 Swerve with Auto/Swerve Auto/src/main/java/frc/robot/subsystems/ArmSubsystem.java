// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  CANSparkMax ArmRight = new CANSparkMax(Constants.ArmConstants.kArmRightId, MotorType.kBrushless);
  CANSparkMax ArmLeft = new CANSparkMax(Constants.ArmConstants.kArmRightId, MotorType.kBrushless);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
      ArmRight.setInverted(true); 
  }

      //arm up
      public void setArmHigh() {
        ArmRight.set(-.50);
        ArmLeft.set(-.50);
    
      }
      //arm down
      public void setArmDown() {
        ArmRight.set(.50);
        ArmLeft.set(.50);
    
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
      }
      
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
