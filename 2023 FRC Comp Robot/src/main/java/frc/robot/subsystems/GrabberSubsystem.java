// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.ElectronicsConstants;
import frc.robot.Constants.GrabberConstants;
//unused imports hashed out
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatusFrame;
// import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
// import edu.wpi.first.wpilibj.I2C.Port;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class GrabberSubsystem extends SubsystemBase {
  /** Creates a new GrabberSubsystem. */
  private final DoubleSolenoid GrabberSolenoid;
  
  public GrabberSubsystem() {
     GrabberSolenoid = new DoubleSolenoid(ElectronicsConstants.kPneumaticsModuleType,
    GrabberConstants.GrabberDeployedPort, GrabberConstants.GrabberRetractedPort);

 
    }
  
  //solenoid retract
  public void setGrabberRetracted() {
    GrabberSolenoid.set(Value.kReverse);
  }
  //solenoid deploy
  public void setGrabberDeployed() {
    GrabberSolenoid.set(Value.kForward);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
