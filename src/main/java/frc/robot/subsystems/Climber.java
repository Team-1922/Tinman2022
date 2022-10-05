// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
 // private Solenoid climberSolenoid = new Solenoid(0, PneumaticsModuleType.CTREPCM, 2);

  private Servo climberMotor = new Servo(2);
 
  
  /** Creates a new Climber. */
  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClimberAnglePeriodic", climberMotor.getAngle());
  }

  public void climberUp(){
    climberMotor.setAngle(90); 
    SmartDashboard.putNumber("ClimberAngle", climberMotor.getAngle());
  }

  public void climberDown(){
    climberMotor.setAngle(0); //Typically 190 testing for 0
    SmartDashboard.putNumber("ClimberAngle", climberMotor.getAngle());
   // climberSolenoid.set(false);
  }

  public void climberSet(double input){
    climberMotor.setAngle(input);
    SmartDashboard.putNumber("ClimberSetAngle", climberMotor.getAngle());
  }


  
  public double climberAngle(){
    return climberMotor.getAngle();
  }
}
