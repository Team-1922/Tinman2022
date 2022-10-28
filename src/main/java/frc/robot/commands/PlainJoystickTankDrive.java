// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PlainJoystickTankDrive extends CommandBase {
  DriveTrain m_drivetrain;
  Joystick m_joystickLeft;
  Joystick m_joystickRight;

  double tankSpeed;
  double deadzone = .15;
 
  /** Creates a new JoystickTankDrive. */
  public PlainJoystickTankDrive(DriveTrain drivetrain, Joystick JoystickLeft, Joystick JoystickRight) {
    m_drivetrain = drivetrain;
    m_joystickLeft = JoystickLeft;
    m_joystickRight = JoystickRight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  NetworkTable ozram = NetworkTableInstance.getDefault().getTable("OzRam");
  tankSpeed = ozram.getEntry("TankSpeed").getDouble(.25);
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
/*
    double turningScale = 0.5*(1-Math.abs(m_joystickLeft.getY()/3))*0.5; 
    double throttleScale = 0.6;
   
    
   double arcadedrivepartone  = m_joystickLeft.getY()*throttleScale-m_joystickRight.getX()*turningScale;
   double arcadedriveparttwo =m_joystickLeft.getY()*throttleScale+m_joystickRight.getX()*turningScale;
   */
// because I do noooooot understand the new controls

if(m_joystickLeft.getY() > deadzone || m_joystickLeft.getY() < -deadzone || m_joystickRight.getY() > deadzone || m_joystickRight.getY() < -deadzone){

if(m_drivetrain.getFlipped() == true){
  m_drivetrain.flipDrive(
    ((Math.pow(-m_joystickRight.getY(), 3) * .5 + -m_joystickRight.getY()) * tankSpeed), 
    ((Math.pow(-m_joystickLeft.getY(), 3) * .5 + -m_joystickLeft.getY()) * tankSpeed),
    m_drivetrain.getFlipped());

} else {

   m_drivetrain.flipDrive(
     ((Math.pow(-m_joystickLeft.getY(), 3) * .5 + -m_joystickLeft.getY()) * tankSpeed), 
     ((Math.pow(-m_joystickRight.getY(), 3) * .5 + -m_joystickRight.getY()) * tankSpeed),
     m_drivetrain.getFlipped());
}
} else {
  m_drivetrain.flipDrive(0, 0, m_drivetrain.getFlipped());

}
  }
   
/*
  m_drivetrain.flipDrive(
    ((Math.pow(-m_joystickRight.getY(), 3) * .5 + -m_joystickRight.getY()) * tankSpeed), 
    ((Math.pow(-m_joystickLeft.getY(), 3) * .5 + -m_joystickLeft.getY()) * tankSpeed),
    m_drivetrain.getFlipped()); 
 
  }*/


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
