// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveStraight extends CommandBase {
  DriveTrain m_driveTrain;
  Joystick m_joystickLeft;

  double tankSpeed;

  /** Creates a new DriveStraight. */
  public DriveStraight(DriveTrain driveTrain, Joystick joystickLeft) {
    m_driveTrain = driveTrain;
    m_joystickLeft = joystickLeft;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
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
   // m_driveTrain.flipDrive(-m_joystickLeft.getY(), -m_joystickLeft.getY(), m_driveTrain.getFlipped());
 
 
 
   m_driveTrain.velocityDrive((Math.pow(-m_joystickLeft.getY(), 3) * .5 + -m_joystickLeft.getY()) * .2 * Constants.maxVelocity);
 

   /*  m_driveTrain.flipDrive(
    ((Math.pow(-m_joystickLeft.getY(), 3) * .5 + -m_joystickLeft.getY()) * .2), 
    ((Math.pow(-m_joystickLeft.getY(), 3) * .5 + -m_joystickLeft.getY()) * .2),
    m_driveTrain.getFlipped());
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
