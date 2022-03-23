// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class JoystickTankDrive extends CommandBase {
  DriveTrain m_drivetrain;
  Joystick m_joystickLeft;
  Joystick m_joystickRight;

  double tankSpeed;
  /** Creates a new JoystickTankDrive. */
  public JoystickTankDrive(DriveTrain drivetrain, Joystick JoystickLeft, Joystick JoystickRight) {
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
    m_drivetrain.drive((m_joystickLeft.getY()*0.6-m_joystickRight.getX()*0.5*(1-Math.abs(m_joystickLeft.getY()/3))*0.5),(m_joystickLeft.getY()*0.6+m_joystickRight.getX()*0.5*(1-Math.abs(m_joystickLeft.getY()/3))*0.5));

   // m_drivetrain.drive(-m_joystickLeft.getY()*tankSpeed, -m_joystickRight.getY()*tankSpeed);
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
