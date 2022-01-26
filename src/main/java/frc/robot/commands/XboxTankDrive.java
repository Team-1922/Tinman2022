// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class XboxTankDrive extends CommandBase {
  DriveTrain m_drivetrain;
  XboxController m_XboxController;
  

  double encoderLeft;
  double encoderRight;

  /** Creates a new XboxTankDrive. */
  public XboxTankDrive(DriveTrain drivetrain, XboxController xboxController) {
  

m_drivetrain = drivetrain;
m_XboxController = xboxController;

addRequirements(m_drivetrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(m_XboxController.getRawAxis(1)*.20, m_XboxController.getRawAxis(5)*.20);
 
    encoderLeft = m_drivetrain.getLeftEncoder();
    encoderRight =  m_drivetrain.getRightEncoder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
