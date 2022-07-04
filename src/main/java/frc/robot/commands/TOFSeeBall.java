// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TOF;

public class TOFSeeBall extends CommandBase {
  TOF m_TOF;
  /** Creates a new TOFSeeBall. */
  public TOFSeeBall(TOF TOF) {
    m_TOF = TOF;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_TOF);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_TOF.getDistanceLeft() <= 50 && m_TOF.getDistanceRight() <= 50); 
  }
}
