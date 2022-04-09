// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.TOF;

public class TOFDrive extends CommandBase {
  DriveTrain m_driveTrain;
  TOF m_TOF;

  double speed = 15000;


  /** Creates a new DistanceDrive. */
  public TOFDrive(DriveTrain driveTrain, TOF TOF) {
    m_driveTrain = driveTrain;
    m_TOF = TOF;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain, m_TOF);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.velocityDrive(speed);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_TOF.getDistanceLeft() <= 100 && m_TOF.getDistanceRight() <= 100);
    
  }
}
