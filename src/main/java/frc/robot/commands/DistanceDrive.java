// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DistanceDrive extends CommandBase {
  DriveTrain m_driveTrain;
  double distance;

  double startEncoder;
  double newEncoder;

  double speed = 15000;

  boolean motion = false;

  /** Creates a new DistanceDrive. */
  public DistanceDrive(DriveTrain driveTrain, double targetDistance) {
    m_driveTrain = driveTrain;
    distance = targetDistance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startEncoder = m_driveTrain.getLeftEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    newEncoder = m_driveTrain.getLeftEncoder();
    m_driveTrain.velocityDrive(speed);

    if(m_driveTrain.getLeftEncoder() >= speed * .8 && m_driveTrain.getLeftEncoder() <= speed * 1.2){
      motion = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(newEncoder - startEncoder) >= Math.abs(distance) || (m_driveTrain.motorVelocityLeft() <= speed / 2) && motion == true);
    
  }
}
