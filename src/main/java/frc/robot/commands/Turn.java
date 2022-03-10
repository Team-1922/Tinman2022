// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Turn extends CommandBase {
  double targetAngle;
  double currentAngle;

  double error;
  double PGain;
  double minSpeed;
  double response;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tv = table.getEntry("tv");
  DriveTrain m_drivetrain;
  /** Creates a new Turn. */
  public Turn(DriveTrain drivetrain) {
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
    NetworkTable ozram = NetworkTableInstance.getDefault().getTable("OzRam");
    PGain = ozram.getEntry("BallPGain").getDouble(.01);
    minSpeed = ozram.getEntry("BallMinSpeed").getDouble(.15);
    
    currentAngle = m_drivetrain.robotAngle();
    targetAngle = currentAngle - 180;



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = m_drivetrain.robotAngle();
// use tv to find target (1 or 0), use tx to point at it
    error = targetAngle - currentAngle;

    response = (PGain * error) + minSpeed * Math.signum(error);

    m_drivetrain.drive(response*.5, -response*.5);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(Math.abs(currentAngle - targetAngle) <= 5);

  }
}
