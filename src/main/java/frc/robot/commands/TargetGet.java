// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TargetGet extends CommandBase {
  DriveTrain m_drivetrain;
  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight"); // Set to "limelight"
  NetworkTableEntry tx = limelight.getEntry("tx");
  NetworkTableEntry thor = limelight.getEntry("thor");

  double error;
  double startingError;
  double mod;

  double response;

  double PGain;
  double DGain;

  double speed;

  double acceptWidth;

  /** Creates a new TargetGet. */
  public TargetGet(DriveTrain drivetrain) {
    m_drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTable ozram = NetworkTableInstance.getDefault().getTable("OzRam");

    PGain = ozram.getEntry("BallPGain").getDouble(.01);
    DGain = ozram.getEntry("BallDGain").getDouble(.01);
    speed = ozram.getEntry("BallSpeed").getDouble(.25);
    acceptWidth = ozram.getEntry("AcceptableWidth").getDouble(100);

    NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("pipeline").setNumber(2); // Set to 2

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = tx.getDouble(0.0);

    mod = (startingError - error) * DGain;

    response = (PGain * error) + mod;

    if(thor.getDouble(0.0) == 0){
      m_drivetrain.drive(0, 0);
    } else {

    m_drivetrain.drive(speed - response, speed + response);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (thor.getDouble(0.0) >= 90);

  }
}
