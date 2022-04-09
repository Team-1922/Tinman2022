// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class BallAim extends CommandBase {
  DriveTrain m_drivetrain;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-collect");
 NetworkTableEntry tx = table.getEntry("tx");

 double pipeline;

 double error;
 double startingError;
 double mod;

 double response;

 double PGain;
 double DGain;

 double timerGoal;
 double timerStart;
 double minSpeed;
 double angle;

  /** Creates a new BallAim. */
  public BallAim(DriveTrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTable ozram = NetworkTableInstance.getDefault().getTable("OzRam");

    PGain = ozram.getEntry("BallPGain").getDouble(.01);
    DGain = ozram.getEntry("BallDGain").getDouble(.01);
    timerGoal = ozram.getEntry("BallTimeGoal").getDouble(10);
    minSpeed = ozram.getEntry("BallMinSpeed").getDouble(.15);
    angle = ozram.getEntry("BallAngle").getDouble(2);

    pipeline = ozram.getEntry("Pipeline").getDouble(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = tx.getDouble(0.0);

    mod = (startingError - error) * DGain;

    response = PGain * error + mod + minSpeed * Math.signum(error);


    SmartDashboard.putNumber("BallResponse", response);

    m_drivetrain.drive(response*.5, -response*.5);

    startingError = error;


    if(error <= angle && error >= -angle){
      timerStart++;
    } else {
      timerStart = 0;
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
    if(timerStart == timerGoal){
      return true;
    } else{
      return false;
    }
    
  }
}
