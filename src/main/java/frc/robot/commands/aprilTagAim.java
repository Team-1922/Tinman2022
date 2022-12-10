// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class aprilTagAim extends CommandBase {
  PhotonCamera limelight = new PhotonCamera("gloworm");

  DriveTrain m_driveTrain;
  PhotonPipelineResult result;
  double distance;
  double PGain = .01;

  double response;


  /** Creates a new aprilTagAim. */
  public aprilTagAim(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    result = limelight.getLatestResult();

    if(result.hasTargets()){

      distance = PhotonUtils.calculateDistanceToTargetMeters(.9, .8, 0,  Units.degreesToRadians(result.getBestTarget().getPitch()));
      SmartDashboard.putNumber("DistanceToTag", distance);
      response = PGain * result.getBestTarget().getYaw() + (.1 * Math.signum(result.getBestTarget().getYaw()));


      m_driveTrain.drive(response - .25, -response - .25);




    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (distance <= 2);
  }
}
