// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class goToBall extends CommandBase {
 /*
 * @param m_driveTrain
 * @param m_goToBall*/
  public  goToBall(DriveTrain m_driveTrain) {
    
    
      generatetrajectory();{
      var autoStart = new Pose2d(
        Units.feetToMeters(0),
        Units.feetToMeters(0),
        Rotation2d.fromDegrees(0))
      ;
      // x position
      //  y position 
      //, heading or angle
//
      var getBall = new Pose2d(
         Units.feetToMeters(3),
          Units.feetToMeters(0),
          Rotation2d.fromDegrees(0)) ;


      var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(2),Units.feetToMeters(0)));
      
// start velocity , end velocity , reversed , constraints
      TrajectoryConfig config = new TrajectoryConfig(0,0){}
;
      

     var trajectorytest = TrajectoryGenerator.generateTrajectory(
      autoStart,
      interiorWaypoints,
      getBall, 
      config
      

      );
      
    }

    // Use addRequirements() here to declare subsystem dependencies.
  }

  private void generatetrajectory() {
}

// Called when the command is initially scheduled.
  @Override
  public void initialize( 
   
    
  ) {List<Translation2d> interiorWaypoints;
    TrajectoryConfig config;
    ControlVector autoStart;
    ControlVector getBall;
    TrajectoryGenerator.generateTrajectory(
      autoStart(),
      interiorWaypoints(),
      getBall(),
      config());
      
    
  }

  private ControlVector getBall() {
    return null;
  }

  private List<Translation2d> interiorWaypoints() {
    return null;
  }

  private ControlVector autoStart() {
    return null;
  }

  private TrajectoryConfig config() {
    return null;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(
     ){  
    TrajectoryGenerator.generateTrajectory(
      autoStart(),
      interiorWaypoints(),
      getBall(), 
      config()
      
     );{}}
     
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
