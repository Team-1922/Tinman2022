// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.management.MBeanAttributeInfo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Auto_forward extends CommandBase {
  private double newEncoder;
  private double encoder;
  private DriveTrain m_driveTrain;
  /** Creates a new Auto_forward. */
  public Auto_forward(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain=driveTrain;
  addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoder=m_driveTrain.getLeftEncoder();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   //.addchild .addchild .addchild
   newEncoder=m_driveTrain.getLeftEncoder();
   m_driveTrain.drive(-.25,-.25);
  }
 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
if (newEncoder-encoder>=12){
  return true;
}

    return false;
    
  }
}
