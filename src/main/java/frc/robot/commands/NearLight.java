// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.TOF;




public class NearLight extends CommandBase {
  /** Creates a new NearLight. */
  TOF m_TOF;
LED m_LED;
  public NearLight(TOF TOF, LED LED) {
    m_TOF = TOF;
    m_LED = LED;
    // Use addRequirements(requirements);) here to declare subsystem dependencies.
    addRequirements(m_LED, m_TOF);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // m_TOF.setRangingMode(RangingMode.Long, 500);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ( m_TOF.getDistanceLeft()<=500 || m_TOF.getDistanceRight()<=500){
//doesn't make lead paint
    m_LED.lightUp(1,0,0);
    } else {
      m_LED.lightUp(0,1,0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  
  public void end(boolean interrupted) {
    m_LED.lightUp(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
