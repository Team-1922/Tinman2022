// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.TOF;

public class CollectorOut2 extends CommandBase {
Collector m_collector;
TOF m_TOF;
XboxController m_xbox;
boolean on = false;

double startInput = 0;

  /** Creates a new Collector_out. */
  public CollectorOut2(Collector collector, XboxController xbox) {
    m_collector = collector;
    m_xbox = xbox;
   
    addRequirements(m_collector);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled hehe i change it to execute
  @Override
  public void execute() {


if(m_xbox.getRawAxis(3) >= .1){
  m_collector.collect();
} else {
  m_collector.stopCollect();
}

}


/*

    // if (change in pushed value) is up, toggle 'on'
    if(m_xbox.getRawAxis(3) - startInput > 0){ // Shoots out immediately? Work on that issue, prob w/ this line
      if(on = true){
        on = false;
      } else {
        on = true;
      }
    }

    // Run the collector dependent on the 'on' variable
    if(on = true){
      m_collector.collect();
    } else {
      m_collector.stopCollect();
    }

    // Reset startInput for next execute cycle
    startInput = m_xbox.getRawAxis(3);

    }

*/




  @Override

  public void end(boolean interrupted) {
    m_collector.stopCollect();
  }

  @Override

  public boolean isFinished() {
    return false;//(m_TOF.getDistanceRight() < 30 && m_TOF.getDistanceLeft() < 30);
  }
 
}