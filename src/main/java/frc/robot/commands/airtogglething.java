// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CompressorSubsystem;

public class airtogglething extends CommandBase {

  RobotContainer m_robotContainer;
  CompressorSubsystem m_CompressorSubsystem;
  pneumaticSensor m_PneumaticSensor =  new pneumaticSensor();
  /** Creates a new airtogglething. */
  public airtogglething() {
   
    //addRequirements(m_CompressorSubsystem);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
    if (m_CompressorSubsystem.pneumaticSensor(m_PneumaticSensor.pneumatic) >1){
      m_CompressorSubsystem.compressortoggle();
  
  
    } 
  //  public void airtoggle
   
  // m_compssorSubsy
  
    
   // m_compressorSubsystem.setDefaultCommand(airToggle());
    if (m_CompressorSubsystem.pneumaticSensor(m_PneumaticSensor.pneumatic) <1)
      m_CompressorSubsystem.compressortoggle();
    
  // return m_compressorSubsystem.compressorStatus();
  
    }

   // m_robotContainer.airToggle();
  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
