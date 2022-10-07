// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TOF;
import frc.robot.subsystems.Transfer;

public class TransferIn2 extends CommandBase {

  Transfer m_transfer;
  TOF m_TOF;
  XboxController m_xbox;
  /** Creates a new TransferOut. */
  public TransferIn2(Transfer transfer, TOF TOF, XboxController xbox) {
    m_transfer = transfer;
    m_TOF = TOF;
    m_xbox = xbox;
    

    addRequirements(m_transfer, m_TOF);

    // Use addRequirements() here to declare subsystem dependencies.
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_xbox.getRawAxis(2) > .1){

   if (m_TOF.getDistanceRight() < 90 && m_TOF.getDistanceLeft() < 90){
      m_transfer.sideMotor(10000);
      m_transfer.frontMotor(6000);
      m_transfer.rearMotor(6000);
    } else if (m_TOF.getDistanceRight() < 90){
      m_transfer.sideMotor(15000); 
      m_transfer.frontMotor(25000); 
      m_transfer.rearMotor(0);
    } else {
    m_transfer.sideMotor(15000); // Later on adjust speed, direction if needed 
    m_transfer.frontMotor(25000); 
    m_transfer.rearMotor(-8000);
    }

  } else {
    m_transfer.sideMotor(0); // Later on adjust speed, direction if needed 
    m_transfer.frontMotor(0);
    m_transfer.rearMotor(0);
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transfer.sideMotor(0); // Later on adjust speed, direction if needed 
    m_transfer.frontMotor(0);
    m_transfer.rearMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
