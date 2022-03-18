// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TOF;
import frc.robot.subsystems.Transfer;

public class TransferIn extends CommandBase {

  Transfer m_transfer;
  TOF m_TOF;
  /** Creates a new TransferOut. */
  public TransferIn(Transfer transfer, TOF TOF) {
    m_transfer = transfer;
    m_TOF = TOF;
    

    addRequirements(m_transfer, m_TOF);

    // Use addRequirements() here to declare subsystem dependencies.
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /*if (m_TOF.getDistanceRight() > 1 && m_TOF.getDistanceRight() < 30){
      m_transfer.sideMotor(20000); 
      m_transfer.frontMotor(-25000); 
      m_transfer.rearMotor(25000);
    } else*/ if (m_TOF.getDistanceRight() < 90){
      m_transfer.sideMotor(20000); 
      m_transfer.frontMotor(25000); 
      m_transfer.rearMotor(0);
    } else {
    m_transfer.sideMotor(20000); // Later on adjust speed, direction if needed 
    m_transfer.frontMotor(25000); 
    m_transfer.rearMotor(-8000);
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
