// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TOF extends SubsystemBase {
  private boolean leftStatus = false;
  private boolean rightStatus = false;

  private TimeOfFlight TOFLeft = new TimeOfFlight(Constants.TOFLeft);
  private TimeOfFlight TOFRight = new TimeOfFlight(Constants.TOFRight);
  /** Creates a new TOF. */
  public TOF() {

  }

  @Override
  public void periodic() {


  }

  
  

  public void putDistance(double left, double right){
    SmartDashboard.putNumber("TOFLeft16", TOFLeft.getRange());

    if(left < 90){
      leftStatus = true;
    } else {
      leftStatus = false;
    }

    if(right < 90){
      rightStatus = true;
    } else {
      rightStatus = false;
    }

    SmartDashboard.putBoolean("LeftStatus", leftStatus);
    SmartDashboard.putBoolean("RightStatus", rightStatus);

    SmartDashboard.putNumber("TOFRight16", TOFRight.getRange());
  } 

  public double getDistanceLeft(){
    return TOFLeft.getRange();

  }

  public double getDistanceRight(){
    return TOFRight.getRange();

  }
}
