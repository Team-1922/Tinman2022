// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  public LED() {}
  private CANifier canifier = new CANifier(7);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void lightUp(double R, double G, double B){
    canifier.setLEDOutput(G, CANifier.LEDChannel.LEDChannelA);
    canifier.setLEDOutput(R, CANifier.LEDChannel.LEDChannelB);
    canifier.setLEDOutput(B, CANifier.LEDChannel.LEDChannelC);
  } // Redo which colors correlate to which channels if we do a new canifier


}
