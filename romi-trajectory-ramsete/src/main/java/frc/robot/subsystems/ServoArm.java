  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo; 

public class ServoArm extends SubsystemBase 
{
  Servo m_arm;
  /** Creates a new ServoArm. */
  public ServoArm(int port) 
  {
    m_arm = new Servo(port);
  }

  public void setPosition(double pos)
  {
    m_arm.setAngle(pos * 90);
  //  m_arm.set(pos);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
