// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;

import frc.robot.Constants.mControl;

public class Intake extends SubsystemBase 
{
  private int pwmPort;          //PWM Port Number
  private PWM pwm;              //pwm Object
  private double m_speed;       //PWM Speed Value
  private DigitalOutput dIN1A;  //Digital Output Object for the Motor A IN1 Control Signal
  private DigitalOutput dIN2A;  //Digital Output Object for the Motor A IN1 Control Signal
  
  private mControl.STATE state; //State Value of Intake

  /** Creates a new Intake. */
  public Intake(int pPort, int dPort1, int dPort2) 
  {
    pwmPort = pPort;                    //The Assigned PWM Port
    pwm = new PWM(pwmPort);             //PWM Object
    m_speed = -1;                       //Initial set the speed to the Minimum Value
    state = mControl.STATE.BRAKE;       //Set the Initial State to Brake Mode
    dIN1A = new DigitalOutput(dPort1);  //Digital Output for the Motor A IN1 Control Signal Value
    dIN2A = new DigitalOutput(dPort2);  //Digital Output for the Motor A IN2 Control Signal Value
  }

  //Speed conversion and Limit
  private double convertSpeed(double axis)
  {
    //Limit Speed Value to [-1..1]
      if (axis > 1) axis = 1;
      else if (axis < -1) axis = -1;

      return axis;
  }

  //Directly set the Speed of the motor
  public void setSpeed(double p_speed)
  {
    m_speed = p_speed;

  }

  //Directly set the State / Direction of the motor
  public void setState(mControl.STATE p_state)
  {
    state = p_state;

  }


  @Override
  public void periodic() 
  {
    //Set the State of the Motor Controller based on the recorded state.
    switch (state) {
      case BRAKE:
        dIN1A.set(mControl.BRAKE.in1);
        dIN2A.set(mControl.BRAKE.in2);
        break;

      case CCW:
        dIN1A.set(mControl.CCW.in1);
        dIN2A.set(mControl.CCW.in2);
        break;

      case CW:
        dIN1A.set(mControl.CW.in1);
        dIN2A.set(mControl.CW.in2);
        break;

      case COAST:
        dIN1A.set(mControl.COAST.in1);
        dIN2A.set(mControl.COAST.in2);
        break;
    
      default:
        break;
    } 
  
    //Set the Speed of the Motor to the stored value
    pwm.setSpeed(convertSpeed(m_speed));

  }
}
