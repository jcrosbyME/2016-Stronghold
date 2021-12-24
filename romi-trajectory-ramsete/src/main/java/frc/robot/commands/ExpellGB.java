// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.mControl;
import frc.robot.subsystems.Intake;

public class ExpellGB extends CommandBase 
{
  private Intake m_intake;          //Intake Sub-system
  private Boolean m_done = false;   //Start as not done

  /** Creates a new IntakeGB. */
  public ExpellGB(Intake p_intake) 
  {
    m_intake = p_intake;                    //Save the Intake Subsystem locally
    m_intake.setState(mControl.reverse);    //Set the "State" to reverse = Expell Golf Balls
    m_intake.setSpeed(mControl.maxSpeed);   //Set the Speed to Maximum
    addRequirements(m_intake);              //Make the Subsystem Required
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    //End Command on Interrupt
    if (interrupted == true)
      m_done = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return m_done;
  }
}
