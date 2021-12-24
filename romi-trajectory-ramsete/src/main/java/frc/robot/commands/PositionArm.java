// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ServoArm;
import java.util.function.Supplier;


public class PositionArm extends CommandBase 
{
  private final ServoArm m_arm;
  //private final  double  m_pos;

  private final Supplier<Double> m_zaxisRotateSupplier;

  /** Creates a new PositionArm. */
  public PositionArm(ServoArm p_arm,  Supplier<Double> zaxisRotateSuppplier) 
  {
    m_arm = p_arm;
    //m_pos = 0;
    m_zaxisRotateSupplier = zaxisRotateSuppplier;
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_arm.setPosition(m_zaxisRotateSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
