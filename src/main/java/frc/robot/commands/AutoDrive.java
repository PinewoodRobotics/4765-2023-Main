// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutoDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem m_drive;
  private double m_forwardAxis;
  private double m_leftAxis;
  private double m_seconds;
  private double iterations;
  private double counter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoDrive(DriveSubsystem subsystem, double forwardAxis, double leftAxis, double seconds) {
    m_drive = subsystem;
    m_forwardAxis = forwardAxis;
    m_leftAxis = leftAxis;
    m_seconds = seconds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    iterations = m_seconds*50;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //april tag math here

    m_drive.drive(m_forwardAxis, m_leftAxis, 0, false);
    counter +=1;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (counter >= iterations) {
      return true;
    } else {
      return false;
    }
  }
}