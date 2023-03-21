// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutoBumpScore extends SequentialCommandGroup {
  /**
   * Creates a new ExampleCommand.
   *
   *
   */
  public AutoBumpScore(DriveSubsystem m_drive) {
    addCommands(
    new AutoDrive(m_drive, -0.25, 0, 0.25)
    );

  }

  // Called once the command ends or is interrupted.
}