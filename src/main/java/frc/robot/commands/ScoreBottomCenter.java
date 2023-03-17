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
public class ScoreBottomCenter extends SequentialCommandGroup {
  /**
   * Creates a new ExampleCommand.
   *
   *
   */
  public ScoreBottomCenter(DriveSubsystem m_drive, ArmSubsystem m_arm) {
    addCommands(
    new AlignWithTag(m_drive),
    new SetArm(m_arm, ArmConstants.kPitchCubeBotScoreMod, ArmConstants.kReachCubeBotScoreMod),
    new OpenClaw(),
    new RetractArm(m_arm)
    );

  }

  // Called once the command ends or is interrupted.
}