// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class Score extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_drive;
  private final ArmSubsystem m_arm;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Score(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {
    m_drive = driveSubsystem;
    m_arm = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //hey shuffleboard whats the fuckin number
    //idk dude go shove a game piece down ur drive base
    int scoreNumber = 0;
    if (SmartDashboard.getBoolean("Top Left", false) == true) {
      scoreNumber = 1;
    } else if (SmartDashboard.getBoolean("Top Center", false) == true) {
      scoreNumber = 2;
    } else if (SmartDashboard.getBoolean("Top Right", false) == true) {
      scoreNumber = 3;
    } else if (SmartDashboard.getBoolean("Mid Left", false) == true) {
      scoreNumber = 4;
    } else if (SmartDashboard.getBoolean("Mid Center", false) == true) {
      scoreNumber = 5;
    } else if (SmartDashboard.getBoolean("Mid Right", false) == true) {
      scoreNumber = 6;
    } else if (SmartDashboard.getBoolean("Bottom Left", false) == true) {
      scoreNumber = 7;
    }else if (SmartDashboard.getBoolean("Bottom Center", false) == true) {
      scoreNumber = 8;
    }else if (SmartDashboard.getBoolean("Bottom Right", false) == true) {
      scoreNumber = 9;
    }

    switch (scoreNumber) {
      case 1: new ScoreTopLeft(m_drive, m_arm);
              System.out.println("top left score");
              break;
      case 2: new ScoreTopCenter(m_drive, m_arm);
              System.out.println("top center score");
              break;
      case 3: new ScoreTopRight(m_drive, m_arm);
              System.out.println("top right score");
              break;
      case 4: new ScoreMidLeft(m_drive, m_arm);
              System.out.println("mid left score");
              break;
      case 5: new ScoreMidCenter(m_drive, m_arm);
              System.out.println("mid center score");
              break;
      case 6: new ScoreMidRight(m_drive, m_arm);
              System.out.println("mid right score");
              break;
      case 7: new ScoreBottomLeft(m_drive, m_arm);
              System.out.println("bottom left score");
              break;
      case 8: new ScoreBottomCenter(m_drive, m_arm);
              System.out.println("bottom center score");
              break;
      case 9: new ScoreBottomRight(m_drive, m_arm);
              System.out.println("bottom right score");
              break;
      default: break;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}