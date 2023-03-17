// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AlignWithTag extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_drive;
  private double m_distance;
  private double m_desiredDistance;
  private float kpDistance;
  private PIDController m_tagPID;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * 
   */
  public AlignWithTag(DriveSubsystem subsystem) {
    m_drive = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_tagPID = new PIDController(kpDistance, 1, 1);

    m_desiredDistance = 18;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    // NetworkTableEntry ty = table.getEntry("ty");

    double targetOffsetAngle_Vertical = SmartDashboard.getNumber("ty", 0);

// how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 0;

// distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 22;

// distance from the target to the floor
    double goalHeightInches = 17;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

//calculate distance
    m_distance = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

    kpDistance = -0.1f;  // Proportional control constant for distance
    double current_distance = m_distance;  // see the 'Case Study: Estimating Distance'


    double distance_error = m_desiredDistance - current_distance;

    //driveSubsystem.

    double driving_adjust = kpDistance * distance_error;

    //left_command += distance_adjust;
    //right_command += distance_adjust;

    //april tag math

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}