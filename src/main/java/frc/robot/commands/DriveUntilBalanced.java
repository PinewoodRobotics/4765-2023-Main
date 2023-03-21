// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class DriveUntilBalanced extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_drive;
  //private final double m_axis;
  private double m_pitch;
  private double m_tempPitch;
  private AHRS ahrs = new AHRS(SPI.Port.kMXP);
  private boolean climbed = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveUntilBalanced(DriveSubsystem subsystem, double axis) {
    m_drive = subsystem;
    //m_axis = axis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  public DriveUntilBalanced(DriveSubsystem subsystem) {
    m_drive = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {



    m_pitch = ahrs.getRoll(); //grab from navx
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_tempPitch = ahrs.getRoll(); //get from navx

SmartDashboard.putNumber("NavX Pitch", ahrs.getPitch());
SmartDashboard.putNumber("NavX Yaw", ahrs.getYaw());
SmartDashboard.putNumber("NavX Roll", ahrs.getRoll());


    m_drive.drive(-0.15, 0, 0, false);  
    if (m_tempPitch<m_pitch-8) {
      climbed = true;
    }
    
    //april tag math here

 //from navx

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0,0,0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (climbed == true) {
      if (m_tempPitch>=m_pitch -2) {
        return true;
      }
    }
    return false;
  }
}