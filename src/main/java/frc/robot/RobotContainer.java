// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Test of pull-request

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.button.Trigger;

// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import java.util.List;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();

  // The driver's controller

  // 4765: converted this from xbox to joystick
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  // Arm Controller
  XboxController m_armController = new XboxController(1);

  // 4765: converted this from xbox to joystick
  // XboxController m_driverController = new
  // XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    // 4765: This command will run every iteration of the main scheduler loop - good
    // for driving command

    m_robotDrive.setDefaultCommand(
        // 4765: Controller commands converted for various joysticks
        new RunCommand(
            () -> m_robotDrive.drive(
                m_driverController.getRawAxis(1) * -1,
                m_driverController.getRawAxis(0) * -1,
                m_driverController.getRawAxis(2) * -1,
                false,
                m_driverController.getRawButton(6),
                m_driverController.getRawButton(3)),
            m_robotDrive));

    m_armSubsystem.setDefaultCommand(
        new RunCommand(
            () -> m_armSubsystem.move(
              m_armController.getRawAxis(1),
              m_armController.getRawAxis(3)),
            m_armSubsystem));

    m_clawSubsystem.setDefaultCommand(
    new RunCommand(
        () -> m_clawSubsystem.moveClaw(
          m_armController.getRawButton(6)),
        m_clawSubsystem));
}

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */

  private void configureButtonBindings() {
    // 4765: To be used to assign buttons for things like using the arm and the claw

    // m_armController.a().onTrue(new ClawGrab());

    // Trigger claw_trigger = new JoystickButton(m_armController, 1);
    // claw_trigger.onTrue(m_clawSubsystem.grab());
  

    // new JoystickButton(m_armController, 1)
    // .onFalse(new OpenClaw());

    

  }

  // 4765: Commented out the autonomous code that because it uses odometry which
  // we don't need yet

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //return new AutoBumpScore(m_robotDrive);

    //return new AutonDropBackUp(m_robotDrive, m_armSubsystem);

    return new AutoBumpLeaveBalance(m_robotDrive);

    //return new AutoBumpLeaveAutoBalance(m_robotDrive);

    // if button 1 hit - return exampleCommand

  }
  // // Create config for trajectory
  // TrajectoryConfig config =
  // new TrajectoryConfig(
  // AutoConstants.kMaxSpeedMetersPerSecond,
  // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  // // Add kinematics to ensure max speed is actually obeyed
  // .setKinematics(DriveConstants.kDriveKinematics);

  // // An example trajectory to follow. All units in meters.
  // Trajectory exampleTrajectory =
  // TrajectoryGenerator.generateTrajectory(
  // // Start at the origin facing the +X direction
  // new Pose2d(0, 0, new Rotation2d(0)),
  // // Pass through these two interior waypoints, making an 's' curve path
  // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
  // // End 3 meters straight ahead of where we started, facing forward
  // new Pose2d(3, 0, new Rotation2d(0)),
  // config);

  // var thetaController =
  // new ProfiledPIDController(
  // AutoConstants.kPThetaController, 0, 0,
  // AutoConstants.kThetaControllerConstraints);
  // thetaController.enableContinuousInput(-Math.PI, Math.PI);

  // SwerveControllerCommand swerveControllerCommand =
  // new SwerveControllerCommand(
  // exampleTrajectory,
  // m_robotDrive::getPose, // Functional interface to feed supplier
  // DriveConstants.kDriveKinematics,

  // // Position controllers
  // new PIDController(AutoConstants.kPXController, 0, 0),
  // new PIDController(AutoConstants.kPYController, 0, 0),
  // thetaController,
  // m_robotDrive::setModuleStates,
  // m_robotDrive);

  // // Reset odometry to the starting pose of the trajectory.
  // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

  // // Run path following command, then stop at the end.
  // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
  // false));
  // }
}
