// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import javax.swing.tree.MutableTreeNode;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.networktables.NetworkTableEntry;

public class SwerveModule {
  // 4765: Updated for our hardware
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final CANCoder m_turningEncoder;

  // 4765 TODO: tune P, I, and D for this PID controller
  private final PIDController m_drivePIDController = new PIDController(
      ModuleConstants.kPModuleDriveController,
      0,
      0);

  // 4765 TODO: tune P, I, and D for this PID controller

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      0.0000,
      0.0000,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  // 4765: member variables needed to use tabs in shuffleboard
  private String m_abbreviation;
  private ShuffleboardTab m_tab;
  private GenericEntry m_drivePIDEncoderValue;
  private GenericEntry m_drivePIDWantValue;
  private GenericEntry m_drivePIDOutputValue;
  private GenericEntry m_turnPIDEncoderValue;
  private GenericEntry m_turnPIDWantValue;
  private GenericEntry m_turnPIDOutputValue;
  private GenericEntry m_tempSetDrive;
  private GenericEntry m_tempSetTurn;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel          The channel of the drive motor.
   * @param turningMotorChannel        The channel of the turning motor.
   * @param turningEncoderChannel      The channel of the turning encoder.
   * @param driveEncoderReversed       Whether the drive encoder is reversed.
   * @param turningEncoderReversed     Whether the turning encoder is reversed.
   * @param turningEncoderMagnetOffset The offset of the turning encoder magnet.
   * @param abbreviation               An abbreviatd name to use on Suffleboard
   */
  public SwerveModule(
      // 4765: updated for our hardward config
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed,
      double turningEncoderMagnetOffset,
      String abbreviation) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    // 4765: Note the drive encoder is built in to the spark max and doesn't need
    // its own address
    m_driveEncoder = m_driveMotor.getEncoder();

    m_turningEncoder = new CANCoder(turningEncoderChannel);

    // 4765: It seems more reliable to set these values in software on boot than to
    // rely on writing them to the flash of the CANCoders.
    CANCoderConfiguration config = new CANCoderConfiguration();
    // in radians: 0.3789
    config.magnetOffsetDegrees = turningEncoderMagnetOffset;
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    config.sensorCoefficient = 2 * Math.PI / 4096.0;
    config.unitString = "rad";
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    // config.sensorTimeBase = SensorTimeBase.PerSecond;
    m_turningEncoder.configAllSettings(config);

    // 4765: Might be for odometry
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    //m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

    // 4765 TODO: Not sure that any of our motors should be reversed.
    // Set whether drive encoder should be reversed or not
    //m_driveEncoder.setInverted(driveEncoderReversed);

    // 4765: Not sure if we need this, but this is the factor for our modules'
    // dirving gear ratios
    //m_driveEncoder.setVelocityConversionFactor(0.000148);
    m_driveEncoder.setVelocityConversionFactor(0.000148);

    // Set the distance (in this case, angle) in radians per pulse for the turning
    // encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // 4765: Not sure if this is needed or for odometry
    // m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not
    // 4765: Updated to the correct call for our hardware. Values *seem* to work.
    m_turningEncoder.configSensorDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // 4765: Updated to the correct call for our hardware.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // 4765: Establishes Shuffleboard tab for this module and establishes initial
    // values to show
    m_abbreviation = abbreviation;

    System.out.print(abbreviation + ": " + m_driveEncoder.getVelocityConversionFactor());


    m_tab = Shuffleboard.getTab(m_abbreviation);
    m_drivePIDEncoderValue = m_tab.add(m_abbreviation + " Drive Enc", 0).getEntry();
    m_drivePIDWantValue = m_tab.add(m_abbreviation + " Drive Des", 0).getEntry();
    m_drivePIDOutputValue = m_tab.add(m_abbreviation + " Drive Out", 0).getEntry();
    m_turnPIDEncoderValue = m_tab.add(m_abbreviation + " Turn Enc", 0).getEntry();
    m_turnPIDWantValue = m_tab.add(m_abbreviation + " Turn Des", 0).getEntry();
    m_turnPIDOutputValue = m_tab.add(m_abbreviation + " Turn Output", 0).getEntry();
    m_tempSetDrive = m_tab.add(m_abbreviation + " Temp Drive", 0).getEntry();
    m_tempSetTurn = m_tab.add(m_abbreviation + " Temp Turn", 0).getEntry();
  }

  // 4765: Updated to the correct calls for our hardware
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  // 4765: Seems to be for odometry
  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  // public SwerveModulePosition getPosition() {
  // return new SwerveModulePosition(
  // m_driveEncoder.getDistance(), new
  // Rotation2d(m_turningEncoder.getDistance()));
  // }

  // 4765: updated to the correct calls for our hardware
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    double driveEncoderVelocity = m_driveEncoder.getVelocity();
    double turningEncoderPosition = m_turningEncoder.getAbsolutePosition();

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turningEncoderPosition));

    // Calculate the drive output from the drive PID controller.

    final double driveOutput = m_drivePIDController.calculate(driveEncoderVelocity, state.speedMetersPerSecond);

    // 4765: updates the values in Shuffleboard on this module's tab
    m_drivePIDEncoderValue.setValue(driveEncoderVelocity);
    m_drivePIDWantValue.setValue(state.speedMetersPerSecond);
    m_drivePIDOutputValue.setValue(driveOutput);

    // 4765: Temporary NON-PID output value calculation to bring swerve up without
    // tuning PID

    double tempSetDrive = state.speedMetersPerSecond;
    // 4765: updates the values in Shuffleboard on this module's tab
    m_tempSetDrive.setValue(tempSetDrive);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(turningEncoderPosition, state.angle.getRadians());

    // 4765: updates the values in Shuffleboard on this module's tab
    m_turnPIDEncoderValue.setValue(turningEncoderPosition);
    m_turnPIDWantValue.setValue(state.angle.getRadians());
    m_turnPIDOutputValue.setValue(turnOutput);

    // 4765: Temporary NON-PID output value calculation to bring swerve up without
    // tuning PID
    double tempSetTurn = (turningEncoderPosition - state.angle.getRadians());
    // 4765: updates the values in Shuffleboard on this module's tab
    m_tempSetTurn.setValue(tempSetTurn);

    // 4765: this block uses the temp NON-PID values
    m_driveMotor.set(driveEncoderVelocity + driveOutput);
    m_turningMotor.set(tempSetTurn);

    // 4765: this block uses the PID values with some experimental fudge factors
    // 4765 TODO: Figure this out!!!!
    // m_driveMotor.set(0.5 * 6.75);
    // m_turningMotor.set(turnOutput * 5);
  }

  // 4765: Our encoders don't reset and not sure if we need this

  /** Zeroes all the SwerveModule encoders. */
  // public void resetEncoders() {
  // m_driveEncoder.reset();
  // m_turningEncoder.reset();
  // }
}
