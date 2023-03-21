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
import com.revrobotics.SparkMaxPIDController;

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
  private final RelativeEncoder m_sparkMaxDriveEncoder;
  private SparkMaxPIDController m_sparkMaxDrivePIDController;

  
  private final CANSparkMax m_turnMotor;
  private final RelativeEncoder m_sparkMaxTurnEncoder;
  private SparkMaxPIDController m_sparkMaxTurnPIDController;

  private final CANCoder m_canCoderEncoder;

  private double m_loopCount = 0;

  private double m_continuousFactor = 0;
  
  public double
  kDriveP,
  kDriveI,
  kDriveD,
  kDriveIZ,
  kDriveFF,
  kDriveMaxOutput, 
  kDriveMinOutput,
  kDriveMaxRPM;

  public double
  kTurnP,
  kTurnI,
  kTurnD,
  kTurnIZ,
  kTurnFF,
  kTurnMaxOutput,
  kTurnMinOutput;

  public GenericEntry 
  sb_kDriveP,
  sb_kDriveI,
  sb_kDriveD,
  sb_kDriveIZ,
  sb_kDriveFF,
  sb_kDriveMaxOutput,
  sb_kDriveMinOutput,
  sb_kDriveMaxRPM,
  sb_manualRPM,
  sb_driveDesired,
  sb_driveDesiredProc;
  
  public GenericEntry 
  sb_kTurnP,
  sb_kTurnI,
  sb_kTurnD,
  sb_kTurnIZ,
  sb_kTurnFF,
  sb_kTurnMaxOutput,
  sb_kTurnMinOutput,
  sb_manualRotations,
  sb_manualDegrees,
  sb_turnDesired,
  sb_turnDesiredProc,
  sb_turnSparkStart,
  sb_turnCanCStart,
  sb_turnCanCFactor,
  sb_turnNewPosition,
  sb_turnSparkNew,
  sb_turnCanCLive,
  sb_turnCanCPosLive,
  sb_turnSetPoint,
  sb_turnProcessVariable,
  sb_turnSparkLive;

  // // 4765 TODO: tune P, I, and D for this PID controller
  // private final PIDController m_drivePIDController = new PIDController(
  //     ModuleConstants.kPModuleDriveController,
  //     0,
  //     0);

  // // 4765 TODO: tune P, I, and D for this PID controller

  // private final PIDController m_turningPIDController = new PIDController(
  //     ModuleConstants.kPModuleTurningController,
  //     0,
  //     0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  // private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
  //     ModuleConstants.kPModuleTurningController,
  //     0.0000,
  //     0.000,
  //     new TrapezoidProfile.Constraints(
  //         ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
  //         ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  // 4765: member variables needed to use tabs in shuffleboard
  private String m_abbreviation;
  private ShuffleboardTab m_tab;

  // private GenericEntry m_drivePIDEncoderValue;
  // private GenericEntry m_drivePIDWantValue;
  // private GenericEntry m_drivePIDOutputValue;
  // private GenericEntry m_turnPIDEncoderValue;
  // private GenericEntry m_turnPIDWantValue;
  // private GenericEntry m_turnPIDOutputValue;
  // private GenericEntry m_tempSetDrive;
  // private GenericEntry m_tempSetTurn;
  // private GenericEntry m_turnPIDatSetpoint;
  // private GenericEntry m_turnPIDPositionTolerance;
  // private GenericEntry m_driveDesiredState;
  // private GenericEntry m_turnDesiredState;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel          The channel of the drive motor.
   * @param turnMotorChannel        The channel of the turning motor.
   * @param turnEncoderChannel      The channel of the turning encoder.
   * @param driveEncoderReversed       Whether the drive encoder is reversed.
   * @param turnEncoderReversed     Whether the turning encoder is reversed.
   * @param turnEncoderMagnetOffset The offset of the turning encoder magnet.
   * @param abbreviation               An abbreviatd name to use on Suffleboard
   */
  public SwerveModule(
      // 4765: updated for our hardward config
      int driveMotorChannel,
      int turnMotorChannel,
      int canCoderEncoderChannel,
      boolean driveEncoderReversed,
      boolean canCoderReversed,
      double canCoderMagnetOffset,
      String abbreviation) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_sparkMaxDriveEncoder = m_driveMotor.getEncoder();
    m_sparkMaxDrivePIDController = m_driveMotor.getPIDController();

  //m_sparkMaxDriveEncoder.setVelocityConversionFactor(0.000148);

    m_turnMotor = new CANSparkMax(turnMotorChannel, MotorType.kBrushless);
    m_turnMotor.setInverted(true);
    m_sparkMaxTurnEncoder = m_turnMotor.getEncoder();
    m_sparkMaxTurnPIDController = m_turnMotor.getPIDController();


    //m_sparkMaxTurnEncoder.setPositionConversionFactor(1);

    m_canCoderEncoder = new CANCoder(canCoderEncoderChannel);

    // 4765: It seems more reliable to set these values in software on boot than to
    // rely on writing them to the flash of the CANCoders.
    CANCoderConfiguration config = new CANCoderConfiguration();
    // in radians: 0.3789
    config.magnetOffsetDegrees = canCoderMagnetOffset;
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    config.sensorCoefficient = 2 * Math.PI / 4096.0;
    config.unitString = "rad";
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorDirection = canCoderReversed;
    // config.sensorTimeBase = SensorTimeBase.PerSecond;

   // m_canCoderEncoder.configAllSettings(config);

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

    // Set the distance (in this case, angle) in radians per pulse for the turning
    // encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // 4765: Not sure if this is needed or for odometry
    // m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not
    // 4765: Updated to the correct call for our hardware. Values *seem* to work.
    //m_turningEncoder.configSensorDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // 4765: Updated to the correct call for our hardware.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    //m_turningPIDController.setTolerance(0.01, m_turningPIDController.getVelocityTolerance());
    

    //m_turnMotor.restoreFactoryDefaults();
    m_driveMotor.restoreFactoryDefaults();

    // PID coefficients
    
    kDriveP = 6e-5;
    kDriveI = 0;
    kDriveD = 0;
    kDriveIZ = 0;
    kDriveFF = 0.000015;
    kDriveMaxOutput = 1;
    kDriveMinOutput = -1;

    kDriveMaxRPM = 5700;

    kTurnP = 0.1;
    kTurnI = 1e-4;
    kTurnD = 1;
    kTurnIZ = 0;
    kTurnFF = 0;
    kTurnMaxOutput = 1;
    kTurnMinOutput = -1;

    // set PID coefficients
    m_sparkMaxDrivePIDController.setP(kDriveP);
    m_sparkMaxDrivePIDController.setI(kDriveI);
    m_sparkMaxDrivePIDController.setD(kDriveD);
    m_sparkMaxDrivePIDController.setIZone(kDriveIZ);
    m_sparkMaxDrivePIDController.setFF(kDriveFF);
    m_sparkMaxDrivePIDController.setOutputRange(kDriveMinOutput, kDriveMaxOutput);

    m_sparkMaxTurnPIDController.setP(kTurnP);
    m_sparkMaxTurnPIDController.setI(kTurnI);
    m_sparkMaxTurnPIDController.setD(kTurnD);
    m_sparkMaxTurnPIDController.setIZone(kTurnIZ);
    m_sparkMaxTurnPIDController.setFF(kTurnFF);
    m_sparkMaxTurnPIDController.setOutputRange(kTurnMinOutput, kTurnMaxOutput);

    // 4765: Establishes Shuffleboard tab for this module and establishes initial
    // values to show
    m_abbreviation = abbreviation;

    m_tab = Shuffleboard.getTab(m_abbreviation);
    
    Shuffleboard.selectTab(abbreviation);

    sb_kDriveP = m_tab.add("D01: kDriveP", kDriveP).getEntry();
    sb_kDriveI = m_tab.add("D02: kDriveI", kDriveI).getEntry();
    sb_kDriveD = m_tab.add("D03: kDriveD", kDriveD).getEntry();
    sb_kDriveIZ = m_tab.add("D04: kDriveIZ", kDriveIZ).getEntry();
    sb_kDriveFF = m_tab.add("D05: kDriveFF", kDriveFF).getEntry();
    sb_kDriveMaxOutput = m_tab.add("D06: kDriveMaxOutput", kDriveMaxOutput).getEntry();
    sb_kDriveMinOutput = m_tab.add("D07: kDriveMinOutput", kDriveMinOutput).getEntry();
    sb_manualRPM = m_tab.add("D08: Manual RPM", 0).getEntry();
    sb_driveDesired = m_tab.add("D09: Desired", 0).getEntry();
    sb_driveDesiredProc = m_tab.add("D10: Desired Proc", 0).getEntry();

    sb_kTurnP = m_tab.add("T01: kTurnP", kTurnP).getEntry();
    sb_kTurnI = m_tab.add("T02: kTurnI",kTurnI).getEntry();
    sb_kTurnD = m_tab.add("T03: kTurnD", kTurnD).getEntry();
    sb_kTurnIZ = m_tab.add("T04: kTurnIZ", kTurnIZ).getEntry();
    sb_kTurnFF = m_tab.add("T05: kTurnFF", kTurnFF).getEntry();
    sb_kTurnMaxOutput = m_tab.add("T06: kTurnMaxOutput", kTurnMaxOutput).getEntry();
    sb_kTurnMinOutput = m_tab.add("T07: kTurnMinOutput", kTurnMinOutput).getEntry();
   
    sb_manualRotations = m_tab.add("T08: Manual Rotations", 0).getEntry();
    sb_manualDegrees = m_tab.add("T09: Manual Rotations", 0).getEntry();
    sb_turnDesired = m_tab.add("T10: Desired", 0).getEntry();
    sb_turnDesiredProc = m_tab.add("T11: Desired Proc", 0).getEntry();
    
    sb_turnSparkStart = m_tab.add(
      "T12: Spark Start",
      m_sparkMaxTurnEncoder.getPosition()).getEntry();
   
      sb_turnCanCStart = m_tab.add(
        "T13: CanC Start", 
        m_canCoderEncoder.getAbsolutePosition()).getEntry();

    sb_turnCanCFactor = m_tab.add(
      "T14: CanC Factor", 
      (m_canCoderEncoder.getAbsolutePosition()*(180/Math.PI))/(360/(150/7))).getEntry();
      
      sb_turnNewPosition = m_tab.add(
        "T15: New Position", 
        sb_turnCanCFactor.getDouble(0)).getEntry();

        double postion = sb_turnCanCFactor.getDouble(0);

        if ( postion < 0 ) {
            postion = postion;
        }

    m_sparkMaxTurnEncoder.setPosition(postion);

    sb_turnSparkNew = m_tab.add(
      "T16: Spark New", 
      m_sparkMaxTurnEncoder.getPosition()).getEntry();
    
      sb_turnCanCLive = m_tab.add(
        "T17: CanC Live", 
        m_canCoderEncoder.getAbsolutePosition()).getEntry();

        sb_turnCanCPosLive = m_tab.add(
          "T17b: CanC Pos Live", 
          m_canCoderEncoder.getPosition()).getEntry();

    sb_turnSetPoint = m_tab.add("T18: Set Point", 0).getEntry();
    sb_turnProcessVariable = m_tab.add("T19: Proc Vari", 0).getEntry();
    sb_turnSparkLive = m_tab.add("T20: Spark Live", 0).getEntry();

  }
  
    // 4765: Updated to the correct calls for our hardware
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  // public SwerveModuleState getState() {
  //   return new SwerveModuleState(
  //     // TODO: Fix this!
  //       m_sparkMaxDriveEncoder.getVelocity(), 
  //       new Rotation2d(((Math.PI * 2)/(150/7)) * m_sparkMaxDriveEncoder.getPosition()));
  // }

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


public double makeContinous(double originalRadians, double newRadians) {

double newContinuousFactor = m_continuousFactor;

// if (Math.abs(originalRadians - newRadians) < 1.5708) {
//       if (originalRadians - newRadians) >
// }

return newContinuousFactor;
}




  // 4765: updated to the correct calls for our hardware
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    // CAREFUL! POSTION OR ABSOLUTE

double sparkMaxEncoderPosition = m_sparkMaxTurnEncoder.getPosition();

sb_turnSparkNew.setValue(sparkMaxEncoderPosition);

sb_turnCanCLive.setValue(m_canCoderEncoder.getAbsolutePosition());
sb_turnCanCPosLive.setValue(m_canCoderEncoder.getPosition());


double driveP = sb_kDriveP.getDouble(0);
double driveI = sb_kDriveI.getDouble(0);
double driveD = sb_kDriveD.getDouble(0);
double driveIZ = sb_kDriveIZ.getDouble(0);
double driveFF = sb_kDriveFF.getDouble(0);
double driveMaxOutput = sb_kDriveMaxOutput.getDouble(0);
double driveMinOutput = sb_kDriveMinOutput.getDouble(0);

double turnP = sb_kTurnP.getDouble(0);
double turnI = sb_kTurnI.getDouble(0);
double turnD = sb_kTurnD.getDouble(0);
double turnIZ = sb_kTurnIZ.getDouble(0);
double turnFF = sb_kTurnFF.getDouble(0);
double turnMaxOutput = sb_kTurnMaxOutput.getDouble(0);
double turnMinOutput = sb_kTurnMinOutput.getDouble(0);

// if PID coefficients on SmartDashboard have changed, write new values to
    // controller

    if ((driveP != kDriveP)) {
      m_sparkMaxDrivePIDController.setP(driveP);
      kDriveP = driveP;
    }
    if ((driveI != kDriveI)) {
      m_sparkMaxDrivePIDController.setI(driveI);
      kDriveI = driveI;
    }
    if ((driveD != kDriveD)) {
      m_sparkMaxDrivePIDController.setD(driveD);
      kDriveD = driveD;
    }
    if ((driveIZ != kDriveIZ)) {
      m_sparkMaxDrivePIDController.setIZone(driveIZ);
      kDriveIZ = driveIZ;
    }
    if ((driveFF != kDriveFF)) {
      m_sparkMaxDrivePIDController.setFF(driveFF);
      kDriveFF = driveFF;
    }
    if ((driveMaxOutput != kDriveMaxOutput) || (driveMinOutput != kDriveMinOutput)) {
      m_sparkMaxDrivePIDController.setOutputRange(driveMinOutput, driveMaxOutput);
      kDriveMinOutput = driveMinOutput;
      kDriveMaxOutput = driveMaxOutput;
    }

    if ((turnP != kTurnP)) {
      m_sparkMaxTurnPIDController.setP(turnP);
      kTurnP = turnP;
    }
    if ((turnI != kTurnI)) {
      m_sparkMaxTurnPIDController.setI(turnI);
      kTurnI = turnI;
    }
    if ((turnD != kTurnD)) {
      m_sparkMaxTurnPIDController.setD(turnD);
      kTurnD = turnD;
    }
    if ((turnIZ != kTurnIZ)) {
      m_sparkMaxTurnPIDController.setIZone(turnIZ);
      kTurnIZ = turnIZ;
    }
    if ((turnFF != kTurnFF)) {
      m_sparkMaxTurnPIDController.setFF(turnFF);
      kTurnFF = turnFF;
    }
    if ((turnMaxOutput != kTurnMaxOutput) || (turnMinOutput != kTurnMinOutput)) {
      m_sparkMaxTurnPIDController.setOutputRange(turnMinOutput, turnMaxOutput);
      kTurnMinOutput = turnMinOutput;
      kTurnMaxOutput = turnMaxOutput;
    }
 
    sb_driveDesired.setValue(desiredState.speedMetersPerSecond);
    sb_turnDesired.setValue(desiredState.angle.getRadians());

    
double manualRPM = sb_manualRPM.getDouble(0);
double driveDesired = sb_driveDesired.getDouble(0);
double driveDesiredProc = sb_driveDesiredProc.getDouble(0);

double manualRotations = sb_manualRotations.getDouble(0);
double manualDegrees = sb_manualDegrees.getDouble(0);
double turnDesired = sb_turnDesired.getDouble(0);
double turnDesiredProc = sb_turnDesiredProc.getDouble(0);
double turnSparkStart = sb_turnSparkStart.getDouble(0);
double turnCanCStart = sb_turnCanCStart.getDouble(0);
double turnCanCFactor = sb_turnCanCFactor.getDouble(0);
double turnNewPosition = sb_turnNewPosition.getDouble(0);
double turnSparkNew = sb_turnSparkNew.getDouble(0);
double turnCanCLive = sb_turnCanCLive.getDouble(0);
double turnCanCPosLive = sb_turnCanCPosLive.getDouble(0);




    //double driveEncoderVelocity = m_driveEncoder.getVelocity();
    //double turningEncoderPosition = m_turningEncoder.getAbsolutePosition();
    
    // Optimize the reference state to avoid spinning further than 90 degrees

    

    sb_turnSparkLive.setValue(((Math.PI * 2)/(150/7)) * sparkMaxEncoderPosition);

    SwerveModuleState state = SwerveModuleState.optimize(
      desiredState, 

      new Rotation2d(sparkMaxEncoderPosition*((Math.PI * 2)/(150/7))));

    sb_driveDesiredProc.setValue(state.speedMetersPerSecond);
    sb_turnDesiredProc.setValue(state.angle.getRadians());

    // Calculate the drive output from the drive PID controller.

    //final double driveOutput = m_drivePIDController.calculate(driveEncoderVelocity, state.speedMetersPerSecond);

    // 4765: updates the values in Shuffleboard on this module's tab
    // m_drivePIDEncoderValue.setValue(driveEncoderVelocity);
    // m_drivePIDWantValue.setValue(state.speedMetersPerSecond);
    // m_drivePIDOutputValue.setValue(driveOutput);

    // 4765: Temporary NON-PID output value calculation to bring swerve up without
    // tuning PID

    // double tempSetDrive = state.speedMetersPerSecond;
    // // 4765: updates the values in Shuffleboard on this module's tab
    // m_tempSetDrive.setValue(tempSetDrive);

    // Calculate the turning motor output from the turning PID controller.
    //final double turnOutput = m_turningPIDController.calculate(turningEncoderPosition, state.angle.getRadians());
     

    //m_sparkMaxDrivePIDController.setReference(1000, CANSparkMax.ControlType.kVelocity);

    m_sparkMaxDrivePIDController.setReference(state.speedMetersPerSecond * kDriveMaxRPM * 3, CANSparkMax.ControlType.kVelocity);

    //m_driveMotor.set(state.speedMetersPerSecond);



// m_continuousFactor = makeContinous(
//   sparkMaxEncoderPosition*((Math.PI * 2)/(150/7)), 
//   state.angle.getRadians());



    double rotations = (state.angle.getRadians()/((Math.PI * 2)/(150/7)));

    m_sparkMaxTurnPIDController.setReference(rotations, CANSparkMax.ControlType.kPosition);

    sb_turnSetPoint.setValue(rotations);
    sb_turnProcessVariable.setValue(m_sparkMaxTurnEncoder.getPosition());
    //sb_turnSparkLive.setValue(m_sparkMaxTurnEncoder.getPosition());

    // 4765: updates the values in Shuffleboard on this module's tab
    // m_turnPIDEncoderValue.setValue(turningEncoderPosition);
    // m_turnPIDWantValue.setValue(state.angle.getRadians());
    // m_turnPIDOutputValue.setValue(turnOutput);
    // m_turnPIDatSetpoint.setValue(m_turningPIDController.atSetpoint());
    // m_turnPIDPositionTolerance.setValue(m_turningPIDController.getPositionTolerance());

    // 4765: Temporary NON-PID output value calculation to bring swerve up without
    // tuning PID
    // double tempSetTurn = (turningEncoderPosition - state.angle.getRadians());
    // // 4765: updates the values in Shuffleboard on this module's tab 
    // m_tempSetTurn.setValue(tempSetTurn);

    // 4765: this block uses the temp NON-PID values
    //m_driveMotor.set(tempSetDrive * 6.75);
    //m_turningMotor.set(tempSetTurn);

    // 4765: this block uses the PID values with some experimental fudge factors
    // 4765 TODO: Figure this out!!!!
    
    //m_driveMotor.set(driveEncoderVelocity + driveOutput);


// if (m_turningPIDController.atSetpoint()) {
//   m_turningMotor.set(0);
// } else {
//   m_turningMotor.set(turnOutput);
//}



  }

  // 4765: Our encoders don't reset and not sure if we need this

  /** Zeroes all the SwerveModule encoders. */
  // public void resetEncoders() {
  // m_driveEncoder.reset();
  // m_turningEncoder.reset();
  // }
}
