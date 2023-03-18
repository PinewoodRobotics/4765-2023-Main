// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PitchSubsystem extends PIDSubsystem {
    private final TalonSRX m_pitchMotor = new TalonSRX(5);
    private final DutyCycleEncoder m_pitchEncoder = new DutyCycleEncoder (5); // pitch
    private final double upperLimit = ArmConstants.kPitchLevel + 100; // placeholder values
    private final double lowerLimit = ArmConstants.kPitchLevel - 100;
    private final double upMotorCoefficient = 0.2;
    private final double downMotorCoefficient = 0.4;
    private final double calibrationOffset = 0.8803;
    //high 1.13
    //low  0.699
    /** Creates a new PitchSubsystem. */
    
//0.93
//0.5

    public PitchSubsystem() {
        // setup our PID loop
        super(new PIDController(0.2, 0.001, 0.0));
        getController().setTolerance(4/Math.PI);  // 2° tolerance
        SmartDashboard.putNumber("Meas Pitch", m_pitchEncoder.get());
    }

         
    public void move(double axis){
        double encoderPitchValue = m_pitchEncoder.get();
        if (encoderPitchValue <= lowerLimit) {
            if (axis<0) {
                m_pitchMotor.set(ControlMode.PercentOutput, axis * downMotorCoefficient);
            } else {
                m_pitchMotor.set(ControlMode.PercentOutput, 0);
            }
        } else if (encoderPitchValue >= upperLimit) {
            if (axis>0) {
                m_pitchMotor.set(ControlMode.PercentOutput, axis * upMotorCoefficient);
            } else {
                m_pitchMotor.set(ControlMode.PercentOutput, 0);
            }
        } else {
            if (axis<0) {
                m_pitchMotor.set(ControlMode.PercentOutput, axis*downMotorCoefficient);
            } else if (axis>0) {
                m_pitchMotor.set(ControlMode.PercentOutput, axis*upMotorCoefficient);
            }
        }
        SmartDashboard.putNumber("pitch", encoderPitchValue);
        SmartDashboard.putNumber("Meas Pitch", m_pitchEncoder.get());
    }

    public void set(double number) {
        /*
        double currentPitch = m_pitchEncoder.get();
        if (currentPitch > number) {
            m_pitchMotor.set(ControlMode.PercentOutput, -0.3);
        } else if (currentPitch < number) {
            m_pitchMotor.set(ControlMode.PercentOutput, 0.3);
        } */
        setSetpoint(number);
    }

    public double getPos(){
        return m_pitchEncoder.get();
    }

    // Return the arm pitch in radians, where 0.0° is horizontal.
    // Angles above horizontal are positive, and below horizontal are negative.
    public double getPosRadians(){
        // Get the arm pitch - units are encoder ticks
        double encoderTicks = m_pitchEncoder.get();

        // Offset the value so that 0.0° is horizontal
        encoderTicks -= calibrationOffset;

        // Convert to radians
        double radians = encoderTicks * 2.0 * Math.PI / 4096.0;

        // Now adjust for our pully system
        radians *= 32.0; // Number of teeth on the driving pully
        radians /= 42.0; // Number of teeth on the driven pully
        
        return radians;
    }

    // Set arm pitch to specified angle (radians) above horizontal
    public void moveTo(double pitch){
        // Enforce safety limits here
        pitch = Math.max(pitch, calibrationOffset-0.11813);
        pitch = Math.min(pitch, calibrationOffset+0.2497);

        // PID Loop Stuff goes here
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        m_pitchMotor.set(TalonSRXControlMode.Position, output);
    }

    @Override
    protected double getMeasurement() {
        return getPosRadians();
    }
}

