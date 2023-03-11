// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ReachSubsystem extends SubsystemBase {
    private final TalonSRX m_reachMotor = new TalonSRX(9);
    private final DutyCycleEncoder m_reachEncoder = new DutyCycleEncoder (0); // pitch
    private final double upperLimit = 5.37;
    private final double lowerLimit = -4.22;
    private final double motorCoefficient = 0.9;

    public void move(double axis){
        double encoderReachValue = m_reachEncoder.get();
        if (encoderReachValue <= lowerLimit) {
            if (axis<0) {
                m_reachMotor.set(ControlMode.PercentOutput, axis * motorCoefficient);
            } else {
                m_reachMotor.set(ControlMode.PercentOutput, 0);
            }
        } else if (encoderReachValue >= upperLimit) {
            if (axis>0) {
                m_reachMotor.set(ControlMode.PercentOutput, axis * motorCoefficient);
            } else {
                m_reachMotor.set(ControlMode.PercentOutput, 0);
            }
        } else {
            m_reachMotor.set(ControlMode.PercentOutput, axis * motorCoefficient);
        }

    }


    public double getPos(){
        return m_reachEncoder.get();
    }
}