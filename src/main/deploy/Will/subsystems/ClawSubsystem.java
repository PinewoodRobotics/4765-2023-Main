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

public class ClawSubsystem extends SubsystemBase {
    private final TalonSRX m_clawMotor = new TalonSRX(6);

    public ClawSubsystem() {
        m_clawMotor.configContinuousCurrentLimit(3);
        m_clawMotor.configPeakCurrentLimit(3);
    }

    public void grab(){
        m_clawMotor.set(ControlMode.PercentOutput, -0.5);
    }

    public void release(){
        m_clawMotor.set(ControlMode.PercentOutput, 0.2);
    }
}