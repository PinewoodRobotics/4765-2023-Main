// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ClawGrab;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClawSubsystem extends SubsystemBase {
    private final TalonSRX m_clawMotor = new TalonSRX(6);

    public ClawSubsystem() {
        m_clawMotor.configContinuousCurrentLimit(8);
        m_clawMotor.configPeakCurrentLimit(8);
        m_clawMotor.enableCurrentLimit(true);
        //SmartDashboard.putBoolean("claw grabbing", false);
    }

    public void moveClaw(){
        if (SmartDashboard.getBoolean("claw grabbing", false) == true) {
            grab();
        } else {
            release();
        }
    }

    public void moveClaw(boolean aPressed) {
        if (aPressed)
        { grab(); } 
        else {
            release();
        }
    }


    public void grab(){
        m_clawMotor.set(ControlMode.PercentOutput, -0.9);
    }

    public void release(){
        m_clawMotor.set(ControlMode.PercentOutput, 0.2);
    }
}