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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
    private final ReachSubsystem m_reachSubsystem = new ReachSubsystem();
    private final PitchSubsystem m_pitchSubsystem = new PitchSubsystem();
    private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();


    public void movePitch(double axis){
        m_pitchSubsystem.move(axis);
    }

    public void moveReach(double axis){
        m_reachSubsystem.move(axis);
    }

    public void moveClaw(double axis){
        m_clawSubsystem.grab();
    }

    public void move(double pitch, double reach, int grab) {
        // TODO: Do smart things here...

        m_pitchSubsystem.move(pitch * 0.9);
        m_reachSubsystem.move(reach * 0.9);

        if (grab > -1) {
            m_clawSubsystem.grab();
        } else {
            m_clawSubsystem.release();
        }
    }

    public double getPosPitch(){
        return m_pitchSubsystem.getPos();
    }

    public double getPosReach(){
        return m_reachSubsystem.getPos();
    }

}