package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import javax.swing.plaf.metal.MetalBorders.PopupMenuBorder;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


public class BalanceSubsystem extends SubsystemBase {

    static final double offBalanceThreshold = 6;
    static final double onBalanceThreshold  = 2;

    private final AHRS ahrs = new AHRS(SPI.Port.kMXP);
    private DriveSubsystem m_robotDrive;

    public void balance(){

        double pitch = ahrs.getPitch(); // tilting on the x axis 
        double roll = ahrs.getRoll(); // tilting on the y axis 

        boolean autoBalanceXMode = false;
        boolean autoBalanceYMode = false;

        double xSpeed = 0;
        double ySpeed = 0;
        
        while ((Math.abs(pitch) >= Math.abs(offBalanceThreshold)) || (Math.abs(roll) >= Math.abs(offBalanceThreshold))){

            xSpeed = 0;
            ySpeed = 0;

            autoBalanceXMode = determineAutoBalance(autoBalanceXMode, pitch);
            autoBalanceYMode = determineAutoBalance(autoBalanceYMode, roll);
            
            if ( autoBalanceXMode ) {
                double pitchAngleRadians = pitch * (Math.PI / 180.0);
                xSpeed = Math.sin(pitchAngleRadians) * -1;
            }
            if ( autoBalanceYMode ) {
                double rollAngleRadians = roll * (Math.PI / 180.0);
                ySpeed = Math.sin(rollAngleRadians) * -1;
            }

            m_robotDrive.drive(xSpeed, ySpeed, 0,false);

            Timer.delay(0.005);     // wait for a motor update time

            pitch = ahrs.getPitch();
            roll = ahrs.getRoll();
        }
    }

    private boolean determineAutoBalance(boolean currentMode, double axis ) {

        // checks to see which axis it needs to correct
        if ( !currentMode && (Math.abs(axis) >= Math.abs(offBalanceThreshold))) {
            return true;
        }
        else if ( currentMode && (Math.abs(axis) <= Math.abs(onBalanceThreshold))) {
            return false;
        } 

        return currentMode;
    }

    public BalanceSubsystem(DriveSubsystem drive) {
        m_robotDrive = drive;

    }
}