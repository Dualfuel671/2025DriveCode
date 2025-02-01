package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Shooter extends SubsystemBase {
    private final TalonFX shooterMotor = new TalonFX(10);
    private final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    public Shooter() {
        shooterConfig.Slot0.kP = 0.1;
        shooterConfig.Slot0.kI = 0.0;
        shooterConfig.Slot0.kD = 0.0;
        // prolly need a kF value if its a feed forward controller
        //shooterConfig.Slot0.kForward = 0.0;
       // shooterConfig.Slot0.integralZone = 0;
        //shooterConfig.Slot0.allowableClosedloopError = 0;
        //shooterConfig.Slot0.maxIntegralAccumulator = 0;
       // shooterConfig.Slot0.closedLoopPeakOutput = 1.0;
       // shooterConfig.Slot0.closedLoopPeriod = 1;
       // shooterConfig.Slot0.closedLoopHysteresis = 0;
       // shooterConfig.Slot0.closedLoopError = 0;
      //  shooterConfig.Slot0.closedLoopRamp = 0;
      //  shooterConfig.Slot0.sensorTerm = TalonFXConfiguration.SensorTerm.SensorTerm_Sum0;
        //shooterConfig.Slot0.filter1 = 0;
        
    
}
    
        public void setShooterSpeed(double speed) {
            shooterMotor.set(speed);
        }
    
        public void stopShooter() {
            shooterMotor.set(0.0);
        }
    }
