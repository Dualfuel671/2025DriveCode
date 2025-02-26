package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.DigitalInput;
=======
>>>>>>> 2affb3831071b2c03e9c4128468fb62bdca91dda

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


<<<<<<< HEAD
public class Shooter extends SubsystemBase {
    private final TalonFX shooterMotor = new TalonFX(11);
    private final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    //make shooter beans
    DigitalInput beamBreak = new DigitalInput(1);
    //make Algae Limit Switch (non-existant at the moment)
    DigitalInput AlgaeLimitSwitch = new DigitalInput(5);

    public Shooter() {

=======

public class Shooter extends SubsystemBase {
    private final TalonFX shooterMotor = new TalonFX(11);
    private final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    public Shooter() {
>>>>>>> 2affb3831071b2c03e9c4128468fb62bdca91dda
        shooterConfig.Slot0.kP = 0.1;
        shooterConfig.Slot0.kI = 0.0;
        shooterConfig.Slot0.kD = 0.0;
        shooterMotor.getConfigurator().apply(shooterConfig);
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
<<<<<<< HEAD

    //tells us when shooter bean is tripped
    public boolean isCoralPresent() {
        return !beamBreak.get();
    }

    //tells us when algae limit switch is tripped
    public boolean isAlgaePresent() {
        return !AlgaeLimitSwitch.get();
    }
    
    //puts coral/Algae status on smartDashboard
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("is coral in shooter" , isCoralPresent());
        SmartDashboard.putBoolean("is Algae in shooter" , isAlgaePresent());
    }
=======
>>>>>>> 2affb3831071b2c03e9c4128468fb62bdca91dda
}
