package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
<<<<<<< HEAD

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

=======
// need beam break 
>>>>>>> 2affb3831071b2c03e9c4128468fb62bdca91dda



public class RampSubSystem extends SubsystemBase {
    private final TalonFX rampMotor = new TalonFX(10);//TODO: update with correct value
    private final TalonFXConfiguration rampConfig = new TalonFXConfiguration();
<<<<<<< HEAD
    DigitalInput RampBeans = new DigitalInput(2);
=======
>>>>>>> 2affb3831071b2c03e9c4128468fb62bdca91dda

    public RampSubSystem() {
        rampConfig.Slot0.kP = 0.1;
        rampConfig.Slot0.kI = 0.0;
        rampConfig.Slot0.kD = 0.0;
        rampMotor.getConfigurator().apply(rampConfig);
        // rampMotor need a kF value if its a feed forward controller
        //rampConfig.Slot0.kForward = 0.0;
       // rampConfig.Slot0.integralZone = 0;
        //rampConfig.Slot0.allowableClosedloopError = 0;
        //rampConfig.Slot0.maxIntegralAccumulator = 0;
       // rampConfig.Slot0.closedLoopPeakOutput = 1.0;
       // rampConfig.Slot0.closedLoopPeriod = 1;
       // rampConfig.Slot0.closedLoopHysteresis = 0;
       // rampConfig.Slot0.closedLoopError = 0;
      //  rampConfig.Slot0.closedLoopRamp = 0;
      //  rampConfig.Slot0.sensorTerm = TalonFXConfiguration.SensorTerm.SensorTerm_Sum0;
        //rampConfig.Slot0.filter1 = 0;
        
        rampMotor.setNeutralMode(NeutralModeValue.Brake);
        //rampMotor.setNeutralMode(NeutralModeValue.Coast);
        rampMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setRampSpeed(double speed) {
        rampMotor.set(speed);
    }

    public void stopRamp() {
        rampMotor.set(0.0);
    }
<<<<<<< HEAD

    //Method to get value of ramp beans
    public boolean RampBeans() {
        return !RampBeans.get();
    }

    //puts ramp beans on smartdashboard
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Ramp beans" , RampBeans());
    }
=======
>>>>>>> 2affb3831071b2c03e9c4128468fb62bdca91dda
}
