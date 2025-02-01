package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubSystem extends SubsystemBase {
    
    private final TalonFX wristMotor = new TalonFX(9);//TODO: update with correct value
    private final TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    public enum WristPosition {//TODO: set these values correctly
        LOW(0),
        MIDDLE(10),
        HIGH(20);
        private int value;    
      
        private WristPosition(int value) {
          this.value = value;
        }
      
        public int getValue() {
          return value;
        }
    }
    public WristSubSystem(){
        wristConfig.Slot0.kP = 0.1;
        wristConfig.Slot0.kI = 0.0;
        wristConfig.Slot0.kD = 0.0;
        wristMotor.getConfigurator().apply(wristConfig);
    }

    public void setWristPosition(WristPosition state) {
        wristMotor.setControl(m_request.withPosition(state.getValue()));
    }
}
