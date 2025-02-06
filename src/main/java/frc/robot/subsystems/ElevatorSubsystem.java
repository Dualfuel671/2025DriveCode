package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax motor1;
    private SparkMax motor2;
    private SparkMaxConfig motorConfig1;
    private SparkMaxConfig motorConfig2;
    private SparkLimitSwitch forwardLimitSwitch1;
    private SparkLimitSwitch reverseLimitSwitch1;
    private SparkLimitSwitch forwardLimitSwitch2;
    private SparkLimitSwitch reverseLimitSwitch2;
    private RelativeEncoder encoder1;
    private RelativeEncoder encoder2;

    public ElevatorSubsystem() {
        // Initialize the SPARK MAX motors and get their limit switch and encoder objects for later use.
        motor1 = new SparkMax(1, MotorType.kBrushless);
        motor2 = new SparkMax(2, MotorType.kBrushless);
        forwardLimitSwitch1 = motor1.getForwardLimitSwitch();
        reverseLimitSwitch1 = motor1.getReverseLimitSwitch();
        forwardLimitSwitch2 = motor2.getForwardLimitSwitch();
        reverseLimitSwitch2 = motor2.getReverseLimitSwitch();
        encoder1 = motor1.getEncoder();
        encoder2 = motor2.getEncoder();

        // Create new SPARK MAX configuration objects.
        motorConfig1 = new SparkMaxConfig();
        motorConfig2 = new SparkMaxConfig();

        // Set the idle mode to brake to stop immediately when reaching a limit
        motorConfig1.idleMode(IdleMode.kBrake);
        motorConfig2.idleMode(IdleMode.kBrake);

        // Enable limit switches to stop the motors when they are closed
        motorConfig1.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyOpen)
            .forwardLimitSwitchEnabled(true)
            .reverseLimitSwitchType(Type.kNormallyOpen)
            .reverseLimitSwitchEnabled(true);
        motorConfig2.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyOpen)
            .forwardLimitSwitchEnabled(true)
            .reverseLimitSwitchType(Type.kNormallyOpen)
            .reverseLimitSwitchEnabled(true);

        // Set the soft limits to stop the motors at -50 and 50 rotations
        motorConfig1.softLimit
            .forwardSoftLimit(50)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(-50)
            .reverseSoftLimitEnabled(true);
        motorConfig2.softLimit
            .forwardSoftLimit(50)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(-50)
            .reverseSoftLimitEnabled(true);

        // Set Motor 2 to follow Motor 1
        motorConfig2.follow(motor1);    

        // Apply the configuration to the SPARK MAX motors.
        motor1.configure(motorConfig1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        motor2.configure(motorConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Reset the positions to 0 to start within the range of the soft limits
        encoder1.setPosition(0);
        encoder2.setPosition(0);

        // Initialize dashboard values
        SmartDashboard.setDefaultBoolean("Direction", true);
    }

    public void setMotorSpeed(double speed) {
        motor1.set(speed);
        motor2.set(speed);
    }
    // Create a public method to get the position of the encoder in a different class
    public double getEncoder1Position() {
        return encoder1.getPosition();
    }

    @Override
    public void periodic() {
        // Display data from SPARK onto the dashboard
        SmartDashboard.putBoolean("Forward Limit Reached Motor 1", forwardLimitSwitch1.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Reached Motor 1", reverseLimitSwitch1.isPressed());
        SmartDashboard.putBoolean("Forward Limit Reached Motor 2", forwardLimitSwitch2.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Reached Motor 2", reverseLimitSwitch2.isPressed());
        SmartDashboard.putNumber("Applied Output Motor 1", motor1.getAppliedOutput());
        SmartDashboard.putNumber("Applied Output Motor 2", motor2.getAppliedOutput());
        SmartDashboard.putNumber("Position Motor 1", encoder1.getPosition());
        SmartDashboard.putNumber("Position Motor 2", encoder2.getPosition());
    }
}