package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Shooter {

    private final SparkMax shooterMotor;


    public Shooter() {
        shooterMotor = new SparkMax(62, MotorType.kBrushless);
        shooterMotor.setInverted(false);
    }

    public void setSpeed(double speed) {
        shooterMotor.set(speed);
    }

    
}
