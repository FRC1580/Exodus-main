package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake {

    private final SparkMax IntakeMotor;


    public Intake() {
        IntakeMotor = new SparkMax(62, MotorType.kBrushless);
        IntakeMotor.setInverted(true);
    }

    public Command setSpeed(double speed) {
        return new InstantCommand(() -> IntakeMotor.set(speed));
    }

    
}
