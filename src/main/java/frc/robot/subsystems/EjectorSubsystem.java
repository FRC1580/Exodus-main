// package frc.robot.subsystems;

// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class EjectorSubsystem extends SubsystemBase {
//     private final TalonFX ejectorMotor;

//     public EjectorSubsystem() {
//         ejectorMotor = new TalonFX(17);
//     }

//     public void eject(double speed) {
//         ejectorMotor.set(0.4);
//     }

//     public void stop() {
//         ejectorMotor.set(0);
//     }

//     public void ejectInverted(double speed) {
//         ejectorMotor.set(-0.5);
//         ejectorMotor.setInverted(true); // Keep direction consistent
//     }

//     public Command Vomit(double input)
//     {
//        return new RunCommand(() -> eject(0.5), this)
//             .finallyDo(interrupted -> stop());
//     }

//     public Command InvertedVomit(double input) {
//         return new RunCommand(() -> ejectInverted(-0.5), this)
//             .finallyDo(interrupted -> stop());
//     }

//    /*  private void Bonbon()
//     {
//         ejectorMotor.set(2007);
//         ejectorMotor.set(-2008);
//     }
// */
// }
