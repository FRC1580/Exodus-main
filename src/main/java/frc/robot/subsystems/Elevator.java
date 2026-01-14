// package frc.robot.subsystems;


// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Constants.OperatorConstants;

// public class Elevator extends SubsystemBase {
//     private final TalonFX slaveMotor;
//     private final TalonFX masterMotor;
//     private final PIDController pidController = new PIDController(0.48,0, 0);
//     public final EncoderRotationCounter encoderRotationCounter = new EncoderRotationCounter();
//     private final double maxRoations = 3.05;
//     private final double errorRange = 0.02;
//     private double krakenEncoderValue = 0;
//     private double DesiredRotations = 0;
//     private final double Angle = 80;

//     public Elevator() {
//         masterMotor = new TalonFX(12);
//         slaveMotor = new TalonFX(13);
//         masterMotor.setNeutralMode(NeutralModeValue.Brake);
//         slaveMotor.setNeutralMode(NeutralModeValue.Brake);
//         DesiredRotations = getAbsPos();
//         setDefaultCommand(new RunCommand(this::update, this));
//     }


//     public void StopSima()
//     {
//         setSpeed(OperatorConstants.constPower);
//     }
//     public void setSpeed(double speed)
//     {
//         masterMotor.set(speed);
//         slaveMotor.set(-speed);
//     }

//     public double getAbsPos() 
//     {
//         return encoderRotationCounter.getRotations();
//     }

    

//     public void setPos(double pos) 
//     {
//         pidController.setSetpoint(pos);
//         double output = pidController.calculate(getAbsPos());
//         setSpeed(output+OperatorConstants.constPower);
//     }

    
//     public void update() 
//     {
//         if (getAbsPos() >= maxRoations-0.1) {
//             setSpeed(Constants.OperatorConstants.constPower);
//             return;
//         }

//         if (Math.abs(DesiredRotations - getAbsPos()) < errorRange) {
//             StopSima();
//             Constants.OperatorConstants.correctHeight = true;

//         }  
//         else
//         {
//             setPos(DesiredRotations);
//             Constants.OperatorConstants.correctHeight = false;
//         }
        
//         encoderRotationCounter.countRotations();

//         System.out.println("Elevator Absolute Position: " + getAbsPos());
//         System.out.println("rotation not shit " + encoderRotationCounter.getRealRotations());
//         System.out.println("encoder shit " + encoderRotationCounter.getEncoderValue());
//         System.out.println(krakenEncoderValue);
//     }


//     public Command moveTo(double rotations) 
//     {
//         DesiredRotations = Math.max(0, Math.min(rotations, maxRoations-0.1)); // Clamps between 0 and 1.3
//         if (rotations == OperatorConstants.GROUND)
//         {
//             OperatorConstants.speedLimiter = 1;
//         }
//         else if (rotations == OperatorConstants.FIRST_LEVEL)
//         {
//             OperatorConstants.speedLimiter = 0.5;
//         }
//         else if (rotations == OperatorConstants.SECOND_LEVEL)
//         {
//             OperatorConstants.speedLimiter = 0.25;
//         }
//         else if (rotations == OperatorConstants.THIRD_LEVEL)
//         {
//             OperatorConstants.speedLimiter = 0.125;
//         }
//         else if (rotations == OperatorConstants.FOURTH_LEVEL)
//         {
//             OperatorConstants.speedLimiter = 0;
//         }

    
//         return new RunCommand(() -> {
//             DesiredRotations = rotations;
//             setPos(DesiredRotations);
//         }, this).until(() -> Math.abs(DesiredRotations - getAbsPos()) < 0.1);
//     }
// }