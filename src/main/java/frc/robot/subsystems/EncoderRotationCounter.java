// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.Notifier;


// public class EncoderRotationCounter {
//     private final double offset = 0.321;//change if shit
//     public double rotations = 0;
//     private final double WRAP_THRESHHOLD_HIGH_END = 0.9;
//     private final double WRAP_THRESHHOLD_LOW_END = 0.1;
//     private final DutyCycleEncoder encoder = new DutyCycleEncoder(3);
//     private double lastValue = 0;

//     public double getRealRotations()
//     {
//         return -rotations;
//     }
//     public double getRotations() {
//         return -rotations - encoder.get()+offset;
//     }

//     public double getEncoderValue()
//     {
//         return encoder.get();
//     }

//     public void countRotations() {
//         double current = encoder.get();
//         if (lastValue >= WRAP_THRESHHOLD_HIGH_END && current <= WRAP_THRESHHOLD_LOW_END) 
//         {
//             rotations++;
//         } 
//         else if (lastValue <= WRAP_THRESHHOLD_LOW_END && current >= WRAP_THRESHHOLD_HIGH_END)
//         {
//             rotations--;
//         }
//         lastValue = current;
//     }


// }
