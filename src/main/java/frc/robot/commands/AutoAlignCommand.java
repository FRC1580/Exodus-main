// File: AutoAlignCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlignCommand extends Command {
    // private final Vision limelight;
    // // Set desired target area (tune this based on your measurements)
    // private final double desiredTargetArea;

    // public AutoAlignCommand(Vision limelight, double desiredTargetArea) {
    //     this.limelight = limelight;
        
    //     this.desiredTargetArea = desiredTargetArea;
    //     addRequirements(limelight);
    // }

    @Override
    public void initialize() {
        // Turn on the Limelight LEDs for vision processing
        // this.limelight.setLEDMode(3);// Force on
    }

    @Override
    public void execute() {
        // if (limelight.hasTarget()) {
        //     // double steerCmd = limelight.calculateSteer();
        //     // double driveCmd = limelight.calculateDrive(desiredTargetArea);
            
        // } 
        // else 
        // {
            
        // }
    }

    @Override
    public void end(boolean interrupted) {
        // Optionally, turn off LEDs or revert to default settings when alignment stops
        // limelight.setLEDMode(1); // Force off
        // drivetrain.arcadeDrive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        // Run until canceled or add a termination condition (e.g., when error is within threshold)
        return false;
    }
}
