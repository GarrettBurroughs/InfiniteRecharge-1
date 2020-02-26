package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.Shoot;

public class AutoDrive extends SequentialCommandGroup {

    // Test Mode that will drive a distance of 5 "units" and fire payload
    public AutoDrive() {
        addCommands(
                new DriveStraight(5),
                new Shoot()
        );
    }
}
