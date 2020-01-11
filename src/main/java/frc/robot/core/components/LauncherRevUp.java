package frc.robot.core.components;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

public class LauncherRevUp implements LauncherState {
    private final SpeedController motor;
    private final Encoder encoder;

    public LauncherRevUp(SpeedController motor, Encoder encoder){
        this.motor = motor;
        this.encoder = encoder;
    }


    @Override
    public LauncherState run() {
        motor.set(1);
        if(encoder.getRate() > 10){
            LauncherState state = new LauncherShoot(null, 1, 3);// need to edit based on other variables
            state = state.run();
        }
        return this;
    }

}