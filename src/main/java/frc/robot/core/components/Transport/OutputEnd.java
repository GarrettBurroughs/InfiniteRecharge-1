//PACKAGE//
package frc.robot.core.components.Transport;

//IMPORTS//
import edu.wpi.first.wpilibj.SpeedController;

class OutputEnd implements TransportStateMachine {
    public TransportStateMachine run(Transport transport) {

        TransportStateMachine nextState = new TransportRest();

        for (SpeedController motor: transport.getOutputMotors()) {

            motor.stopMotor(); //CHANGE CONSTANT//

        }

        return nextState;

    }
}