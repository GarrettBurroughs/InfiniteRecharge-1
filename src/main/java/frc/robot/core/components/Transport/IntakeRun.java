//PACKAGE//
package frc.robot.core.components.Transport;

//IMPORTS//
import frc.robot.core.utils.StateMachine.*;

class IntakeRun extends StateMachineBase<Transport>{
    public IntakeRun(Transport caller, String useId){super(caller, useId);}

    public StateMachineBase run() {

        StateMachineBase nextState = new IntakeRun(caller, useId);

        if(caller.getHasIntaken()) { 
            nextState = new IntakeEnd(caller, useId); 
            caller.setHasIntaken(false);
        }

        return nextState;

    }
}