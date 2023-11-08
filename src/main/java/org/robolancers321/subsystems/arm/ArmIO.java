package org.robolancers321.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs{
        public double anchorPosition = 0.0;
        public double floatingPosition = 0.0;
    }

    public default void updateInputs(ArmIOInputsAutoLogged inputs){}
    }

