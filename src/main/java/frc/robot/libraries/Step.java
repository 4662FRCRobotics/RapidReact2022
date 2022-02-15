package frc.robot.libraries;

import java.util.function.BooleanSupplier;

public class Step {
    String m_name;
    BooleanSupplier m_enabled;

    public Step(String name, BooleanSupplier enabled) {
        m_name = name;
        m_enabled = enabled;
    }

    public Step(String name) {
        this(name, () -> true);
    }

/*public Step(String name, String refName) {
    this(name, refName.getBoolenSupplier);
}*/



    public boolean isTrue() {
        return this.m_enabled.getAsBoolean(); 
    }

    public String getName() {
        return this.m_name;
    }
}
